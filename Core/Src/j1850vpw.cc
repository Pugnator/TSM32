#include "tsm.h"
#include "j1850.h"
#include <string.h>
#include <assert.h>
#include "dwtdelay.h"
#include "watchdog.h"

#if J1850_ENABLED
#define BIT_PER_BYTE 7

static volatile uint32_t riseEdgeTime = 0;
static volatile uint32_t fallEdgeTime = 0;
static volatile bool capturePolarityRising = true;
volatile bool messageStarted = false;
volatile bool messageCollected = false;

/* Live assembly buffer, ISR-only.  A completed frame is snapshotted into
 * payloadJ1850/j1850RXctr for the main loop, so reception continues while
 * the previous frame is being parsed (Fixes #74). */
static uint8_t rxAssembly[J1850_PAYLOAD_SIZE] = {0};
static volatile uint8_t rxByteCtr = 0;
static volatile uint8_t bitCounter = 0;

/* Completed-frame snapshot consumed by parseFrame()/printFrame(). */
uint8_t payloadJ1850[J1850_PAYLOAD_SIZE] = {0};
volatile uint8_t j1850RXctr = 0;
volatile uint32_t j1850DroppedFrames = 0;

uint16_t rpms = 0;
uint16_t kph = 0;
bool mil = 0;     // 0x88 frames with header priority 3 (fault/lamp channel)
bool milAux = 0;  // 0x88 frames with other priorities (periodic status)
bool sil = 0;     // 0x89 frames with header priority 6 (lamp-drive channel)
bool silAux = 0;  // 0x89 frames with header priority 7 (second channel, #80)
uint8_t dtc = 0;
uint32_t trip = 0;
int8_t gear_num = 0;       // 0=unknown, 1-6
bool in_neutral = false;
bool clutch_engaged = false;
uint8_t turn_signals = 0;  // 0=off 1=left 2=right 3=both
uint8_t engine_temp_f = 0; // degrees Fahrenheit
uint32_t fuel_ticks = 0;   // 0.000040 L per tick
uint8_t fuel_gauge_level = 0; // 0-15
bool passwordDtcSeen = false;
volatile bool securityPollPending = false; /* 0x93/0x2A poll awaiting our 0x92 reply */
bool ecmSeen         = false;
bool bcmDtcSeen      = false;
bool ipcDtcSeen      = false; // set when P1009/P1010 seen in DTC response

namespace J1850VPW
{
  static inline void J1850delayUS(uint32_t us) // microseconds
  {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);

    while (DWT->CYCCNT - startTick < delayTicks)
      ;
  }

  void messageReset()
  {
    /* Full RX reset (used around our own TX): clears both the live
     * assembly state and the completed-frame snapshot.  messageCollected
     * is released LAST so the main loop can never observe a half-cleared
     * snapshot (Fixes #47). */
    memset(rxAssembly, 0, sizeof(rxAssembly));
    memset(payloadJ1850, 0, sizeof(payloadJ1850));
    bitCounter = 0;
    rxByteCtr = 0;
    j1850RXctr = 0;
    messageStarted = false;
    __DMB();
    messageCollected = false;
  }

  /* Snapshot a byte-aligned completed frame for the main loop and rearm
   * the live assembly immediately, so the next frame on the bus is
   * received even while the previous one is still being parsed
   * (Fixes #74, #75).  ISR context only. */
  static void deliverCompletedFrame()
  {
    messageStarted = false;
    const uint8_t len = rxByteCtr;
    rxByteCtr = 0;
    if (len == 0 || bitCounter != 0 || len > 12)
    {
      bitCounter = 0; // glitch, partial byte or oversized: discard silently
      return;
    }
    if (messageCollected)
    {
      j1850DroppedFrames++; // main loop still busy with the previous frame
      return;
    }
    memcpy(payloadJ1850, rxAssembly, len);
    j1850RXctr = len;
    __DMB();
    messageCollected = true;
  }

  /* Called from the TIM3 EOF one-shot ISR (switch_ctrl.cc). */
  void onEofTimeout()
  {
    deliverCompletedFrame();
  }

  static void startEOFtimer()
  {
    __HAL_TIM_CLEAR_FLAG(&J1850_EOF_TIMER, TIM_SR_UIF);
    __HAL_TIM_SET_COUNTER(&J1850_EOF_TIMER, 0);
    HAL_TIM_Base_Start_IT(&J1850_EOF_TIMER);
  }

  /* Arbitration monitoring can be disabled for one blind retry when the
   * monitor itself is suspected of tripping on our own transceiver tail
   * (safety net so a miscalibrated monitor can never kill the TX path). */
  static bool arbMonitorEnabled_ = true;

  /* J1850 VPW is a CSMA bus: wait until the RX line has been passive for
   * at least one IFS before starting our SOF, so we do not stomp a frame
   * another node is transmitting (Fixes #62). Returns false if the bus
   * never goes quiet (line stuck active / heavy traffic) within ~20 ms. */
  static bool waitBusIdle()
  {
    const uint32_t ticksPerUs   = SystemCoreClock / 1000000;
    /* 280 us, not TX_IFS (300 us): the receive-side minimum IFS is 281 us
     * (RX_IFS_MIN), so demanding more can starve TX under sustained
     * near-spec traffic (Fixes #77). */
    const uint32_t idleTicks    = 280u * ticksPerUs;
    const uint32_t timeoutTicks = 20000u * ticksPerUs;
    const uint32_t waitStart    = DWT->CYCCNT;
    uint32_t idleStart          = DWT->CYCCNT;
    for (;;)
    {
      /* This wait can legitimately run up to 20 ms; keep the 65 ms WWDG
       * deadman satisfied while we are demonstrably alive. */
      watchdog_refresh();
      if (HAL_GPIO_ReadPin(J1850RX_GPIO_Port, J1850RX_Pin) == GPIO_PIN_SET)
      {
        idleStart = DWT->CYCCNT; // bus active: restart the idle window
      }
      else if (DWT->CYCCNT - idleStart >= idleTicks)
      {
        return true;
      }
      if (DWT->CYCCNT - waitStart >= timeoutTicks)
      {
        return false;
      }
    }
  }

  J1850error sendFrame(const uint8_t *data, uint8_t size)
  {
    if (!size || size > 11)
    {
      return J1850error::IncorrectFrame;
    }
    uint8_t crc = crc1850(data, size);
    if (!waitBusIdle())
    {
      return J1850error::LostArbitration;
    }
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_SET);
    J1850delayUS(TX_SOF);
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
    for (uint8_t i = 0; i < size; i++)
    {
      if (sendByte(data[i]) != J1850error::OK)
      {
        return J1850error::LostArbitration; // TX pin is already passive
      }
    }
    if (sendByte(crc) != J1850error::OK)
    {
      return J1850error::LostArbitration;
    }
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
    J1850delayUS(TX_EOF + TX_EOD);
    return J1850error::OK;
  }

  J1850error sendByte(const uint8_t byte)
  {
    uint8_t nbits = 8;
    uint8_t temp_ = byte;
    const uint32_t ticksPerUs = SystemCoreClock / 1000000;
    while (nbits--) // send 8 bits
    {
      if (nbits & 1) // start allways with passive symbol
      {
        const uint32_t delayTicks =
            ((temp_ & 0x80) ? TX_LONG : TX_SHORT) * ticksPerUs;
        /* VPW is CSMA/CR: while we drive passive, an active bus level means
         * another node is transmitting over us - yield so its frame
         * survives (Fixes #76).  VPW transitions are slow BY SPEC (~16 us
         * slew) and the receive comparator adds lag, so our own active
         * symbol's tail reads active well into the passive symbol: blank
         * the first 32 us and only declare a loss if the active level then
         * PERSISTS for 8 us (a real contender drives >=34 us symbols; the
         * field test with a 16 us blank and no persistence check tripped
         * on 100% of transmissions). */
        const uint32_t blankTicks   = 32u * ticksPerUs;
        const uint32_t persistTicks = 8u * ticksPerUs;
        const uint32_t start = DWT->CYCCNT;
        uint32_t activeSince = 0;
        bool     activeSeen  = false;
        HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
        while (DWT->CYCCNT - start < delayTicks)
        {
          if (!arbMonitorEnabled_ || (DWT->CYCCNT - start) <= blankTicks)
          {
            continue;
          }
          if (HAL_GPIO_ReadPin(J1850RX_GPIO_Port, J1850RX_Pin) == GPIO_PIN_SET)
          {
            if (!activeSeen)
            {
              activeSeen  = true;
              activeSince = DWT->CYCCNT;
            }
            else if (DWT->CYCCNT - activeSince >= persistTicks)
            {
              return J1850error::LostArbitration;
            }
          }
          else
          {
            activeSeen = false;
          }
        }
      }
      else // send active symbol
      {
        const uint32_t delay = (temp_ & 0x80) ? TX_SHORT : TX_LONG;
        HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_SET);
        J1850delayUS(delay);
      }
      temp_ <<= 1; // next bit
    }
    return J1850error::OK;
  }
}

#ifdef __cplusplus
extern "C"
{
#endif

  /*
  If the pulse duration falls within the range for a "Start Of Frame" pulse, the messageStarted flag is set to true and the LED is toggled.

  If the messageStarted flag is true, the pulse duration is compared to the ranges for "long" and "short" pulses.
  If the pulse duration falls within the range for a "long" pulse, a 0 is added to the current byte being constructed in the payloadJ1850 buffer.
  If the pulse duration falls within the range for a "short" pulse, a 1 is added to the current byte.
  The byteCounter and bitCounter variables are used to keep track of the current position in the payloadJ1850 buffer.
  If the pulse duration does not fall within any of the predefined ranges, a message is printed indicating that an unknown signal was received.
  */
  static inline void onFallingEdge(TIM_HandleTypeDef *htim)
  {
    fallEdgeTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    /* 16-bit modulo subtraction handles the counter wrap for any pulse
     * shorter than the 65.5 ms timer period, which every legal J1850
     * symbol is - no need to abort the frame on rise > fall (Fixes #78). */
    const uint32_t pulse = (fallEdgeTime - riseEdgeTime) & 0xFFFFu;

    // Start of Frame
    if (pulse <= RX_SOF_MAX && pulse > RX_SOF_MIN)
    {
      frameCounter++;
      TRACE_LOG("Start Of Frame, %uus\r\n", pulse);
      messageStarted = true;
      /* SOF begins a fresh frame: drop any stale partial-frame state left
       * behind by an EOD/EOF/IFS-classified gap or a glitch pulse, so old
       * bytes cannot corrupt this frame (Fixes #63). */
      bitCounter = 0;
      rxByteCtr = 0;
      memset(rxAssembly, 0, sizeof(rxAssembly));
      fallEdgeTime = 0;
      __HAL_TIM_SET_COUNTER(&J1850_IC_INSTANCE, 0);
      return;
    }

    if (!messageStarted)
    {
      // We saw no SOF and got something else here.
      return;
    }

    if (pulse <= RX_LONG_MAX && pulse > RX_LONG_MIN)
    {
      TRACE_LOG("Active 0, %uus\r\n", pulse);
      rxAssembly[rxByteCtr] &= ~(1UL << (BIT_PER_BYTE - bitCounter++));
    }
    else if (pulse <= RX_SHORT_MAX && pulse > RX_SHORT_MIN)
    {
      TRACE_LOG("Active 1, %uus\r\n", pulse);
      rxAssembly[rxByteCtr] |= 1UL << (BIT_PER_BYTE - bitCounter++);
    }
    else
    {
      TRACE_LOG("Unknown signal. Active, %uus\r\n", pulse);
    }
  }

  /*
  If the message has not started yet, the function returns early.
  Otherwise, the pulse duration (the time between the falling and rising edges of the pulse)
  is calculated and compared to various predefined thresholds. Depending on which threshold the pulse duration falls within,
  different actions are taken, such as logging the pulse duration and resetting the messageStarted flag to false.

  The function parses the pulse durations to extract data bits from the J1850 message.
  If the pulse duration falls within the range for a "long" pulse, a 1 is added to the current byte being constructed in the payloadJ1850 buffer.
  If the pulse duration falls within the range for a "short" pulse, a 0 is added to the current byte.
  The byteCounter and bitCounter variables are used to keep track of the current position in the payloadJ1850 buffer.
  */
  static inline void onRisingEdge(TIM_HandleTypeDef *htim)
  {
    riseEdgeTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    if (!messageStarted)
    {
      return;
    }

    /* Wrap-safe passive-gap width, same as in onFallingEdge (Fixes #78). */
    const uint32_t pulse = (riseEdgeTime - fallEdgeTime) & 0xFFFFu;
    if (pulse > RX_IFS_MIN)
    {
      TRACE_LOG("\r\nIFS, %uus\r\n", pulse);
      J1850VPW::deliverCompletedFrame();
    }
    else if (pulse > RX_EOF_MIN)
    {
      TRACE_LOG("\r\nEOF, %uus\r\n", pulse);
      J1850VPW::deliverCompletedFrame();
    }
    else if (RX_EOD_MAX >= pulse && pulse > RX_EOD_MIN)
    {
      TRACE_LOG("\r\nEOD, %uus\r\n", pulse);
      J1850VPW::deliverCompletedFrame();
    }
    else if (RX_LONG_MAX >= pulse && pulse > RX_LONG_MIN)
    {
      TRACE_LOG("Passive 1, %uus\r\n", pulse);
      rxAssembly[rxByteCtr] |= 1UL << (BIT_PER_BYTE - bitCounter++);
    }
    else if (RX_SHORT_MAX >= pulse && pulse > RX_SHORT_MIN)
    {
      TRACE_LOG("Passive 0, %uus\r\n", pulse);
      rxAssembly[rxByteCtr] &= ~(1UL << (BIT_PER_BYTE - bitCounter++));
    }
    else
    {
      TRACE_LOG("Unknown signal. Passive, %uus\r\n", pulse);
    }
  }

  void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
  {
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2)
    {
      return;
    }
#if J1850_ENABLED
    /* No messageCollected gate here: completed frames are snapshotted by
     * deliverCompletedFrame() and reception continues immediately, so
     * back-to-back bus frames are no longer dropped (Fixes #74). */
    if (capturePolarityRising)
    {
      __HAL_TIM_SET_COUNTER(&J1850_EOF_TIMER, 0);
      HAL_TIM_Base_Stop_IT(&J1850_EOF_TIMER);
      capturePolarityRising = false;
      onRisingEdge(htim);
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
      onFallingEdge(htim);
      capturePolarityRising = true;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
      // Start a check if it's the end of the frame
      J1850VPW::startEOFtimer();
    }
    if (bitCounter == 8)
    {
      TRACE_LOG("J1850: the bit counter == 8 [0x%.2X]\r\n", rxAssembly[rxByteCtr]);
      bitCounter = 0;
      rxByteCtr++;
      if (rxByteCtr >= J1850_PAYLOAD_SIZE)
      {
        TRACE_LOG("J1850: frame is too large: %u\r\n", J1850_PAYLOAD_SIZE);
        /* Reset only the live assembly - a pending snapshot stays valid. */
        rxByteCtr = 0;
        messageStarted = false;
      }
    }
#endif
  }

  uint8_t crc1850(const uint8_t *msg_buf, uint8_t nbytes)
  {
    if (0 == nbytes || 11 < nbytes)
    {
      return 0;
    }
    uint8_t crc_reg = 0xff, poly, byte_count, bit_count;
    const uint8_t *byte_point;
    uint8_t bit_point;

    for (byte_count = 0, byte_point = msg_buf; byte_count < nbytes; ++byte_count, ++byte_point)
    {
      for (bit_count = 0, bit_point = 0x80; bit_count < 8; ++bit_count, bit_point >>= 1)
      {
        if (bit_point & *byte_point) // case for new bit = 1
        {
          if (crc_reg & 0x80)
            poly = 1; // define the polynomial
          else
            poly = 0x1c;
          crc_reg = ((crc_reg << 1) | 1) ^ poly;
        }
        else // case for new bit = 0
        {
          poly = 0;
          if (crc_reg & 0x80)
            poly = 0x1d;
          crc_reg = (crc_reg << 1) ^ poly;
        }
      }
    }
    return ~crc_reg; // Return CRC
  }

  // Application TX entry point.  Briefly masks the input-capture interrupt
  // so the RX state machine cannot misinterpret our own bit-banged pulses,
  // then drives the frame via sendFrame() (which appends the CRC).
  // Returns true only if the frame actually went out on the bus; failures
  // are logged with PrintF so they are visible in production RTT
  // (Fixes #69).
  bool j1850TxRaw(const uint8_t *bytes, uint8_t len)
  {
    if (!bytes || len == 0 || len > 10)
    {
      PrintF("j1850 tx: invalid length (%u, max 10)\r\n", (unsigned)len);
      return false;
    }
    /* Carrier-sense while the input capture is still armed, so a frame
     * that is mid-flight during the wait is received, not destroyed
     * (Fixes #77).  sendFrame() re-checks idle just before the SOF. */
    if (!J1850VPW::waitBusIdle())
    {
      PrintF("[%lu] j1850 tx: bus busy, %u-byte frame not sent\r\n",
             (unsigned long)HAL_GetTick(), (unsigned)len);
      return false;
    }
    HAL_TIM_IC_Stop_IT(&J1850_IC_INSTANCE, TIM_CHANNEL_2);
    J1850VPW::messageReset();
    J1850VPW::J1850error rc = J1850VPW::sendFrame(bytes, len);
    if (rc == J1850VPW::J1850error::LostArbitration)
    {
      /* Either a genuine collision or the arbitration monitor tripping on
       * our own transceiver tail.  Wait out the (possible) foreign frame,
       * then retry ONCE with the monitor disabled so a miscalibrated
       * monitor can never kill the TX path entirely (field failure mode:
       * 100% LOST_ARB, no heartbeat on the bus, SIL lit). */
      PrintF("[%lu] j1850 tx: LOST_ARB, retrying blind\r\n",
             (unsigned long)HAL_GetTick());
      if (J1850VPW::waitBusIdle())
      {
        J1850VPW::arbMonitorEnabled_ = false;
        rc = J1850VPW::sendFrame(bytes, len);
        J1850VPW::arbMonitorEnabled_ = true;
      }
    }
    HAL_TIM_IC_Start_IT(&J1850_IC_INSTANCE, TIM_CHANNEL_2);
    if (rc != J1850VPW::J1850error::OK)
    {
      PrintF("[%lu] j1850 tx: %u bytes -> %s\r\n",
             (unsigned long)HAL_GetTick(), (unsigned)len,
             rc == J1850VPW::J1850error::IncorrectFrame ? "BAD_FRAME"
                                                        : "LOST_ARB");
    }
    return rc == J1850VPW::J1850error::OK;
  }

#ifdef __cplusplus
}
#endif
#endif