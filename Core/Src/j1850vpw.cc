#include "tsm.h"
#include "j1850.h"
#include <string.h>
#include <assert.h>
#include "dwtdelay.h"

#if J1850_ENABLED
#define BIT_PER_BYTE 7

static volatile uint32_t riseEdgeTime = 0;
static volatile uint32_t fallEdgeTime = 0;
static volatile bool capturePolarityRising = true;
volatile bool messageStarted = false;
volatile bool messageCollected = false;
volatile uint8_t j1850RXctr = 0;
static volatile uint8_t bitCounter = 0;
uint8_t payloadJ1850[J1850_PAYLOAD_SIZE] = {0};

uint16_t rpms = 0;
uint16_t kph = 0;
bool mil = 0;
bool sil = 0;
uint8_t dtc = 0;
uint32_t trip = 0;

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
    messageCollected = false;
    memset(payloadJ1850, 0, sizeof(payloadJ1850));
    bitCounter = 0;
    j1850RXctr = 0;
    messageStarted = false;
  }

  static void startEOFtimer()
  {
    __HAL_TIM_CLEAR_FLAG(&J1850_EOF_TIMER, TIM_SR_UIF);
    __HAL_TIM_SET_COUNTER(&J1850_EOF_TIMER, 0);
    HAL_TIM_Base_Start_IT(&J1850_EOF_TIMER);
  }

  J1850error sendFrame(const uint8_t *data, uint8_t size)
  {
    if (!size || size > 11)
    {
      return J1850error::IncorrectFrame;
    }
    uint8_t crc = crc1850(data, size);
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_SET);
    J1850delayUS(TX_SOF);
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
    for (uint8_t i = 0; i < size; i++)
    {
      sendByte(data[i]);
    }
    sendByte(crc);
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
    J1850delayUS(TX_EOF + TX_EOD);
    return J1850error::OK;
  }

  J1850error sendByte(const uint8_t byte)
  {
    // If TX line is Active - arbitration is lost
    // if (HAL_GPIO_ReadPin(J1850RX_GPIO_Port, J1850RX_Pin) == GPIO_PIN_SET)
    {
    }

    uint8_t nbits = 8;
    uint8_t temp_ = byte;
    uint32_t delay;
    while (nbits--) // send 8 bits
    {
      if (nbits & 1) // start allways with passive symbol
      {
        delay = (temp_ & 0x80) ? TX_LONG : TX_SHORT; // send correct pulse lenght
        HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
        J1850delayUS(delay);
      }
      else // send active symbol
      {
        delay = (temp_ & 0x80) ? TX_SHORT : TX_LONG; // send correct pulse lenght
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

    // Check if it's an unrealisting time;
    if (riseEdgeTime > fallEdgeTime)
    {
      J1850VPW::messageReset();
      return;
    }

    const uint32_t pulse = fallEdgeTime - riseEdgeTime;

    // Start of Frame
    if (pulse <= RX_SOF_MAX && pulse > RX_SOF_MIN)
    {
      frameCounter++;
      DEBUG_LOG("Start Of Frame, %uus\r\n", pulse);
      messageStarted = true;
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
      DEBUG_LOG("Active 0, %uus\r\n", pulse);
      payloadJ1850[j1850RXctr] &= ~(1UL << (BIT_PER_BYTE - bitCounter++));
    }
    else if (pulse <= RX_SHORT_MAX && pulse > RX_SHORT_MIN)
    {
      DEBUG_LOG("Active 1, %uus\r\n", pulse);
      payloadJ1850[j1850RXctr] |= 1UL << (BIT_PER_BYTE - bitCounter++);
    }
    else
    {
      DEBUG_LOG("Unknown signal. Active, %uus\r\n", pulse);
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

    const uint32_t pulse = riseEdgeTime - fallEdgeTime;
    if (pulse > RX_IFS_MIN)
    {
      DEBUG_LOG("\r\nIFS, %uus\r\n", pulse);
      messageStarted = false;
    }
    else if (pulse > RX_EOF_MIN)
    {
      DEBUG_LOG("\r\nEOF, %uus\r\n", pulse);
      messageStarted = false;
    }
    else if (RX_EOD_MAX >= pulse && pulse > RX_EOD_MIN)
    {
      DEBUG_LOG("\r\nEOD, %uus\r\n", pulse);
      messageStarted = false;
    }
    else if (RX_LONG_MAX >= pulse && pulse > RX_LONG_MIN)
    {
      DEBUG_LOG("Passive 1, %uus\r\n", pulse);
      payloadJ1850[j1850RXctr] |= 1UL << (BIT_PER_BYTE - bitCounter++);
    }
    else if (RX_SHORT_MAX >= pulse && pulse > RX_SHORT_MIN)
    {
      DEBUG_LOG("Passive 0, %uus\r\n", pulse);
      payloadJ1850[j1850RXctr] &= ~(1UL << (BIT_PER_BYTE - bitCounter++));
    }
    else
    {
      DEBUG_LOG("Unknown signal. Passive, %uus\r\n", pulse);
    }
  }

  void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
  {
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2)
    {
      return;
    }
#if J1850_ENABLED
    // Wait for message to be processed

    if (messageCollected)
    {
      return;
    }

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
      DEBUG_LOG("J1850: the bit counter == 8 [0x%.2X]\r\n", payloadJ1850[j1850RXctr]);
      bitCounter = 0;
      j1850RXctr++;
      if (j1850RXctr > J1850_PAYLOAD_SIZE)
      {
        DEBUG_LOG("J1850: frame is too large: %u\r\n", J1850_PAYLOAD_SIZE);
        J1850VPW::messageReset();
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

#ifdef __cplusplus
}
#endif
#endif