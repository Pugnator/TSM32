#include "tsm.h"
#include "j1850.h"
#include <string.h>
#include <assert.h>
#include "dwtdelay.h"

#ifdef __cplusplus
extern "C"
{
#endif

  static volatile uint32_t riseEdgeTime = 0;
  static volatile uint32_t fallEdgeTime = 0;
  static volatile bool isRisingEdge = true;
  volatile bool messageStarted = false;
  volatile bool messageCollected = false;
  volatile bool rxQueryNotEmpty = false;
  volatile uint8_t j1850RXCtr = 0;
  static volatile uint8_t bitCounter = 0;
  static volatile uint32_t frameCounter = 0;
  static volatile uint32_t badFrameCounter = 0;

#define BIT_PER_BYTE 7

  uint8_t payloadJ1850[PAYLOAD_SIZE] = {0};
  uint8_t sendBufJ1850[PAYLOAD_SIZE] = {0};
  size_t sendBufLen = 0;

  static void startEOFtimer()
  {
    __HAL_TIM_CLEAR_FLAG(&J1850_EOF_TIMER, TIM_SR_UIF);
    __HAL_TIM_SET_COUNTER(&J1850_EOF_TIMER, 0);
    HAL_TIM_Base_Start_IT(&J1850_EOF_TIMER);
  }
#if J1850_ENABLED
  const char *sourceToStr(sourceType type)
  {
    switch (type)
    {
    case ECM:
      return "Engine Control Module";
    case RPM:
      return "RPMs";
    case SPEED:
      return "Speedometer";
    case BLINKER:
      return "Turn signal module";
    case MIL:
      return "Malfunction Indicator Lamp";
    case TEMP:
      return "Engine temperature";
    case ODO:
      return "Odometer";
    case FUEL:
      return "Fuel gauge";
    case ENGSTAT:
      return "Engine status";
    case NET:
      return "Network Control";
    case SECURITY:
      return "Vehicle Security";
    case VSC:
      return "Vehicle Speed Control";
    default:
      return "Unknown source";
    }
  }

  void printFrameJ1850()
  {
    if (j1850RXCtr <= 4)
    {
      INFO_LOG("printFrameJ1850: byteCounter <= 4 [%u]\r\n", j1850RXCtr);
      return;
    }
    uint8_t crc = j1850Crc(payloadJ1850, j1850RXCtr - 1);
    INFO_LOG("Frame #%u [CRC: 0x%02X] %s\r\n", frameCounter, crc, crc == payloadJ1850[j1850RXCtr - 1] ? "VALID" : "INVALID!");
    for (uint8_t i = 0; i < j1850RXCtr; i++)
    {
      INFO_LOG("0x%02X ", payloadJ1850[i]);
    }
    INFO_LOG("\r\n");
    if (crc != payloadJ1850[j1850RXCtr - 1])
    {
      return;
    }
    j1850Header h;
    h.header = payloadJ1850[0];
    /*
    } else if ((x & 0xff0fffff) == 0x6c00f114) {
        if (D) Log.d(TAG, "DTC clear request");
      } else if ((x & 0xffff0fff) == 0x6cf10054) {
        if (D) Log.d(TAG, "DTC clear reply");
      } else
    */
    INFO_LOG("HEADER\r\nPriority: %u\r\n", h.ctx.priority);
    INFO_LOG("%u bytes header\r\n", h.ctx.type ? 1 : 3);
    INFO_LOG("Message to '%s'\r\n", sourceToStr(static_cast<sourceType>(payloadJ1850[1])));
    INFO_LOG("Message from '%s'\r\n", sourceToStr(static_cast<sourceType>(payloadJ1850[2])));
    INFO_LOG("*********************\r\n");

    if (frameCounter == 10)
    {
      // const uint8_t msg[] = {0x6C, 0x10, 0xF1, 0x19, 0xF3};
      //  sendCommandJ1850(msg, 5);
    }
    // RPM = (hex2dec(XX)*256+hex2dec(YY)) / 4
    // KpH = (hex2dec(XX)*256+hex2dec(YY)) / 128
  }

  void messageReset()
  {
    memset(payloadJ1850, 0, sizeof(payloadJ1850));
    bitCounter = 0;
    j1850RXCtr = 0;
    messageStarted = false;
  }

  /* Thanks to B. Roadman's web site for this CRC code */
  uint8_t j1850Crc(uint8_t *msg_buf, int8_t nbytes)
  {
    if (0 == nbytes || 11 < nbytes)
    {
      return 0;
    }
    uint8_t crc_reg = 0xff, poly, byte_count, bit_count;
    uint8_t *byte_point;
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

  /*
  The pulse duration (the time between the falling and rising edges of the pulse) is calculated and compared to various predefined thresholds.
  If the pulse duration falls within the range for a "Start Of Frame" pulse, the messageStarted flag is set to true and the LED is toggled.

  If the messageStarted flag is true, the pulse duration is compared to the ranges for "long" and "short" pulses.
  If the pulse duration falls within the range for a "long" pulse, a 0 is added to the current byte being constructed in the payloadJ1850 buffer.
  If the pulse duration falls within the range for a "short" pulse, a 1 is added to the current byte.
  The byteCounter and bitCounter variables are used to keep track of the current position in the payloadJ1850 buffer.
  If the pulse duration does not fall within any of the predefined ranges, a message is printed indicating that an unknown signal was received.
  */
  static inline void onPassivePulse(TIM_HandleTypeDef *htim)
  {
    fallEdgeTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    // Check if it's an unrealisting time;
    if (riseEdgeTime > fallEdgeTime)
    {
      DEBUG_LOG("Capture overflow\r\n");
      return;
    }

    const uint32_t pulse = fallEdgeTime - riseEdgeTime;
    // Start of Frame
    if (pulse <= RX_SOF_MAX && pulse > RX_SOF_MIN)
    {
      frameCounter++;
      DEBUG_LOG("Start Of Frame, %uus\r\n", pulse);
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      messageStarted = true;
      fallEdgeTime = 0;
      __HAL_TIM_SET_COUNTER(&htim5, 0);

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
      payloadJ1850[j1850RXCtr] &= ~(1UL << (BIT_PER_BYTE - bitCounter++));
    }
    else if (pulse <= RX_SHORT_MAX && pulse > RX_SHORT_MIN)
    {
      DEBUG_LOG("Active 1, %uus\r\n", pulse);
      payloadJ1850[j1850RXCtr] |= 1UL << (BIT_PER_BYTE - bitCounter++);
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
  static inline void onActivePulse(TIM_HandleTypeDef *htim)
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
      payloadJ1850[j1850RXCtr] |= 1UL << (BIT_PER_BYTE - bitCounter++);
    }
    else if (RX_SHORT_MAX >= pulse && pulse > RX_SHORT_MIN)
    {
      DEBUG_LOG("Passive 0, %uus\r\n", pulse);
      payloadJ1850[j1850RXCtr] &= ~(1UL << (BIT_PER_BYTE - bitCounter++));
    }
    else
    {
      DEBUG_LOG("Unknown signal. Passive, %uus\r\n", pulse);
    }
  }

  void sendCommandJ1850(const uint8_t *data, size_t size)
  {
    sendBufLen = size;
    memcpy(sendBufJ1850, data, size);
    // sendBufJ1850[sendBufLen] = j1850Crc(sendBufJ1850, sendBufLen);
    // sendBufLen++;
    Print("##########################\r\n");
    Print("Sending message\r\n");
    for (uint8_t i = 0; i < sendBufLen; i++)
    {
      DEBUG_LOG("0x%02X ", sendBufJ1850[i]);
    }
    Print("\r\n##########################\r\n");
    rxQueryNotEmpty = true;
  }
#endif

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
      DEBUG_LOG("The message is still in the queue. Ignore the new one\r\n");
      return;
    }

    if (isRisingEdge)
    {
      __HAL_TIM_SET_COUNTER(&J1850_EOF_TIMER, 0);
      HAL_TIM_Base_Stop_IT(&J1850_EOF_TIMER);
      isRisingEdge = false;
      onActivePulse(htim);
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
      onPassivePulse(htim);
      isRisingEdge = true;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
      // Start a check if it's the end of the frame
      startEOFtimer();
    }
    if (bitCounter == 8)
    {
      DEBUG_LOG("J1850: the bit counter == 8 [0x%.2X]\r\n", payloadJ1850[j1850RXCtr]);
      bitCounter = 0;
      j1850RXCtr++;
      if (j1850RXCtr > PAYLOAD_SIZE)
      {
        DEBUG_LOG("J1850: frame is too large: %u\r\n", PAYLOAD_SIZE);
        messageReset();
      }
    }
#endif
  }

  void j1850SendMessage()
  {
#if J1850_ENABLED
    bool bitActive = false;
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_SET);
    DWT_Delay(TX_SOF);
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
    for (size_t i = 0; i < sendBufLen; i++)
    {
      size_t bit = 7;
      uint8_t temp = sendBufJ1850[i];
      while (bit >= 0)
      {
        if (temp & 0x01)
        {
          // 1
          // DEBUG_LOG("bit %d is 1\n", bit);
          if (bitActive)
          {
            HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_SET);
            DWT_Delay(TX_SHORT);
          }
          else
          {
            HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
            DWT_Delay(TX_LONG);
          }
        }
        else
        {
          // 0
          // DEBUG_LOG("bit %d is 0\n", bit);
          if (bitActive)
          {
            HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_SET);
            DWT_Delay(TX_LONG);
          }
          else
          {
            HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
            DWT_Delay(TX_SHORT);
          }
        }

        bit--;
        bitActive = !bitActive;
        temp = temp >> 1;
      }
    }
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
    rxQueryNotEmpty = false;
    sendBufLen = 0;
#endif
  }

#ifdef __cplusplus
}
#endif