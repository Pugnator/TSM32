#include "tsm.h"
#include "j1850.h"
#include <string.h>
#include <assert.h>

static volatile uint32_t riseEdgeTime = 0;
static volatile uint32_t fallEdgeTime = 0;
static volatile bool isRisingEdge = true;
volatile bool messageStarted = false;
volatile bool messageCollected = false;
volatile uint8_t byteCounter = 0;
static volatile uint8_t bitCounter = 0;
static volatile uint32_t frameCounter = 0;
static volatile uint32_t badFrameCounter = 0;

#define BIT_PER_BYTE 7

uint8_t payloadJ1850[12] = {0};

static void stop_tim3()
{
  HAL_TIM_Base_Stop_IT(&htim3);
}

static void start_tim3()
{
  stop_tim3();
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_SR_UIF);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  HAL_TIM_Base_Start_IT(&htim3);
}

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
  case TSM:
    return "Turn signal module";
  case CHECK_ENGINE:
    return "Check engine light";
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
  uint8_t crc = j1850Crc(payloadJ1850, byteCounter - 1);
  PrintF("Frame #%u [CRC: 0x%02X] %s\r\n", frameCounter, crc, crc == payloadJ1850[byteCounter - 1] ? "VALID" : "IVALID!");
  for (uint8_t i = 0; i < byteCounter; i++)
  {
    PrintF("0x%02X ", payloadJ1850[i]);
  }
  Print("\r\n");
  if (crc != payloadJ1850[byteCounter - 1] || byteCounter < 3)
  {
    return;
  }
  j1850Header h;
  h.header = payloadJ1850[0];
  PrintF("HEADER\r\nPriority: %u\r\n", h.priority);
  PrintF("%u bytes header\r\n", h.type ? 1 : 3);
  PrintF("Message to '%s'\r\n", sourceToStr(payloadJ1850[1]));
  PrintF("Message from '%s'\r\n", sourceToStr(payloadJ1850[2]));
  PrintF("*********************\r\n");
}

void messageReset()
{
  memset(payloadJ1850, 0, sizeof(payloadJ1850));
  bitCounter = 0;
  byteCounter = 0;
  messageStarted = false;
}

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

static inline void onFallingEdge(TIM_HandleTypeDef *htim)
{
  fallEdgeTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

  uint32_t pulse = fallEdgeTime - riseEdgeTime;
  // Start of Frame
  if (pulse <= RX_SOF_MAX && pulse > RX_SOF_MIN)
  {
    // HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
    frameCounter++;
    // PrintF("Start Of Frame, %uus\r\n", pulse);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    messageStarted = true;
  }

  if (!messageStarted)
  {
    return;
  }
  if (pulse <= RX_LONG_MAX && pulse > RX_LONG_MIN)
  {
    // PrintF("Active 0, %uus\r\n", pulse);
    payloadJ1850[byteCounter] &= ~(1UL << (BIT_PER_BYTE - bitCounter++));
  }
  else if (pulse <= RX_SHORT_MAX && pulse > RX_SHORT_MIN)
  {
    // PrintF("Active 1, %uus\r\n", pulse);
    payloadJ1850[byteCounter] |= 1UL << (BIT_PER_BYTE - bitCounter++);
  }
}

static inline void onRisingEdge(TIM_HandleTypeDef *htim)
{
  riseEdgeTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

  if (!messageStarted)
  {
    return;
  }

  uint32_t pulse = riseEdgeTime - fallEdgeTime;
  if (pulse > RX_IFS_MIN)
  {
    // PrintF("\r\nIFS, %uus\r\n", pulse);
    messageStarted = false;
  }
  else if (pulse > RX_EOF_MIN)
  {
    PrintF("EOF, %uus\r\n", pulse);
    messageStarted = false;
  }
  else if (RX_EOD_MAX >= pulse && pulse > RX_EOD_MIN)
  {
    PrintF("EOD, %uus\r\n", pulse);
    messageStarted = false;
  }
  else if (RX_LONG_MAX >= pulse && pulse > RX_LONG_MIN)
  {
    // PrintF("Passive 1, %uus\r\n", pulse);
    payloadJ1850[byteCounter] |= 1UL << (BIT_PER_BYTE - bitCounter++);
  }
  else if (RX_SHORT_MAX >= pulse && pulse > RX_SHORT_MIN)
  {
    // PrintF("Passive 0, %uus\r\n", pulse);
    payloadJ1850[byteCounter] &= ~(1UL << (BIT_PER_BYTE - bitCounter++));
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2)
  {
    return;
  }

  // Wait for message to be processed
  if (messageCollected)
  {
    return;
  }

  if (isRisingEdge)
  {
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Base_Stop_IT(&htim3);
    onRisingEdge(htim);
    isRisingEdge = false;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
  }
  else
  {
    onFallingEdge(htim);
    isRisingEdge = true;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
    start_tim3();
  }
  if (bitCounter == 8)
  {
    bitCounter = 0;
    byteCounter++;
  }
  assert(byteCounter < 11);
}

void j1850_break()
{
  HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, 1);
  HAL_Delay(5);
  HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, 0);
}