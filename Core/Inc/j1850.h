#pragma once
#include <stdint.h>

#define J1850_PAYLOAD_SIZE 64

#ifdef __cplusplus
extern "C"
{
#endif

  extern uint16_t rpms;
  extern uint16_t kph;
  extern bool mil;
  extern bool sil;
  extern uint8_t dtc;
  extern uint32_t trip;

  extern uint8_t payloadJ1850[J1850_PAYLOAD_SIZE];
  extern volatile uint8_t j1850RXctr;
  extern volatile uint32_t frameCounter;
  extern volatile bool messageCollected;

  uint8_t crc1850(const uint8_t *msg_buf, uint8_t nbytes);
#ifdef __cplusplus
}
#endif
namespace J1850VPW
{

  enum class J1850error
  {
    OK,
    IncorrectFrame,
    LostArbitration
  };

  typedef union j1850Header
  {
    struct
    {
      uint8_t spectype : 2;
      uint8_t mode : 1;
      uint8_t ifr : 1;
      uint8_t type : 1;
      uint8_t priority : 3;
    } ctx;
    uint8_t header;
  } j1850Header;

  typedef enum sourceType
  {
    ERROR = 0,
    ECM = 0x10,
    GAUGE = 0x10,
    TSM = 0x40,
    RPM = 0x1B,
    SPEED = 0x29,
    IMMO = 0xC0,
    BLINKER = 0xDA,
    MIL = 0x88,
    SIL = 0x89,
    GEAR = 0x3b,
    TEMP = 0x49,
    IPC = 0x61,
    HUD = 0x62,
    ODO = 0x69,
    FUEL = 0x83,
    NET = 0xFE,
    SECURITY = 0x93,
    VSC = 0x63,
    ENGSTAT = 0xFF
  } sourceType;

  const char *sourceToStr(sourceType type);
  void printFrame();
  bool parseFrame();
  J1850error sendFrame(const uint8_t *data, uint8_t size);
  J1850error sendByte(const uint8_t byte);
  void messageReset();

// define J1850 VPW timing requirements in accordance with SAE J1850 standard
// all width times in us
// transmitting pulse width
#define TX_SHORT 64 // Short pulse nominal time
#define TX_LONG 128 // Long pulse nominal time
#define TX_SOF 200  // Start Of Frame nominal time
#define TX_EOD 200  // End Of Data nominal time
#define TX_EOF 280  // End Of Frame nominal time
#define TX_BRK 300  // Break nominal time
#define TX_IFS 300  // Inter Frame Separation nominal time

// see SAE J1850 chapter 6.6.2.5 for preferred use of In Frame Respond/Normalization pulse
#define TX_IFR_SHORT_CRC 64   // short In Frame Respond, IFR contain CRC
#define TX_IFR_LONG_NOCRC 128 // long In Frame Respond, IFR contain no CRC

// receiving pulse width
#define RX_SHORT_MIN 34 // minimum short pulse time
#define RX_SHORT_MAX 96 // maximum short pulse time

#define RX_LONG_MIN 97  // minimum long pulse time
#define RX_LONG_MAX 163 // maximum long pulse time

#define RX_SOF_MIN 164 // minimum start of frame time
#define RX_SOF_MAX 239 // maximum start of frame time

#define RX_EOD_MIN 164 // minimum end of data time
#define RX_EOD_MAX 239 // maximum end of data time

#define RX_EOF_MIN 240 // minimum end of frame time, ends at minimum IFS
#define RX_BRK_MIN 400 // minimum break time
#define RX_IFS_MIN 281 // minimum inter frame separation time, ends at next SOF

// see chapter 6.6.2.5 for preferred use of In Frame Respond/Normalization pulse
#define RX_IFR_SHORT_MIN 34 // minimum short in frame respond pulse time
#define RX_IFR_SHORT_MAX 96 // maximum short in frame respond pulse time
#define RX_IFR_LONG_MIN 96  // minimum long in frame respond pulse time
#define RX_IFR_LONG_MAX 163 // maximum long in frame respond pulse time
}