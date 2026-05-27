#include "tsm.h"
#include "j1850.h"

volatile uint32_t frameCounter = 0;
namespace J1850VPW
{
  // Bench-mode default: dump every successfully captured RX frame on the
  // RTT log channel.  Disable at runtime with 'j1850 trace off' once the
  // signal path is validated.
  volatile bool j1850TraceEnabled = true;

  sourceType convertByteToSourceType(uint8_t inputByte)
  {
    switch (inputByte)
    {
    case 0x10:
      return ECM;
    case 0x40:
      return TSM;
    case 0x1B:
      return RPM;
    case 0x29:
      return SPEED;
    case 0xC0:
      return IMMO;
    case 0xDA:
      return BLINKER;
    case 0x88:
      return MIL;
    case 0x89:
      return SIL;
    case 0x3B:
      return GEAR;
    case 0x49:
      return TEMP;
    case 0x61:
      return IPC;
    case 0x62:
      return HUD;
    case 0x69:
      return ODO;
    case 0x83:
      return FUEL;
    case 0xFE:
      return NET;
    case 0x93:
      return SECURITY;
    case 0x63:
      return VSC;
    case 0xFF:
      return ENGSTAT;
    default:
      // Handle unknown inputByte value
      // You can throw an exception, return a default value, or handle it in a different way based on your requirements.
      return ERROR;
    }
  }

  bool parseFrame()
  {
    static int32_t odolast = 0;
    if (j1850RXctr == 0 || j1850RXctr > 11)
    {
      WARN_LOG("Empty/corrupted frame [=%u]\r\n", j1850RXctr);
      return false;
    }
    uint8_t crc = crc1850(payloadJ1850, j1850RXctr - 1);
    if (crc != payloadJ1850[j1850RXctr - 1])
    {
      PrintF("J1850: CRC mismatch got=0x%02X expected=0x%02X rx=%u\r\n",
             (unsigned)payloadJ1850[j1850RXctr - 1], (unsigned)crc, (unsigned)j1850RXctr);
      return false;
    }
    j1850Header h;
    h.header = payloadJ1850[0];
    const uint8_t headerSize = h.ctx.type ? 1 : 3;
    if (headerSize == 1)
    {
      PrintF("J1850: 1-byte header frame rejected (not used on this network, rx=%u bytes)\r\n", j1850RXctr);
      return false;
    }

    auto destination = convertByteToSourceType(payloadJ1850[1]);

    if (destination == RPM)
    {
      rpms = payloadJ1850[headerSize + 1] << 8 | payloadJ1850[headerSize + 2];
      rpms /= 4;
      PrintF("RPMs: %u\r\n", rpms);
    }
    else if (destination == SPEED)
    {
      kph = payloadJ1850[headerSize + 1] << 8 | payloadJ1850[headerSize + 2];
      kph /= 128;
      PrintF("Speed: %u\r\n", kph);
    }
    else if (destination == MIL)
    {
      if (payloadJ1850[headerSize + 1] == 0x83)
      {
        // on
        mil = true;
      }
      else if (payloadJ1850[headerSize + 1] == 0x03)
      {
        // off
        mil = false;
      }
      else if (payloadJ1850[headerSize + 1] == 0x0E)
      {
        // unknown state
      }
      else
      {
      }
    }
    else if (destination == SIL)
    {
      if (payloadJ1850[headerSize + 1] == 0x83)
      {
        // on
        sil = true;
      }
      else if (payloadJ1850[headerSize + 1] == 0x03)
      {
        // off
        sil = false;
      }
      else if (payloadJ1850[headerSize + 1] == 0x0E)
      {
        // unknown state
      }
      else
      {
      }
    }
    else if (destination == ODO)
    {
      uint16_t speedSensorTicks = payloadJ1850[headerSize + 2] << 8 | payloadJ1850[headerSize + 3];
      odolast = speedSensorTicks - odolast;
      if (odolast < 0)
      {
        odolast += 65536;
      }
      trip += odolast;
      odolast = speedSensorTicks;
    }

    return true;
  }

  void printFrame()
  {
    if (!j1850TraceEnabled)
    {
      return;
    }
    if (j1850RXctr == 0 || j1850RXctr > 11)
    {
      PrintF("Empty/corrupted frame [=%u]\r\n", j1850RXctr);
      return;
    }

    // One-line dump for bench RX verification.  Format:
    //   [tick] #frame CRC=ok|BAD nB pri=p dst<-src : HH HH HH ...
    const uint8_t n = j1850RXctr;
    const uint8_t crc = crc1850(payloadJ1850, n - 1);
    const bool crcOk = (crc == payloadJ1850[n - 1]);

    j1850Header h;
    h.header = payloadJ1850[0];
    const uint8_t headerSize = h.ctx.type ? 1 : 3;

    char hex[3 * J1850_PAYLOAD_SIZE + 1];
    uint32_t off = 0;
    for (uint8_t i = 0; i < n && off + 3 < sizeof(hex); ++i)
    {
      static const char H[] = "0123456789ABCDEF";
      hex[off++] = H[(payloadJ1850[i] >> 4) & 0xF];
      hex[off++] = H[payloadJ1850[i] & 0xF];
      hex[off++] = ' ';
    }
    hex[off ? off - 1 : 0] = '\0';

    if (headerSize == 3)
    {
      INFO_LOG("[%lu] #%lu %s %uB pri=%u %s<-%s : %s\r\n",
               (unsigned long)HAL_GetTick(),
               (unsigned long)frameCounter,
               crcOk ? "OK " : "BAD",
               (unsigned)n,
               (unsigned)h.ctx.priority,
               sourceToStr(static_cast<sourceType>(payloadJ1850[1])),
               sourceToStr(static_cast<sourceType>(payloadJ1850[2])),
               hex);
    }
    else
    {
      INFO_LOG("[%lu] #%lu %s %uB pri=%u 1B-hdr : %s\r\n",
               (unsigned long)HAL_GetTick(),
               (unsigned long)frameCounter,
               crcOk ? "OK " : "BAD",
               (unsigned)n,
               (unsigned)h.ctx.priority,
               hex);
    }
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
    case BLINKER:
      return "Turn signal module";
    case MIL:
      return "Malfunction Indicator Lamp";
    case SIL:
      return "Security Indicator Lamp";
    case IPC:
      return "Instrument Cluster Panel";
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
}
