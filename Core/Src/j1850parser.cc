#include "tsm.h"
#include "j1850.h"

volatile uint32_t frameCounter = 0;
namespace J1850VPW
{
  // Bench-mode default: dump every successfully captured RX frame on the
  // RTT log channel.
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

  struct DtcTableEntry { char code[6]; const char *desc; };
  static const DtcTableEntry dtcTable[] = {
    {"B0563", "Battery Voltage High"},
    {"B1004", "Fuel Level Sending Unit Low"},
    {"B1005", "Fuel Level Sending Unit High/Open"},
    {"B1006", "Accessory Line Overvoltage"},
    {"B1007", "Ignition Line Overvoltage"},
    {"B1008", "Reset Switch Closed"},
    {"B1131", "Alarm Output Low"},
    {"B1132", "Alarm Output High"},
    {"B1134", "Starter Output High"},
    {"B1135", "Accelerometer Fault"},
    {"B1151", "Sidecar BAS Low"},
    {"B1152", "Sidecar BAS High"},
    {"B1153", "Sidecar BAS Out of Range"},
    {"P0106", "MAP Sensor Rate of Range Error"},
    {"P0107", "MAP Sensor Open/Low"},
    {"P0108", "MAP Sensor High"},
    {"P0112", "IAT Sensor Voltage Low"},
    {"P0113", "IAT Sensor Voltage Open/High"},
    {"P0117", "ET Sensor Voltage Low"},
    {"P0118", "ET Sensor Voltage Open/High"},
    {"P0122", "TP Sensor Open/Low"},
    {"P0123", "TP Sensor High"},
    {"P0261", "Front Injector Open/Low"},
    {"P0262", "Front Injector High"},
    {"P0263", "Rear Injector Open/Low"},
    {"P0264", "Rear Injector High"},
    {"P0373", "CKP Sensor Intermittent"},
    {"P0374", "CKP Sensor Not Detected/Synch Error"},
    {"P0501", "VSS Low"},
    {"P0502", "VSS High/Open"},
    {"P0505", "Loss of Idle Speed Control"},
    {"P0562", "Battery Voltage Low"},
    {"P0563", "Battery Voltage High"},
    {"P0602", "Calibration Memory Error"},
    {"P0603", "ECM EEPROM Error"},
    {"P0604", "RAM Failure"},
    {"P0605", "Program/Flash Memory Error"},
    {"P0607", "Converter Error"},
    {"P1001", "System Relay Coil Open/Low"},
    {"P1002", "System Relay Coil High/Shorted"},
    {"P1003", "System Relay Contacts Open"},
    {"P1004", "System Relay Contacts Closed"},
    {"P1009", "Incorrect Password"},
    {"P1010", "Missing Password"},
    {"P1351", "Front Ignition Coil Open/Low"},
    {"P1352", "Front Ignition Coil High/Shorted"},
    {"P1353", "Front Cylinder No Combustion"},
    {"P1354", "Rear Ignition Coil Open/Low"},
    {"P1355", "Rear Ignition Coil High/Shorted"},
    {"P1356", "Rear Cylinder No Combustion"},
    {"P1357", "Intermittent Secondary Front"},
    {"P1358", "Intermittent Secondary Rear"},
    {"U1016", "Loss of ECM Serial Data"},
    {"U1064", "Loss of TSM/TSSM Serial Data"},
    {"U1097", "Loss of Speedometer Serial Data"},
    {"U1255", "Serial Data Error/Missing Message"},
    {"U1300", "Serial Data Low"},
    {"U1301", "Serial Data Open/High"},
  };

  static const char *dtcLookup(const char *code)
  {
    for (uint8_t i = 0; i < sizeof(dtcTable) / sizeof(dtcTable[0]); ++i)
      if (__builtin_memcmp(dtcTable[i].code, code, 5) == 0)
        return dtcTable[i].desc;
    return nullptr;
  }

  bool parseFrame()
  {
    static int32_t odolast = 0;
    static bool odoSeen = false;
    /* SAE J1850 frames are at most 12 bytes including CRC (Fixes #64). */
    if (j1850RXctr == 0 || j1850RXctr > 12)
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

    // Mark ECM as present the first time we see a valid 3-byte-header frame
    // from 0x10 (headerSize is always 3 here - the 1-byte case returned above).
    if (payloadJ1850[2] == 0x10)
      ecmSeen = true;

    auto destination = convertByteToSourceType(payloadJ1850[1]);

    /* Every decode branch below must check j1850RXctr before indexing:
     * the buffer is zero-filled, so a short (but CRC-valid) frame would
     * otherwise silently decode zeros into rpms/kph, which feed the
     * starter-lock FSM (Fixes #64). */
    if (destination == RPM)
    {
      if (j1850RXctr >= 7)
      {
        rpms = payloadJ1850[headerSize + 1] << 8 | payloadJ1850[headerSize + 2];
        rpms /= 4;
      }
    }
    else if (destination == SPEED)
    {
      if (j1850RXctr >= 7)
      {
        kph = payloadJ1850[headerSize + 1] << 8 | payloadJ1850[headerSize + 2];
        kph /= 128;
      }
    }
    else if (destination == MIL && j1850RXctr >= 5)
    {
      /* Like the SIL, the ECM broadcasts two 0x88 streams that differ only
       * in header priority: pri=3 carries the fault/lamp state, pri=6 is a
       * periodic status that reads "off" - merging them made the flag
       * toggle ~1 Hz while the physical lamp was steady (field log
       * 2026-08-22).  State byte bit7: 1 = lamp ON. */
      const bool newMil = (payloadJ1850[headerSize] & 0x80) != 0;
      bool *slot = (h.ctx.priority == 3) ? &mil : &milAux;
      if (newMil != *slot)
      {
        *slot = newMil;
        PrintF("[%lu] MIL pri=%u -> %s\r\n",
               (unsigned long)HAL_GetTick(), (unsigned)h.ctx.priority,
               newMil ? "ON" : "off");
      }
    }
    else if (destination == SIL && j1850RXctr >= 5)
    {
      /* The IPC broadcasts two distinct 0x89 streams that differ only in
       * header priority (observed pri=6 and pri=7 with opposite states
       * during the 1 Hz flash pattern) - track them separately (#80). */
      const bool newSil = (payloadJ1850[headerSize] & 0x80) != 0;
      bool *slot = (h.ctx.priority == 7) ? &silAux : &sil;
      if (newSil != *slot)
      {
        *slot = newSil;
        PrintF("[%lu] SIL pri=%u -> %s\r\n",
               (unsigned long)HAL_GetTick(), (unsigned)h.ctx.priority,
               newSil ? "ON (fault/armed)" : "off");
      }
    }
    else if (destination == BLINKER)
    {
      // 48 da 40 39 xx : turn signals, xx = 1 left, 2 right, 3 both, 0 off
      if (payloadJ1850[headerSize] == 0x39 && j1850RXctr >= 6)
        turn_signals = payloadJ1850[headerSize + 1] & 0x03;
    }
    else if (destination == GEAR)
    {
      if (payloadJ1850[2] == 0x40 && j1850RXctr >= 5)
      {
        // 48 3b 40 xx : neutral=bit7, clutch=bit7 (per HarleyDroid 0xA0=neutral, 0x20=not neutral)
        in_neutral     = (payloadJ1850[headerSize] & 0x80) != 0;
        clutch_engaged = (payloadJ1850[headerSize] & 0x80) != 0;
      }
      else if (payloadJ1850[headerSize] == 0x03 && j1850RXctr >= 6)
      {
        // a8 3b 10 03 xx : current gear bitmask (1,3,7,15,31,63 = gears 1-6)
        const uint8_t mask = payloadJ1850[headerSize + 1];
        int8_t g = 0;
        uint8_t tmp = mask;
        while (tmp) { tmp >>= 1; g++; }
        gear_num = g;
      }
    }
    else if (destination == TEMP)
    {
      // a8 49 10 10 xx : engine temperature in degrees Fahrenheit
      if (payloadJ1850[headerSize] == 0x10 && j1850RXctr >= 6)
      {
        engine_temp_f = payloadJ1850[headerSize + 1];
      }
    }
    else if (destination == FUEL)
    {
      if (payloadJ1850[2] == 0x10)
      {
        // a8 83 10 0a/8a xx xx : fuel consumption ticks (0.000040 L each)
        const uint8_t subfn = payloadJ1850[headerSize];
        if ((subfn & 0x7F) == 0x0A && j1850RXctr >= 7)
        {
          /* ticks must be 32-bit; subfn bit7 indicates a 65536-tick wrap.
           * The previous uint16_t accumulator made the +=65536 a no-op
           * and silently dropped 2.6 L per wrap event (Fixes #44). */
          uint32_t ticks = ((uint32_t)payloadJ1850[headerSize + 1] << 8)
                         |  (uint32_t)payloadJ1850[headerSize + 2];
          if (subfn & 0x80)
            ticks += 65536u; // wraparound
          fuel_ticks += ticks;
        }
      }
      else if (payloadJ1850[2] == 0x61)
      {
        // a8 83 61 12 dx : fuel gauge level x = 0-15
        if (payloadJ1850[headerSize] == 0x12 && j1850RXctr >= 6)
        {
          fuel_gauge_level = payloadJ1850[headerSize + 1] & 0x0F;
        }
      }
    }
    else if (destination == SECURITY && j1850RXctr >= 5)
    {
      /* Security poll to function 0x93 (our bike: 69 93 61 2A from the
       * IPC).  A donor capture from a bike with a live TSM shows the TSM
       * answering on the mirror address 0x92: 48 92 40 2A 82 22 F2 /
       * 48 92 40 AA FF FF 5B, payload type matching the 0x93 side.  Flag
       * the poll so tsm.cc can send the response (#81). */
      PrintF("[%lu] security poll 0x93 from 0x%02X type=0x%02X\r\n",
             (unsigned long)HAL_GetTick(), (unsigned)payloadJ1850[2],
             (unsigned)payloadJ1850[headerSize]);
      if (payloadJ1850[headerSize] == 0x2A)
        securityPollPending = true;
    }
    else if (destination == ODO && j1850RXctr >= 7)
    {
      // a8 69 10 06/86 xx xx : odometer ticks (0.4 m each; bit7 of subfn = wraparound)
      uint16_t speedSensorTicks = (uint16_t)(payloadJ1850[headerSize + 1] << 8) | payloadJ1850[headerSize + 2];
      if (!odoSeen)
      {
        /* First sample only seeds the reference: the counter is the ECM's
         * free-running value, not distance travelled since boot (Fixes #65). */
        odoSeen = true;
        odolast = speedSensorTicks;
      }
      else
      {
        int32_t delta = speedSensorTicks - odolast;
        if (delta < 0)
          delta += 65536;
        trip += (uint32_t)delta;
        odolast = speedSensorTicks;
      }
    }

    // KWP2000 positive response to ReadDTCByStatus (service 0x59).
    // Frame: 6C F1 <src> 59 [<dtc_hi> <dtc_lo>] ...
    // Minimum 5 bytes (3-byte header + 0x59 + CRC): with only 4 bytes the
    // 0x59 candidate would be the CRC itself and the dtcBytes subtraction
    // below would underflow to 255, over-reading the buffer (Fixes #64).
    if (j1850RXctr >= 5 && payloadJ1850[headerSize] == 0x59)
    {
      const uint8_t src = payloadJ1850[2];
      /* Raw dump first: this is the primary diagnostic record for the
       * SIL/MIL investigation (#81) - never rely on the decode alone. */
      {
        static const char hex[] = "0123456789ABCDEF";
        char raw[3 * 12 + 1];
        uint8_t p = 0;
        for (uint8_t i = 0; i < j1850RXctr && i < 12; ++i)
        {
          raw[p++] = hex[payloadJ1850[i] >> 4];
          raw[p++] = hex[payloadJ1850[i] & 0x0F];
          raw[p++] = ' ';
        }
        raw[p ? p - 1 : 0] = '\0';
        PrintF("[%lu] DTC 0x59 from 0x%02X raw: %s\r\n",
               (unsigned long)HAL_GetTick(), (unsigned)src, raw);
      }
      // Each DTC is 2 bytes; they start at offset headerSize+1.
      const uint8_t dtcBytes = j1850RXctr - 1 - headerSize - 1; // exclude header + service byte + CRC
      if (dtcBytes < 2)
      {
        PrintF("  no codes stored\r\n");
      }
      else
      {
        const uint8_t count = dtcBytes / 2;
        static const char typeChar[] = {'P', 'C', 'B', 'U'};
        static const char hex[] = "0123456789ABCDEF";
        for (uint8_t i = 0; i < count; ++i)
        {
          const uint8_t hi = payloadJ1850[headerSize + 1 + i * 2];
          const uint8_t lo = payloadJ1850[headerSize + 2 + i * 2];
          if (hi == 0x00 && lo == 0x00)
            continue; // P0000 = no fault, skip
          char code[6];
          code[0] = typeChar[(hi >> 6) & 0x03];
          code[1] = hex[(hi >> 4) & 0x03];
          code[2] = hex[hi & 0x0F];
          code[3] = hex[(lo >> 4) & 0x0F];
          code[4] = hex[lo & 0x0F];
          code[5] = '\0';
          // P1009 (hi=0x10,lo=0x09) = Incorrect Password
          // P1010 (hi=0x10,lo=0x10) = Missing Password
          if (hi == 0x10 && (lo == 0x09 || lo == 0x10))
            passwordDtcSeen = true;
          // Track per-module DTC presence for targeted clears.
          if (src == 0x40)
            bcmDtcSeen = true;
          else if (src == 0x61)
            ipcDtcSeen = true;
          const char *desc = dtcLookup(code);
          if (desc)
            PrintF("  %s: %s\r\n", code, desc);
          else
            PrintF("  %s\r\n", code);
        }
        /* An odd trailing byte is the KWP statusOfDTC for the (single) code
         * in this frame - the current-vs-historic discriminator (#81). */
        if (dtcBytes & 1)
        {
          PrintF("  status byte: 0x%02X\r\n",
                 (unsigned)payloadJ1850[headerSize + 1 + count * 2]);
        }
      }
    }

    return true;
  }

  void printFrame()
  {
    if (!j1850TraceEnabled)
      return;
    if (j1850RXctr == 0 || j1850RXctr > 12)
      return; // suppress corrupted-frame noise

    const uint8_t n       = j1850RXctr;
    const uint8_t crc_cal = crc1850(payloadJ1850, n - 1);
    const bool    crcOk   = (crc_cal == payloadJ1850[n - 1]);
    j1850Header h;
    h.header              = payloadJ1850[0];
    const uint8_t hs      = h.ctx.type ? 1 : 3;

    // Only log frames addressed to TSM (0x40), SIL (0x89), or SECURITY (0x93).
    // When J1850_BUS_TRACE is enabled, log every frame regardless of destination.
#if !J1850_BUS_TRACE
    if (hs != 3)
      return;
    {
      const uint8_t dst = payloadJ1850[1];
      if (dst != 0x40 && dst != 0x89 && dst != 0x93)
        return;
    }
#endif

    // Print the frame header prefix (no newline yet).
    if (hs == 3)
      DEBUG_LOG("[%lu] #%lu %s %uB pri=%u %s<-%s : ",
             (unsigned long)HAL_GetTick(), (unsigned long)frameCounter,
             crcOk ? "OK " : "BAD", (unsigned)n, (unsigned)h.ctx.priority,
             sourceToStr(static_cast<sourceType>(payloadJ1850[1])),
             sourceToStr(static_cast<sourceType>(payloadJ1850[2])));
    else
      DEBUG_LOG("[%lu] #%lu %s %uB pri=%u 1B-hdr : ",
             (unsigned long)HAL_GetTick(), (unsigned long)frameCounter,
             crcOk ? "OK " : "BAD", (unsigned)n, (unsigned)h.ctx.priority);

    // For 3-byte-header frames with valid CRC, try to decode the payload.
    bool known = false;
    if (hs == 3 && crcOk && n >= 4)
    {
      const sourceType dst  = convertByteToSourceType(payloadJ1850[1]);
      const uint8_t    src  = payloadJ1850[2];
      const uint8_t   *d    = &payloadJ1850[hs]; // d[0] = first data byte
      const uint8_t    dlen = n - hs - 1;         // data bytes, CRC excluded
      (void)src;

      switch (dst)
      {
      case RPM:
        if (dlen >= 3)
        {
          DEBUG_LOG("RPM=%u", (unsigned)(((uint16_t)d[1] << 8 | d[2]) / 4));
          known = true;
        }
        break;

      case SPEED:
        if (dlen >= 3)
        {
          DEBUG_LOG("Speed=%u km/h", (unsigned)(((uint16_t)d[1] << 8 | d[2]) / 128));
          known = true;
        }
        break;

      case MIL:
        if (dlen >= 1)
        {
          DEBUG_LOG("MIL=%s", (d[0] & 0x80) ? "ON" : "off");
          known = true;
        }
        break;

      case SIL:
        if (dlen >= 1)
        {
          DEBUG_LOG("SIL=%s", (d[0] & 0x80) ? "ON (fault/armed)" : "off");
          known = true;
        }
        break;

      case BLINKER:
        if (dlen >= 2 && d[0] == 0x39)
        {
          [[maybe_unused]] static const char *const sigN[] = {"off", "left", "right", "both"};
          DEBUG_LOG("Turn=%s", sigN[d[1] & 0x03]);
          known = true;
        }
        break;

      case GEAR:
        if (src == 0x40 && dlen >= 1)
        {
          // 48 3b 40 xx : bit7=neutral+clutch
          DEBUG_LOG("%s clutch=%s",
                 (d[0] & 0x80) ? "neutral" : "engaged",
                 (d[0] & 0x80) ? "in" : "out");
          known = true;
        }
        else if (src == 0x10 && dlen >= 2 && d[0] == 0x03)
        {
          // a8 3b 10 03 xx : bitmask 1,3,7,15,31,63 = gears 1-6
          uint8_t tmp = d[1]; int8_t g = 0;
          while (tmp) { tmp >>= 1; g++; }
          DEBUG_LOG("Gear=%d", (int)g);
          known = true;
        }
        break;

      case TEMP:
        if (dlen >= 2 && d[0] == 0x10)
        {
          DEBUG_LOG("Temp=%uF / %dC",
                 (unsigned)d[1], ((int)d[1] - 32) * 5 / 9);
          known = true;
        }
        break;

      case FUEL:
        if (src == 0x10 && dlen >= 3 && (d[0] & 0x7F) == 0x0A)
        {
          DEBUG_LOG("Fuel ticks=%u",
                 (unsigned)(((uint16_t)d[1] << 8) | d[2]));
          known = true;
        }
        else if (src == 0x61 && dlen >= 2 && d[0] == 0x12)
        {
          DEBUG_LOG("Fuel gauge=%u/15", (unsigned)(d[1] & 0x0F));
          known = true;
        }
        break;

      case ODO:
        if (dlen >= 3)
        {
          DEBUG_LOG("Odo=%u ticks",
                 (unsigned)(((uint16_t)d[1] << 8) | d[2]));
          known = true;
        }
        break;

      default:
        // KWP2000 DTC positive response (service 0x59)
        if (dlen >= 1 && d[0] == 0x59)
        {
          DEBUG_LOG("DTC reply src=0x%02X", (unsigned)src);
          known = true;
        }
        break;
      }
    }

    if (!known)
    {
      // Raw hex dump for unrecognised frames.
      [[maybe_unused]] static const char H[] = "0123456789ABCDEF";
      for (uint8_t i = 0; i < n; ++i)
      {
        DEBUG_LOG("%c%c", H[payloadJ1850[i] >> 4], H[payloadJ1850[i] & 0xF]);
        if (i + 1 < n) DEBUG_LOG(" ");
      }
    }

    DEBUG_LOG("\r\n");
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
    case SCANNER:
      return "Scanner";
    case GEAR:
      return "Gear selector";
    case TSM:
      return "TSM";
    case HUD:
      return "HUD";
    case IMMO:
      return "Immobilizer";
    default:
      return "Unknown source";
    }
  }
}
