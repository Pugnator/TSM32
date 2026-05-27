#include "cli.h"
#include "trace.h"
#include "settings.h"
#include "j1850.h"
#include "id.h"

#include "stm32f1xx_hal.h"
#include "SEGGER_RTT.h"

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

namespace
{
  // Line buffer.  Sized for a single hex frame plus a generous command
  // prefix; anything longer than this gets truncated with a clear error.
  constexpr uint32_t CLI_LINE_MAX = 192u;
  char cliLine_[CLI_LINE_MAX];
  uint32_t cliLineLen_ = 0u;
  bool cliPrompted_ = false;

  void cliPrintBanner()
  {
    PrintF("\r\nTSM CLI ready.  Type 'help' for commands.\r\n");
  }

  void cliPrompt()
  {
    PrintF("tsm> ");
    cliPrompted_ = true;
  }

  void cliCmdHelp()
  {
    PrintF("Commands:\r\n"
           "  help                  this list\r\n"
           "  ver                   firmware version + feature flags\r\n"
           "  ahrs                  live yaw/pitch/roll + chip temperature\r\n"
           "  j1850                 J1850 link status (RX counters)\r\n"
           "  j1850 trace on|off    auto-dump every RX frame on this CLI\r\n"
           "  j1850 tx HH HH ...    send one frame (CRC appended), max 10 bytes\r\n"
           "  j1850 clear           send DTC-clear request (6C 00 F1 14)\r\n"
           "  reset                 software CPU reset\r\n");
  }

  void cliCmdVer()
  {
    uint32_t id[3] = {0};
    getCPUid(id, STM32F1_t);
    PrintF("TSM %s %s (%s)\r\n",
           VERSION_BUILD_DATE, VERSION_TAG, VERSION_BUILD);
    PrintF("CPU ID  : %.8lx%.8lx%.8lx\r\n", id[0], id[1], id[2]);
    PrintF("Features: J1850_ENABLED=%d MEMS_ENABLED=%d BLINKER_ENABLED=%d\r\n",
           (int)J1850_ENABLED, (int)MEMS_ENABLED, (int)BLINKER_ENABLED);
  }

  void cliCmdAhrs()
  {
#if MEMS_ENABLED
    int16_t y = 0, p = 0, r = 0;
    cliGetYprDeg(&y, &p, &r);
    PrintF("YPR     : %d / %d / %d  deg\r\n", (int)y, (int)p, (int)r);
    PrintF("Chip T  : %.2f C\r\n", (double)cliGetChipTemperatureC());
#else
    PrintF("AHRS not built in (MEMS_ENABLED=0)\r\n");
#endif
  }

  void cliCmdJ1850(char *args)
  {
#if J1850_ENABLED
    // Strip leading whitespace from the sub-command, if any.
    while (args && (*args == ' ' || *args == '\t'))
      ++args;

    if (!args || *args == '\0')
    {
      PrintF("J1850   : enabled\r\n"
             "  rx bytes        : %u\r\n"
             "  frames received : %u\r\n"
             "  message ready   : %s\r\n"
             "  auto trace      : %s\r\n"
             "  last MIL/SIL    : %d / %d\r\n",
             (unsigned)j1850RXctr,
             (unsigned)frameCounter,
             messageCollected ? "yes" : "no",
             J1850VPW::j1850TraceEnabled ? "on" : "off",
             (int)mil, (int)sil);
      return;
    }

    // Split sub-command vs. its tail.
    char *sub = args;
    char *tail = args;
    while (*tail && *tail != ' ' && *tail != '\t')
      ++tail;
    if (*tail)
    {
      *tail = '\0';
      ++tail;
    }

    if (!strcmp(sub, "trace"))
    {
      while (*tail == ' ' || *tail == '\t')
        ++tail;
      if (!strcmp(tail, "on"))
      {
        J1850VPW::j1850TraceEnabled = true;
        PrintF("j1850: trace ON\r\n");
      }
      else if (!strcmp(tail, "off"))
      {
        J1850VPW::j1850TraceEnabled = false;
        PrintF("j1850: trace OFF\r\n");
      }
      else
      {
        PrintF("usage: j1850 trace on|off\r\n");
      }
      return;
    }

    if (!strcmp(sub, "tx"))
    {
      // Parse a sequence of hex bytes (whitespace-separated or run-on).
      uint8_t frame[11];
      uint8_t n = 0;
      const char *p = tail;
      while (*p && n < sizeof(frame))
      {
        while (*p == ' ' || *p == '\t')
          ++p;
        if (!*p)
          break;
        unsigned v = 0;
        int digits = 0;
        while (digits < 2 && ((*p >= '0' && *p <= '9') ||
                              (*p >= 'a' && *p <= 'f') ||
                              (*p >= 'A' && *p <= 'F')))
        {
          v <<= 4;
          if (*p <= '9')
            v |= (unsigned)(*p - '0');
          else if (*p <= 'F')
            v |= (unsigned)(*p - 'A' + 10);
          else
            v |= (unsigned)(*p - 'a' + 10);
          ++p;
          ++digits;
        }
        if (digits == 0)
        {
          PrintF("j1850 tx: bad hex near '%s'\r\n", p);
          return;
        }
        frame[n++] = (uint8_t)v;
      }
      if (n == 0)
      {
        PrintF("usage: j1850 tx HH HH ... (1-10 bytes)\r\n");
        return;
      }
      cliJ1850TxRaw(frame, n);
      return;
    }

    if (!strcmp(sub, "clear"))
    {
      // Standard KWP-on-J1850 \"clear DTC\" request: priority 6C, dest 00,
      // src F1 (tester), service 14.
      static const uint8_t clr[4] = {0x6C, 0x00, 0xF1, 0x14};
      cliJ1850TxRaw(clr, sizeof(clr));
      return;
    }

    PrintF("unknown j1850 sub-command '%s'.  Try 'help'.\r\n", sub);
#else
    (void)args;
    PrintF("J1850   : disabled at build time (J1850_ENABLED=0)\r\n");
#endif
  }

  void cliCmdReset()
  {
    PrintF("Reset...\r\n");
    // Give the RTT host time to drain the FIFO before we tear the core down.
    for (volatile uint32_t i = 0; i < 100000u; ++i)
    {
      __NOP();
    }
    NVIC_SystemReset();
  }

  void cliDispatch(char *line)
  {
    // Trim leading whitespace.
    while (*line == ' ' || *line == '\t')
      ++line;
    if (*line == '\0')
      return;

    // First token is the command.
    char *cmd = line;
    char *args = line;
    while (*args && *args != ' ' && *args != '\t')
      ++args;
    if (*args)
    {
      *args = '\0';
      ++args;
    }

    if (!strcmp(cmd, "help") || !strcmp(cmd, "?"))
      cliCmdHelp();
    else if (!strcmp(cmd, "ver"))
      cliCmdVer();
    else if (!strcmp(cmd, "ahrs"))
      cliCmdAhrs();
    else if (!strcmp(cmd, "j1850"))
      cliCmdJ1850(args);
    else if (!strcmp(cmd, "reset"))
      cliCmdReset();
    else
      PrintF("Unknown command: '%s'.  Try 'help'.\r\n", cmd);
  }
}

extern "C" void cliPoll(void)
{
  if (!cliPrompted_)
  {
    cliPrintBanner();
    cliPrompt();
  }

  // Drain whatever the host pushed into the down-channel since the last
  // poll.  No more than one full line is processed per call, which keeps
  // the main loop latency bounded if a script dumps many lines at once.
  char ch;
  while (SEGGER_RTT_HasKey())
  {
    int c = SEGGER_RTT_GetKey();
    if (c < 0)
      break;
    ch = (char)c;

    if (ch == '\r' || ch == '\n')
    {
      PrintF("\r\n");
      if (cliLineLen_ > 0u)
      {
        cliLine_[cliLineLen_] = '\0';
        cliDispatch(cliLine_);
      }
      cliLineLen_ = 0u;
      cliPrompt();
      return;
    }

    if (ch == 0x7f || ch == '\b')
    {
      if (cliLineLen_ > 0u)
      {
        --cliLineLen_;
        PrintF("\b \b");
      }
      continue;
    }

    if (ch < 0x20 || ch > 0x7e)
      continue; // ignore non-printable junk

    if (cliLineLen_ + 1u >= CLI_LINE_MAX)
    {
      // Drop the byte and complain.  The host can hit Enter to recover.
      if (cliLineLen_ + 1u == CLI_LINE_MAX)
      {
        PrintF("\r\n[cli: line too long, discarding]\r\n");
        cliLineLen_ = CLI_LINE_MAX; // mark over-length; cleared on Enter
      }
      continue;
    }

    cliLine_[cliLineLen_++] = ch;
    SEGGER_RTT_PutChar(LOGGING_CHANNEL, ch); // local echo
  }
}

// Default weak implementations.  The real ones live in tsm.cc and override
// these whenever the AHRS pipeline is actually present in the build.
extern "C" __attribute__((weak)) float cliGetChipTemperatureC(void)
{
  return 0.0f;
}

extern "C" __attribute__((weak)) void cliGetYprDeg(int16_t *yaw, int16_t *pitch, int16_t *roll)
{
  if (yaw)
    *yaw = 0;
  if (pitch)
    *pitch = 0;
  if (roll)
    *roll = 0;
}

extern "C" __attribute__((weak)) void cliJ1850TxRaw(const uint8_t *bytes, uint8_t len)
{
  (void)bytes;
  (void)len;
  PrintF("j1850 tx: J1850 not built in (J1850_ENABLED=0)\r\n");
}
