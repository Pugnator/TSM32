#include "tsm.h"
#include "settings.h"
#include "j1850.h"

#if J1850_ENABLED
#include "engine_state.h"
#endif

#if MEMS_ENABLED
#include "ahrs.h"
#endif

#include <stdio.h>
#include "id.h"
#include "assert.h"
#include "dwtdelay.h"
#include "watchdog.h"
#include "eeprom.h"
#include "security.h"

bool stopAppExecuting = true;

#if MEMS_ENABLED
Quaternion mpuQ;
float ypr[3];
float yprDeg[3];
bool trackingEnabled = false;
int16_t initialYaw = INT16_MIN;
uint32_t initialTime = 0;
#endif

#ifdef __cplusplus
extern "C"
{
#endif

  void HAL_IncTick(void)
  {
    uwTick += uwTickFreq;
  }

#if MEMS_ENABLED
  static bool detectTurn(int16_t initialYaw, int16_t currentYaw, int16_t threshold)
  {
    int16_t yaw_difference = currentYaw - initialYaw;
    if (yaw_difference > 180)
      yaw_difference -= 360;
    else if (yaw_difference < -180)
      yaw_difference += 360;

    if (abs(yaw_difference) > threshold)
    {
      return true;
    }
    return false;
  }
#endif

  void tsmRunApp()
  {
    /* Sample the settings-menu gesture first: both buttons held while the
     * ignition comes on. */
    const bool settingsRequested =
        LEFT_BUTTON == PRESSED && RIGHT_BUTTON == PRESSED;

    uint32_t id[3] = {0};
    getCPUid(id, STM32F1_t);
    BANNER("Device ID %.8lx%.8lx%.8lx\r\nTSM %s %s (%s) started\r\n",
           id[0], id[1], id[2],
           VERSION_BUILD_DATE, VERSION_TAG, VERSION_BUILD);

    /* Report why the previous boot ended (IWDG hang / fault / power-on) then
     * arm the IWDG BEFORE the slow init below, so a hang in init is caught as
     * well as one in the main loop. Every init phase and the loop refresh
     * within the ~2 s timeout. */
    const watchdog_reset_cause_t resetCause = watchdog_report_reset_cause();
    watchdog_init();

    /* Load the flash-emulated EEPROM (security PIN and future settings)
     * before anything can ask for a stored value. Fast: two page scans. */
    if (!ee_init())
    {
      PrintF("EEPROM: store unavailable\r\n");
    }

    startupSettingsHandler();
    watchdog_refresh();

/*Battery watchdog*/
#if AUTO_LIGHT_ENABLE
    HAL_ADC_Start_DMA(&hadc1, adcDMAbuffer, ADC_DMA_BUF_SIZE);
#endif

/*J1850 logger*/
#if J1850_ENABLED
    HAL_TIM_IC_Start_IT(&J1850_IC_INSTANCE, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(J1850TX_GPIO_Port, J1850TX_Pin, GPIO_PIN_RESET);
#endif
    /*Blinker bulb PWM*/
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    /* The relay initializes low in gpio.c. Only a real ignition/power cycle
     * may release it (fail-lock), and a configured security PIN keeps it low
     * until the PIN is entered - securityInit() applies both policies. */
#if BLINKER_ENABLED && SECURITY_PIN_ENABLED
    securityInit(settingsRequested, watchdog_reset_allows_starter(resetCause));
#else
    /* Immobilizer disabled (SECURITY_PIN_ENABLED=0) or no buttons/lamps in this
     * build: the PIN gate does not apply. Only the reset-cause fail-lock
     * remains - enable the starter on a genuine power-on, keep it latched off
     * otherwise. */
    (void)settingsRequested;
    if (watchdog_reset_allows_starter(resetCause))
    {
      enableStarter();
    }
    else
    {
      disableStarter();
    }
#endif

    leftSideOff();
    rightSideOff();

// uint8_t frame[2] = {0xAA, 0xAA};
// J1850VPW::sendFrame(frame, 2);
#if DEBUG
    uint32_t prevSample = HAL_GetTick();
#endif

#if MEMS_ENABLED
    /* The one AHRS instance lives for the whole run, so it is a function-local
     * static, not a heap object: constructed right here (after MX_SPI1_Init,
     * unlike a file-scope static) and never destroyed. This removed the only
     * dynamic allocation in the firmware and with it the 4 KB vmmu pool. The
     * pointer alias keeps the mpu-> call sites below unchanged. */
    static Ahrs::AhrsBase<Imu::Bus> mpuInstance(IMU_BUS_HANDLE, true);
    Ahrs::AhrsBase<Imu::Bus> *const mpu = &mpuInstance;
    PrintF("MEMS: MPU9250 %s init %s\r\n",
           Imu::kBusName,
           mpu->ok() ? "OK" : "FAILED - check bus / wiring");
#endif
    watchdog_refresh(); // IMU/DMP upload is the slowest init phase
    stopAppExecuting = false;
    while (!stopAppExecuting)
    {
      watchdog_refresh();
#if AUTO_LIGHT_ENABLE
      adcHandler();
#endif

#if J1850_ENABLED
      if (messageCollected)
      {
        J1850VPW::printFrame();
        J1850VPW::parseFrame();
        /* Release only the snapshot - the ISR keeps receiving into its own
         * assembly buffer the whole time (Fixes #74). */
        messageCollected = false;
      }

      /* Frames that completed while the previous snapshot was unprocessed. */
      {
        static uint32_t lastDropReport = 0;
        if (j1850DroppedFrames != lastDropReport)
        {
          lastDropReport = j1850DroppedFrames;
          PrintF("[%lu] J1850 RX overrun: %lu frame(s) dropped total\r\n",
                 (unsigned long)HAL_GetTick(), (unsigned long)lastDropReport);
        }
      }

      Engine::handler();

      // Security-poll responder: the IPC polls function 0x93 with type 0x2A
      // (69 93 61 2A on this bike) and a genuine TSM answers on the mirror
      // address 0x92 - donor capture: 48 92 40 2A 82 22 F2 (idle variant
      // 48 92 40 AA FF FF 5B).  Answer each poll, rate-limited (#81).
      if (securityPollPending)
      {
        static uint32_t secLastTick = 0;
        const uint32_t  now_sec     = HAL_GetTick();
        securityPollPending = false;
        if (secLastTick == 0 || (now_sec - secLastTick) >= 200u)
        {
          static const uint8_t secResp[] = {0x48, 0x92, 0x40, 0x2A, 0x82, 0x22};
          PrintF("[%lu] security reply -> 48 92 40 2A 82 22\r\n",
                 (unsigned long)now_sec);
          j1850TxRaw(secResp, sizeof(secResp));
          secLastTick = HAL_GetTick();
        }
      }

      // TSM presence broadcast: emulate what a genuine TSM/TSSM puts on the
      // bus so the IPC's U1064 "Loss of TSM/TSSM Serial Data" watchdog stays
      // satisfied and the security lamp stays off.  Two frames, both sourced
      // from 0x40, once the bus is active then every 2 s:
      //   68 FF 40 03  - module-status broadcast to function 0xFF.  Every
      //                  other module sends its own (ECM 68 FF 10 03, IPC
      //                  68 FF 61 03, HUD 68 FF 62 03); the TSM's is
      //                  68 FF 40 03 D8, confirmed CRC-valid against a real
      //                  bus capture.  This is the frame the IPC most likely
      //                  keys presence on (#79/#81).
      //   29 FE 40 01  - network-control presence, kept as belt-and-braces.
      // First broadcast goes out immediately on bus wake-up (the presence
      // deadline is only ~3 s) and a failed TX is retried quickly instead of
      // waiting a full period.
      {
        static uint32_t hbLastTick  = 0;
        static uint32_t hbInterval  = 0;
        static bool     hbStarted   = false;
        const uint32_t  now_hb      = HAL_GetTick();
        if (!hbStarted && frameCounter >= 1)
        {
          hbStarted  = true;
          hbLastTick = now_hb;
          hbInterval = 0; // fire on this iteration
        }
        if (hbStarted && (now_hb - hbLastTick) >= hbInterval)
        {
          static const uint8_t status[] = {0x68, 0xFF, 0x40, 0x03};
          static const uint8_t netctl[] = {0x29, 0xFE, 0x40, 0x01};
          const bool s1 = j1850TxRaw(status, sizeof(status));
          const bool s2 = j1850TxRaw(netctl, sizeof(netctl));
          hbLastTick = HAL_GetTick();
          hbInterval = (s1 && s2) ? 2000u : 250u;
        }
      }

#if SECURITY_FLASH_SIL && SECURITY_PIN_ENABLED
      // EXPERIMENTAL: while the immobilizer is locked/waiting, try to flash
      // the cluster security lamp by toggling an 0x89 SIL frame ~1 Hz. See
      // the SECURITY_SIL_* notes in settings.h - unconfirmed on hardware, so
      // every TX is logged for the bench session. Note this fights the
      // "disarmed" security handshake above; if the lamp bounces instead of
      // flashing, the cluster is honouring the IPC's own state and we need
      // the armed-handshake variant, not a direct SIL broadcast.
      {
        const security_state_t ss = securityState();
        const bool silActive = (ss == SECURITY_STATE_LOCKED ||
                                ss == SECURITY_STATE_LOCKOUT);
        static bool     silOn       = false;
        static bool     silWasActive = false;
        static uint32_t silLastTick = 0;
        const uint32_t  now_sil     = HAL_GetTick();
        if (silActive)
        {
          if (!silWasActive || (now_sil - silLastTick) >= SECURITY_SIL_FLASH_MS)
          {
            silOn = !silWasActive ? true : !silOn;
            const uint8_t sil[] = {SECURITY_SIL_HDR, 0x89, SECURITY_SIL_SRC,
                                   (uint8_t)(silOn ? 0x83 : 0x03)};
            PrintF("[%lu] SIL flash -> %02X 89 %02X %02X (%s)\r\n",
                   (unsigned long)now_sil, (unsigned)SECURITY_SIL_HDR,
                   (unsigned)SECURITY_SIL_SRC, (unsigned)(silOn ? 0x83 : 0x03),
                   silOn ? "ON" : "off");
            j1850TxRaw(sil, sizeof(sil));
            silLastTick = now_sil;
          }
        }
        else if (silWasActive)
        {
          /* Just unlocked/idle: command the lamp off once so it doesn't
           * stick lit if the last frame we sent was ON. */
          const uint8_t off[] = {SECURITY_SIL_HDR, 0x89, SECURITY_SIL_SRC, 0x03};
          j1850TxRaw(off, sizeof(off));
          silOn = false;
        }
        silWasActive = silActive;
      }
#endif

      // DTC monitor: once the bus is active (>=5 frames), query ECM(0x10)
      // and both IPC address candidates (0x60, 0x61 - see #72, the log will
      // show which one answers) with 200 ms gaps, log what THIS cycle's
      // responses reported, then cool down 10 s and repeat.  Production code
      // intentionally never transmits service 0x14: automatic clearing can
      // erase unrelated diagnostic history.  0x40 is not queried because it
      // is our own address.
      //
      // States: 0=wait for bus  1-3=querying modules
      //         4=report what was observed  5=cooldown
      {
        /* Wrap-safe deadlines: store the start tick and an interval, then
         * compare via unsigned subtraction. Raw HAL_GetTick() >= deadline
         * comparisons stalled or fired continuously across the 49.7-day
         * tick wrap (Fixes #49). */
        static uint8_t  dtcState     = 0;
        static uint32_t dtcLastTick  = 0;
        static uint32_t dtcInterval  = 0;

        // Modules to query: ECM(0x10), IPC candidates (0x60, 0x61)
        static const uint8_t dtcTargets[] = {0x10, 0x60, 0x61};

        const uint32_t now = HAL_GetTick();

        /* SIL observation for the discriminator below (#81): how long has
         * the pri-6 lamp channel been continuously ON? */
        static uint32_t silOnSince   = 0;
        static uint32_t lastSilCycle = 0;
        if (!sil)
          silOnSince = 0;
        else if (silOnSince == 0)
          silOnSince = now;
        const bool silHeld = silOnSince != 0 && (now - silOnSince) >= 1000u;

        if (dtcState == 0 && frameCounter >= 5)
        {
          dtcState    = 1;
          dtcLastTick = now;
          dtcInterval = 200;
        }
        else if (dtcState >= 1 && dtcState <= 3)
        {
          if ((now - dtcLastTick) >= dtcInterval)
          {
            if (dtcState == 1)
            {
              /* Fresh cycle: forget the previous cycle's responses so the
               * react state reflects the present, not history (#71). */
              passwordDtcSeen = false;
              bcmDtcSeen      = false;
              ipcDtcSeen      = false;
            }
            const uint8_t target = dtcTargets[dtcState - 1];
            const uint8_t req[7] = {0x6C, target, 0xF1, 0x19, 0x52, 0xFF, 0x00};
            if (j1850TxRaw(req, sizeof(req)))
            {
              dtcState    = (dtcState < 3) ? dtcState + 1 : 4;
              dtcInterval = 200;
            }
            else
            {
              dtcInterval = 250; // TX failed: retry the same target (#70)
            }
            dtcLastTick = HAL_GetTick();
          }
        }
        else if (dtcState == 4 && (now - dtcLastTick) >= dtcInterval)
        {
          if (passwordDtcSeen)
          {
            PrintF("[%lu] ECM password DTC present; automatic clear disabled\r\n",
                   (unsigned long)HAL_GetTick());
          }
          if (ipcDtcSeen || silHeld)
          {
            PrintF("[%lu] IPC diagnostic attention (%s); automatic clear disabled\r\n",
                   (unsigned long)HAL_GetTick(),
                   ipcDtcSeen ? "codes stored" : "SIL lit, no codes reported");
          }
          dtcState    = 5;
          dtcLastTick = HAL_GetTick();
          dtcInterval = 10000;
        }
        else if (dtcState == 5)
        {
          // If ECM just appeared on the bus, skip the rest of the cooldown
          // so the query-and-clear cycle fires immediately instead of waiting
          // up to 10 s (cuts the SIL-on window from ~16 s to ~2 s).
          static bool ecmWasSeen = false;
          if (ecmSeen && !ecmWasSeen)
          {
            ecmWasSeen  = true;
            dtcState    = 1;
            dtcLastTick = now;
            dtcInterval = 200;
          }
          /* SIL just lit and held >=1 s: jump-start a cycle now so the log
           * captures what the modules hold at that exact moment (#81). */
          else if (silHeld && (now - lastSilCycle) >= 15000u)
          {
            lastSilCycle = now;
            PrintF("[%lu] SIL held >1s -> immediate DTC cycle\r\n",
                   (unsigned long)now);
            dtcState    = 1;
            dtcLastTick = now;
            dtcInterval = 0;
          }
          else if ((now - dtcLastTick) >= dtcInterval)
          {
            dtcState    = 1;
            dtcLastTick = now;
            dtcInterval = 200;
          }
        }
      }
#endif

#if BLINKER_ENABLED
#if SECURITY_PIN_ENABLED
      if (securityBusy())
      {
        /* PIN entry / settings menu owns the buttons and lamps; the hazard
         * chord still works and blinkerDoBlink below keeps it flashing. */
        securityHandler();
      }
      else
      {
        blinkerHandler();
      }
#else
      /* Immobilizer disabled: the buttons always drive the blinker. */
      blinkerHandler();
#endif

      if (hazardEnabled || leftEnabled || rightEnabled)
      {
        #if MEMS_ENABLED
        if (!hazardEnabled && !postTurnTailActive && !trackingEnabled)
        {
          trackingEnabled = true;
          initialTime = HAL_GetTick();
          initialYaw = INT16_MIN;
          DEBUG_LOG("Tracking started at %lu\r\n", initialTime);
        }
        #endif
        blinkerDoBlink();
      }

      blinkerAutoCancelHandler();
#endif

#if MEMS_ENABLED
      if (!mpu->ok())
        continue;

      mpu->sampleQuant();

      if (HAL_GetTick() < IMU_STARTUP_TIME)
        continue;

#if DEBUG
      if (HAL_GetTick() - prevSample > 1 * 1000)
      {
        auto ypr = mpu->getYawPitchRollD();
        DEBUG_LOG("Y=%.3d\r\n", ypr.x);
        DEBUG_LOG("P=%.3d\r\n", ypr.y);
        DEBUG_LOG("R=%.3d\r\n", ypr.z);
        prevSample = HAL_GetTick();
      }
#endif

      if (hazardEnabled || (!leftEnabled && !rightEnabled))
      {
        trackingEnabled = false;
        continue;
      }

      /* Yaw tracking runs only while armed. Once the turn is detected (or
       * the tracker timed out) trackingEnabled is cleared and the post-turn
       * tail owns the shutdown; without this gate the block below would
       * re-seed initialYaw during the tail and re-trigger the tail on every
       * further 60 degrees of a U-turn (blanking the lamp mid-cycle), and
       * the timeout branch would fire against a stale initialTime and kill
       * the tail early. Re-armed by the tracking block in the blinker
       * section on the next signal activation. */
      if (!trackingEnabled)
        continue;

      auto ypr = mpu->getYawPitchRollD();

      if (initialYaw == INT16_MIN)
      {
        initialYaw = ypr.x;
        DEBUG_LOG("Initial yaw = %.3d\r\n", initialYaw);
        continue;
      }

      const bool turnDetected = detectTurn(initialYaw, ypr.x, TURN_ANGLE_THRESHOLD);
      const bool turnTimedOut = HAL_GetTick() - initialTime >= TURN_MAX_TIME_MS;
      if (!turnDetected && !turnTimedOut)
        continue;

      if (turnDetected)
      {
        DEBUG_LOG("Turn detected, arming post-turn tail: yaw = %.3d\r\n", ypr.x);
        startPostTurnTail();
      }
      else
      {
        DEBUG_LOG("Deactivating the blinker: turn tracking timeout\r\n");
        overtakeMode = false;
        postTurnTailActive = false;
        blinkCounter = 0;
        leftSideOff();
        rightSideOff();
      }

      trackingEnabled = false;
      initialYaw = INT16_MIN;

#endif
    }
    DEBUG_LOG("Stop!\r\n");
  }

#ifdef __cplusplus
}
#endif
