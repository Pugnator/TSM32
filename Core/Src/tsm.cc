#include "tsm.h"
#include "settings.h"
#include "j1850.h"

#if MEMS_ENABLED
#include "ahrs.h"
#include "engine_state.h"
#endif

#include <stdio.h>
#include "id.h"
#include "vmmu.h"
#include "assert.h"
#include "dwtdelay.h"
#include "watchdog.h"

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
    uint32_t id[3] = {0};
    getCPUid(id, STM32F1_t);
    BANNER("Device ID %.8lx%.8lx%.8lx\r\nTSM %s %s (%s) started\r\n",
           id[0], id[1], id[2],
           VERSION_BUILD_DATE, VERSION_TAG, VERSION_BUILD);

    /* Report why the previous boot ended (IWDG hang / fault / power-on) then
     * arm the IWDG BEFORE the slow init below, so a hang in init is caught as
     * well as one in the main loop. Every init phase and the loop refresh
     * within the ~2 s timeout. */
    watchdog_report_reset_cause();
    watchdog_init();

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

    /*Enable starter first*/
    enableStarter();

    leftSideOff();
    rightSideOff();

// uint8_t frame[2] = {0xAA, 0xAA};
// J1850VPW::sendFrame(frame, 2);
#if DEBUG
    uint32_t prevSample = HAL_GetTick();
#endif

#if MEMS_ENABLED
    std::unique_ptr<Ahrs::AhrsBase<Imu::Bus>> mpu(
        new Ahrs::AhrsBase<Imu::Bus>(IMU_BUS_HANDLE, true));
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

      // Periodic IPC DTC clear — DISABLED, but the original justification was
      // WRONG (#81): harley_new.log, the session used to "prove" the heartbeat
      // alone keeps the SIL off, was captured on build g18a4725 and contains
      // "Periodic SIL clear -> IPC" every 10 s from t=40 s — both mechanisms
      // were active.  The SIL regression appeared once this block was removed.
      // Its role is now covered by the event-driven IPC clear in the auto-DTC
      // state machine below (react state), which fires only when the IPC
      // reports codes or the SIL is actually lit.
      //
      // {
      //   static uint32_t silClearLastTick = 0;
      //   static bool     silClearArmed    = false;
      //   static uint32_t busActiveSince   = 0;
      //   const uint32_t  now_sc           = HAL_GetTick();
      //   if (frameCounter >= 5 && busActiveSince == 0)
      //     busActiveSince = now_sc;
      //   if (!silClearArmed && busActiveSince != 0 &&
      //       (now_sc - busActiveSince) >= 30000u)
      //   {
      //     silClearArmed    = true;
      //     silClearLastTick = now_sc;
      //   }
      //   if (silClearArmed && (now_sc - silClearLastTick) >= 10000u)
      //   {
      //     const uint8_t clrIpc[4] = {0x6C, 0x61, 0xF1, 0x14};
      //     PrintF("[%lu] Periodic SIL clear -> IPC\r\n", (unsigned long)now_sc);
      //     j1850TxRaw(clrIpc, sizeof(clrIpc));
      //     silClearLastTick = HAL_GetTick();
      //   }
      // }

      // Auto-DTC poll: once the bus is active (>=5 frames), query ECM(0x10)
      // and both IPC address candidates (0x60, 0x61 - see #72, the log will
      // show which one answers) with 200 ms gaps, then react to what THIS
      // cycle's responses reported, then cool down 10 s and repeat.
      //
      // The clear is no longer a one-shot: the response flags are reset at
      // the start of every cycle, so a password DTC that the ECM re-sets is
      // cleared again on the next pass, rate-limited by the 10 s cooldown
      // (Fixes #71).  0x40 is no longer queried - that is our own address
      // (Fixes #73).
      //
      // States: 0=wait for bus  1-3=querying modules
      //         4=react (clear what was reported)  5=cooldown
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
            // Clear ECM DTCs (P1009/P1010 password fault drives MIL).
            const uint8_t clrEcm[4] = {0x6C, 0x10, 0xF1, 0x14};
            PrintF("[%lu] DTC clear -> ECM (password DTC present)\r\n",
                   (unsigned long)HAL_GetTick());
            j1850TxRaw(clrEcm, sizeof(clrEcm));
          }
          /* IPC clear: either it reported stored codes, or the SIL is lit
           * with nothing reported - clearing in the latter case is the #81
           * discriminator: lamp goes off => it was IPC-stored (U1064/U1255),
           * lamp stays => it is ECM/security-status driven. */
          if (ipcDtcSeen || silHeld)
          {
            const uint8_t clrIpc[4] = {0x6C, 0x61, 0xF1, 0x14};
            PrintF("[%lu] DTC clear -> IPC (%s)\r\n",
                   (unsigned long)HAL_GetTick(),
                   ipcDtcSeen ? "codes stored" : "SIL lit, no codes reported");
            j1850TxRaw(clrIpc, sizeof(clrIpc));
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
      blinkerHandler();

      if (hazardEnabled || leftEnabled || rightEnabled)
      {
        #if MEMS_ENABLED
        if (!hazardEnabled && !trackingEnabled)
        {
          trackingEnabled = true;
          initialTime = HAL_GetTick();
          initialYaw = INT16_MIN;
          DEBUG_LOG("Tracking started at %lu\r\n", initialTime);
        }
        #endif
        blinkerDoBlink();
      }

      if (overtakeMode && OVERTAKE_BLINK_COUNT < blinkCounter)
      {
        DEBUG_LOG("Deactivating the blinker: blink counter\r\n");
        /* Go through the side-off helpers so the sidemark brightness is
         * restored and the blink FSM is reset, like every other off-path
         * (Fixes #60). */
        overtakeMode = false;
        hazardEnabled = false;
        leftSideOff();
        rightSideOff();
        blinkCounter = 0;
      }
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

      auto ypr = mpu->getYawPitchRollD();

      if (initialYaw == INT16_MIN)
      {
        initialYaw = ypr.x;
        DEBUG_LOG("Initial yaw = %.3d\r\n", initialYaw);
        continue;
      }

      if (!detectTurn(initialYaw, ypr.x, TURN_ANGLE_THRESHOLD) &&
          HAL_GetTick() - initialTime < TURN_MAX_TIME_MS)
        continue;

      DEBUG_LOG("Deactivating the blinker: turn detected, yaw = %.3d\r\n", ypr.x);

      if (leftEnabled)
      {
        leftSideToggle();
      }
      else if (rightEnabled)
      {
        rightSideToggle();
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