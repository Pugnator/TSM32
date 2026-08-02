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
    PrintF("Device ID %.8lx%.8lx%.8lx\r\nTSM %s %s (%s) started\r\n",
           id[0], id[1], id[2],
           VERSION_BUILD_DATE, VERSION_TAG, VERSION_BUILD);

    startupSettingsHandler();

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
    stopAppExecuting = false;
    while (!stopAppExecuting)
    {
#if AUTO_LIGHT_ENABLE
      adcHandler();
#endif

#if J1850_ENABLED
      if (messageCollected)
      {
        J1850VPW::printFrame();
        J1850VPW::parseFrame();
        J1850VPW::messageReset();
      }

      Engine::handler();

      // TSM network-presence heartbeat: send "29 FE 40 01 <crc>" every 2 s
      // once the bus is active.  Without this the IPC logs U1255 "Serial Data
      // Error/Missing Message" because it expects to hear from address 0x40
      // (TSM) at least once every ~3 s, and lights the SIL.
      {
        static uint32_t hbLastTick = 0;
        static bool     hbStarted  = false;
        const uint32_t  now_hb     = HAL_GetTick();
        if (!hbStarted && frameCounter >= 5)
        {
          hbStarted  = true;
          hbLastTick = now_hb;
        }
        if (hbStarted && (now_hb - hbLastTick) >= 2000u)
        {
          static const uint8_t hb[] = {0x29, 0xFE, 0x40, 0x01};
          j1850TxRaw(hb, sizeof(hb));
          hbLastTick = HAL_GetTick();
        }
      }

      // Periodic IPC DTC clear — DISABLED: confirmed unnecessary after bike testing.
      //
      // The IPC sets U1255 ("Serial Data Error / Missing Message") only when
      // address 0x40 (TSM) stops sending its network-presence heartbeat
      // (29 FE 40 xx) for more than ~3 s.  The heartbeat below fires every 2 s
      // and fully prevents U1255 from ever being stored, so this fallback clear
      // serves no purpose and needlessly writes to IPC flash every 10 s.
      //
      // Validation: harley_new.log shows SIL=ON exactly once for 63 ms at boot
      // (ECM lamp self-test, not a fault), then SIL=off for the entire 140 s
      // session — zero occurrences of the ~600 ms periodic SIL=ON bursts seen
      // in j1850_live_capture.log (captured without the heartbeat).
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

      // Auto-DTC poll: once the bus is active (>=5 frames), query ECM, BCM
      // and IPC in sequence with 200 ms gaps, then on the first cycle send a
      // one-shot clear to ECM (clears P1010 / security lamp), then repeat
      // the read-only query every 10 s.
      //
      // States: 0=wait for bus  1-3=querying modules
      //         4=one-time clear DTC (first cycle only)  5=cooldown
      {
        /* Wrap-safe deadlines: store the start tick and an interval, then
         * compare via unsigned subtraction. Raw HAL_GetTick() >= deadline
         * comparisons stalled or fired continuously across the 49.7-day
         * tick wrap (Fixes #49). */
        static uint8_t  dtcState     = 0;
        static uint32_t dtcLastTick  = 0;
        static uint32_t dtcInterval  = 0;
        static bool     clearedOnce  = false;

        // Modules to query: ECM(0x10), BCM/TSM(0x40), IPC(0x60)
        static const uint8_t dtcTargets[] = {0x10, 0x40, 0x60};

        const uint32_t now = HAL_GetTick();

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
            const uint8_t target = dtcTargets[dtcState - 1];
            const uint8_t req[7] = {0x6C, target, 0xF1, 0x19, 0x52, 0xFF, 0x00};
            TRACE_LOG("Auto DTC query -> 0x%02X\r\n", (unsigned)target);
            j1850TxRaw(req, sizeof(req));
            dtcLastTick = HAL_GetTick();
            dtcInterval = 200;
            dtcState    = (dtcState < 3) ? dtcState + 1 : 4;
          }
        }
        else if (dtcState == 4 && (now - dtcLastTick) >= dtcInterval)
        {
          if (!clearedOnce && passwordDtcSeen)
          {
            // Clear ECM DTCs (P1009/P1010 password fault drives MIL).
            const uint8_t clrEcm[4] = {0x6C, 0x10, 0xF1, 0x14};
            PrintF("[%lu] Auto DTC clear -> ECM (password DTC present)\r\n",
                   (unsigned long)HAL_GetTick());
            j1850TxRaw(clrEcm, sizeof(clrEcm));
            // Clear BCM DTCs if any were returned (BCM can hold U1064 etc.
            // which independently keep the SIL on).
            if (bcmDtcSeen)
            {
              const uint8_t clrBcm[4] = {0x6C, 0x40, 0xF1, 0x14};
              PrintF("[%lu] Auto DTC clear -> BCM\r\n",
                     (unsigned long)HAL_GetTick());
              j1850TxRaw(clrBcm, sizeof(clrBcm));
            }
            // Clear IPC DTCs if any were returned (IPC can hold U1064
            // "Loss of TSM/TSSM Serial Data" which drives SIL independently
            // of ECM).
            if (ipcDtcSeen)
            {
              const uint8_t clrIpc[4] = {0x6C, 0x61, 0xF1, 0x14};
              PrintF("[%lu] Auto DTC clear -> IPC\r\n",
                     (unsigned long)HAL_GetTick());
              j1850TxRaw(clrIpc, sizeof(clrIpc));
            }
            clearedOnce = true;
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
        overtakeMode = false;
        leftEnabled = false;
        rightEnabled = false;
        hazardEnabled = false;
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