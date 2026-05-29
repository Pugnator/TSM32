#include "tsm.h"
#include "settings.h"
#include "j1850.h"
#include "spi.h"
#include "cli.h"

#if MEMS_ENABLED
#include "spi.h"
#include "imu_spi.h"
#include "imu_i2c.h"
#include "ahrs.h"
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

// Raw, non-owning pointer used by the CLI accessors below to read live
// AHRS state without having to be templated on MpuType.  Owned by the
// unique_ptr in tsmRunApp(); cleared at scope exit.
static Ahrs::AhrsBase<Mpu9250::Mpu9250Spi> *gAhrs_ = nullptr;

extern "C" float cliGetChipTemperatureC(void)
{
  return gAhrs_ ? gAhrs_->getTemperature() : 0.0f;
}

extern "C" void cliGetYprDeg(int16_t *yaw, int16_t *pitch, int16_t *roll)
{
  if (!gAhrs_)
  {
    if (yaw)   *yaw = 0;
    if (pitch) *pitch = 0;
    if (roll)  *roll = 0;
    return;
  }
  auto v = gAhrs_->getYawPitchRollD();
  if (yaw)   *yaw = v.x;
  if (pitch) *pitch = v.y;
  if (roll)  *roll = v.z;
}

extern "C" bool cliGetImuOk(void)
{
  return gAhrs_ && gAhrs_->ok();
}
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
    std::unique_ptr<Ahrs::AhrsBase<Mpu9250::Mpu9250Spi>> mpu(new Ahrs::AhrsBase<Mpu9250::Mpu9250Spi>(&hspi1, true));
    //std::unique_ptr<Ahrs::AhrsBase<Mpu9250::Mpu9250I2c>> mpu(new Ahrs::AhrsBase<Mpu9250::Mpu9250I2c>(&hi2c1, true));
    gAhrs_ = mpu.get();
    PrintF("MEMS: MPU9250 SPI init %s\r\n", gAhrs_->ok() ? "OK" : "FAILED - check SPI/CS wiring");
#endif
    stopAppExecuting = false;
    while (!stopAppExecuting)
    {
      // cliPoll();  /* disabled during J1850 RX bench testing */

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

      // Auto-DTC poll: once the bus is active (>=5 frames), query ECM, BCM
      // and IPC in sequence with 200 ms gaps, then on the first cycle send a
      // one-shot clear to ECM (clears P1010 / security lamp), then repeat
      // the read-only query every 10 s.
      //
      // States: 0=wait for bus  1-3=querying modules
      //         4=one-time clear DTC (first cycle only)  5=cooldown
      {
        static uint8_t  dtcState     = 0;
        static uint32_t dtcNextTick  = 0;
        static bool     clearedOnce  = false;

        // Modules to query: ECM(0x10), BCM/TSM(0x40), IPC(0x60)
        static const uint8_t dtcTargets[] = {0x10, 0x40, 0x60};

        if (dtcState == 0 && frameCounter >= 5)
        {
          dtcState    = 1;
          dtcNextTick = HAL_GetTick() + 200;
        }
        else if (dtcState >= 1 && dtcState <= 3)
        {
          if (HAL_GetTick() >= dtcNextTick)
          {
            const uint8_t target = dtcTargets[dtcState - 1];
            const uint8_t req[7] = {0x6C, target, 0xF1, 0x19, 0x52, 0xFF, 0x00};
            TRACE_LOG("Auto DTC query -> 0x%02X\r\n", (unsigned)target);
            cliJ1850TxRaw(req, sizeof(req));
            dtcNextTick = HAL_GetTick() + 200;
            dtcState    = (dtcState < 3) ? dtcState + 1 : 4;
          }
        }
        else if (dtcState == 4 && HAL_GetTick() >= dtcNextTick)
        {
          if (!clearedOnce)
          {
            // One-time clear of all ECM DTCs after the first read cycle.
            // This dismisses P1010 (Missing Password) so the security lamp
            // goes off.  P1010 is historic — it won't recur until the ECM
            // actively re-challenges and gets no reply.
            const uint8_t clr[4] = {0x6C, 0x10, 0xF1, 0x14};
            PrintF("[%lu] Auto DTC clear -> ECM (MIL should go off)\r\n",
                   (unsigned long)HAL_GetTick());
            cliJ1850TxRaw(clr, sizeof(clr));
            clearedOnce = true;
          }
          dtcState    = 5;
          dtcNextTick = HAL_GetTick() + 10000;
        }
        else if (dtcState == 5 && HAL_GetTick() >= dtcNextTick)
        {
          dtcState    = 1;
          dtcNextTick = HAL_GetTick() + 200;
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
#if MEMS_ENABLED
    gAhrs_ = nullptr;
#endif
  }

#ifdef __cplusplus
}
#endif