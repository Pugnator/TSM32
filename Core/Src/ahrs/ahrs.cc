#include "inc/ahrs.h"
#include "inc/ahrs_config.h"

namespace Ahrs
{
  template <typename MpuType>
  VectorInt16 AhrsBase<MpuType>::getYawPitchRollD()
  {
    auto ypr = getYawPitchRoll();
    VectorInt16 yprD;
    yprD.x = (int16_t)RAD2DEG(ypr.x);
    yprD.y = (int16_t)RAD2DEG(ypr.y);
    yprD.z = (int16_t)RAD2DEG(ypr.z);

    if (yprD.x < -180)
      yprD.x += 360;
    return yprD;
  }

  template <typename MpuType>
  VectorFloat AhrsBase<MpuType>::getYawPitchRollR()
  {
    return getYawPitchRoll();
  }

  template <typename MpuType>
  VectorFloat AhrsBase<MpuType>::getYawPitchRoll()
  {
    VectorFloat ypr_;
    // yaw: (about Z axis)
    ypr_.x = FAST_ATAN2(2.0f * (quan_.x * quan_.y + quan_.w * quan_.z), quan_.w * quan_.w + quan_.x * quan_.x - quan_.y * quan_.y - quan_.z * quan_.z);
    // pitch: (nose up/down, about Y axis)
    ypr_.z = -FAST_ASIN(2.0f * (quan_.x * quan_.z - quan_.w * quan_.y));
    // roll: (tilt left/right, about X axis)
    ypr_.y = FAST_ATAN2(2.0f * (quan_.w * quan_.x + quan_.y * quan_.z), quan_.w * quan_.w - quan_.x * quan_.x - quan_.y * quan_.y + quan_.z * quan_.z);
    return ypr_;
  }

  template <typename MpuType>
  void AhrsBase<MpuType>::getEuler(float *output, Quaternion &q)
  {
    // psi
    output[0] = FAST_ATAN2(2 * q.x * q.y - 2 * q.w * q.z, 2 * q.w * q.w + 2 * q.x * q.x - 1);
    // theta
    output[1] = -FAST_ASIN(2 * q.x * q.z + 2 * q.w * q.y);
    // phi
    output[2] = FAST_ATAN2(2 * q.y * q.z - 2 * q.w * q.x, 2 * q.w * q.w + 2 * q.z * q.z - 1);
  }

  template <typename MpuType>
  void AhrsBase<MpuType>::getGravity(VectorFloat &v, Quaternion &q)
  {
    v.x = 2 * (q.x * q.z - q.w * q.y);
    v.y = 2 * (q.w * q.x + q.y * q.z);
    v.z = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
  }

  template <typename MpuType>
  void AhrsBase<MpuType>::getLinearAccel(VectorInt16 &vectorOut, VectorInt16 &vectorRaw, VectorFloat &gravity)
  {
    // get rid of the gravity component (+1g = +16384 in standard DMP FIFO packet, sensitivity is +-2g)
    vectorOut.x = vectorRaw.x - gravity.x * 16384;
    vectorOut.y = vectorRaw.y - gravity.y * 16384;
    vectorOut.z = vectorRaw.z - gravity.z * 16384;
  }

  template <typename MpuType>
  void AhrsBase<MpuType>::magAutoOffset(VectorFloat &axes)
  {
    // Update magMaxX, magMaxY, magMaxZ with the maximum values
    if (axes.x > magMaxX_)
    {
      magMaxX_ = axes.x;
    }
    if (axes.y > magMaxY_)
    {
      magMaxY_ = axes.y;
    }
    if (axes.z > magMaxZ_)
    {
      magMaxZ_ = axes.z;
    }

    // Update magMinX, magMinY, magMinZ with the minimum values
    if (axes.x < magMinX_)
    {
      magMinX_ = axes.x;
    }
    if (axes.y < magMinY_)
    {
      magMinY_ = axes.y;
    }
    if (axes.z < magMinZ_)
    {
      magMinZ_ = axes.z;
    }

    magOffsetX_ = (magMaxX_ + magMinX_) / 2.0;
    magOffsetY_ = (magMaxY_ + magMinY_) / 2.0;
    magOffsetZ_ = (magMaxZ_ + magMinZ_) / 2.0;
  }

  template <typename MpuType>
  void AhrsBase<MpuType>::accAutoOffset(VectorFloat &axes)
  {
    if (!isCalibration_)
    {
      return;
    }
    // Update accMaxX, accMaxY, accMaxZ with the maximum values
    if (axes.x > accMaxX_)
    {
      accMaxX_ = axes.x;
    }
    if (axes.y > accMaxY_)
    {
      accMaxY_ = axes.y;
    }
    if (axes.z > accMaxZ_)
    {
      accMaxZ_ = axes.z;
    }

    // Update accMinX, accMinY, accMinZ with the minimum values
    if (axes.x < accMinX_)
    {
      accMinX_ = axes.x;
    }
    if (axes.y < accMinY_)
    {
      accMinY_ = axes.y;
    }
    if (axes.z < accMinZ_)
    {
      accMinZ_ = axes.z;
    }

    accOffsetX_ = (accMaxX_ + accMinX_) / 2.0;
    accOffsetY_ = (accMaxY_ + accMinY_) / 2.0;
    accOffsetZ_ = (accMaxZ_ + accMinZ_) / 2.0;
  }

  template <typename MpuType>
  void AhrsBase<MpuType>::gyroAutoOffset(VectorFloat &axes)
  {
    if (!isCalibration_)
      return;

    // Update gyroMaxX, gyroMaxY, gyroMaxZ with the maximum values
    if (axes.x > gyroMaxX_)
    {
      gyroMaxX_ = axes.x;
    }
    if (axes.y > gyroMaxY_)
    {
      gyroMaxY_ = axes.y;
    }
    if (axes.z > gyroMaxZ_)
    {
      gyroMaxZ_ = axes.z;
    }

    // Update gyroMinX, gyroMinY, gyroMinZ with the minimum values
    if (axes.x < gyroMinX_)
    {
      gyroMinX_ = axes.x;
    }
    if (axes.y < gyroMinY_)
    {
      gyroMinY_ = axes.y;
    }
    if (axes.z < gyroMinZ_)
    {
      gyroMinZ_ = axes.z;
    }

    gyroOffsetX_ = (gyroMaxX_ + gyroMinX_) / 2.0;
    gyroOffsetY = (gyroMaxY_ + gyroMinY_) / 2.0;
    gyroOffsetZ_ = (gyroMaxZ_ + gyroMinZ_) / 2.0;
  }

  template <typename MpuType>
  float AhrsBase<MpuType>::getHeadingAngle()
  {
#if DISABLE_MAGNETOMETER
    return std::numeric_limits<float>::quiet_NaN();
#endif
    float magX = mag_.y;
    float magY = mag_.x;
    float magZ = mag_.z;

    VectorFloat pr = getYawPitchRollR();
    float pitch = pr.y;
    float roll = pr.z;

    magX = magX * cos(pitch) + magY * sin(roll) * sin(pitch) - magZ * cos(roll) * sin(pitch);
    magY = magY * cos(roll) + magZ * sin(roll);

    float az = 0.0;
    if (magY > 0)
    {
      az = 90.0f - RAD2DEG(atan(magX / magY));
    }
    else if (magY < 0)
    {
      az = 270.0f - RAD2DEG(atan(magX / magY));
    }
    else if (magY == 0)
    {
      az = magX < 0 ? 180.0f : 0;
    }

    az += MAGNETIC_DECLINATION;
    if (az > 360.f)
    {
      az -= 360.f;
    }

    return az;
  }

  template <typename MpuType>
  const Quaternion &AhrsBase<MpuType>::sampleQuant()
  {
#if DEBUG
    static uint32_t counter = 0;
    static uint32_t beginTime = HAL_GetTick();
#endif
    if (this->useDmp_)
    {
      if (this->interruptStatus() != Mpu9250::InterruptSource::DmpInterrupt)
      {
        return quan_;
      }

      uint16_t fifo = this->dmpFifoRead();
      if (!fifo || 0xFFFF == fifo)
      {
        this->fifoReset();
        return quan_;
      }

      this->getDmpQuaternion(quan_);
    }
    else
    {
      uint32_t sampleTime = HAL_GetTick();
#if FIXED_AHRS_UPDATE_RATE
      static const uint32_t fixedUpdateRateMs = static_cast<uint32_t>(1000.0 / AHRS_UPDATE_RATE);
      if (sampleTime - lastTimeUpdated_ < fixedUpdateRateMs)
        return quan_; // Old value
#endif
      // Bounded poll for the IMU DATA_READY flag.  The previous unconditional
      // busy-wait monopolised the SPI/I2C bus and would freeze the firmware if
      // the IMU INT line ever stuck (mis-wire, ESD event, IMU power-down).
      // At a 100 Hz fixed update rate the IMU always has fresh data by the
      // time we get here, so a few-ms cap is generous; on time-out we return
      // the previous quaternion and try again on the next tick.
      {
        const uint32_t pollDeadline = sampleTime + 3u;
        while (this->interruptStatus() != Mpu9250::InterruptSource::DataReady)
        {
          if (HAL_GetTick() >= pollDeadline)
          {
            DEBUG_LOG("IMU DATA_READY timeout\r\n");
            return quan_;
          }
        }
      }
        // Frequency Update (Hz) = 1 / (Time Interval (ms) * 0.001)

#if FIXED_AHRS_UPDATE_RATE
      sampleFreq_ = AHRS_UPDATE_RATE;
#else
      // Clamp the elapsed time to >= 1 ms so two samples that land in the
      // same SysTick millisecond cannot produce a divide-by-zero in the
      // 1/dt that feeds the Madgwick integrator.
      uint32_t deltaMs = sampleTime - lastTimeUpdated_;
      if (deltaMs < 1u)
        deltaMs = 1u;
      sampleFreq_ = 1.f / (deltaMs * 0.001f);
#endif

      lastTimeUpdated_ = sampleTime;
      this->readAccelAxis(acc_);
      this->readGyroAxis(gyro_);

#if ENABLE_ZUPT
      // Refine the online gyro bias whenever the bike is stationary, then
      // subtract the current bias estimate from the live gyro reading so
      // that downstream Madgwick integration sees a drift-corrected signal.
      updateGyroBiasIfStill();
      gyro_.x -= gyroBiasOnline_.x;
      gyro_.y -= gyroBiasOnline_.y;
      gyro_.z -= gyroBiasOnline_.z;
#endif

      // Refresh chip-die temperature at ~1 Hz (every 100th sample at the
      // 100 Hz fixed update rate).  Used by the ZUPT model and exposed via
      // getTemperature() for thermal-bias compensation.
      {
        static uint32_t tempDivider = 0;
        if (++tempDivider >= 100u)
        {
          tempDivider = 0;
          float t;
          if (this->readChipTemperature(t))
            this->chipTemperature_ = t;
        }
      }
#if !DISABLE_MAGNETOMETER
#if MAGNETOMETER_BLOCKING_MODE
      if (!this->readMagAxis(mag_, true))
#else
      if (!this->readMagAxis(mag_, false))
#endif
      {
        madgwick6DoF(quan_, gyro_, acc_);
      }
      else
      {
        madgwick9DoF(quan_, gyro_, acc_, mag_);
      }
#else
      madgwick6DoF(quan_, gyro_, acc_);
#endif
    }
#if DEBUG
    counter++;
    if (counter == 10 * 100UL)
    {
      uint32_t deltaTime = HAL_GetTick() - beginTime;
      float samplesPerSecond = (float)counter / deltaTime * 1000.f;
      float sampleSpeedHz = 1 / (deltaTime / (float)counter) * 1000.f;
      DEBUG_LOG("%lu measures in %lums = %.4f samples per second = %.4f Hz\r\n", counter, deltaTime, samplesPerSecond, sampleSpeedHz);
    }
#endif
    return quan_;
  }

#if ENABLE_ZUPT
  template <typename MpuType>
  void AhrsBase<MpuType>::updateGyroBiasIfStill()
  {
    // Per-axis stillness check.  Gyro is in deg/s straight from the IMU; we
    // test the raw reading minus the current bias estimate so that a slowly
    // drifting bias on a truly-stationary bike does not lock us out of the
    // stillness window forever.  The accel magnitude must also be within a
    // narrow band around 1 g to reject jolts that happen to cancel across
    // axes.
    const float gxRaw = gyro_.x;
    const float gyRaw = gyro_.y;
    const float gzRaw = gyro_.z;

    const float gResidualX = gxRaw - gyroBiasOnline_.x;
    const float gResidualY = gyRaw - gyroBiasOnline_.y;
    const float gResidualZ = gzRaw - gyroBiasOnline_.z;

    const float aMag = acc_.getMagnitude();

    const bool gyroStill = (fabsf(gResidualX) < ZUPT_GYRO_THRESH_DPS) &&
                           (fabsf(gResidualY) < ZUPT_GYRO_THRESH_DPS) &&
                           (fabsf(gResidualZ) < ZUPT_GYRO_THRESH_DPS);
    const bool accelStill = fabsf(aMag - 1.0f) < ZUPT_ACCEL_THRESH_G;

    if (gyroStill && accelStill)
    {
      if (zuptStableCount_ < ZUPT_HOLD_SAMPLES)
      {
        if (++zuptStableCount_ == ZUPT_HOLD_SAMPLES && !zuptActive_)
        {
          zuptActive_ = true;
          DEBUG_LOG("ZUPT epoch start, bias=(%.3f, %.3f, %.3f) deg/s, T=%.1fC\r\n",
                    gyroBiasOnline_.x, gyroBiasOnline_.y, gyroBiasOnline_.z,
                    this->chipTemperature_);
        }
      }

      if (zuptActive_)
      {
        // EMA toward the raw reading: bias = (1-a)*bias + a*reading.
        const float a = ZUPT_BIAS_ALPHA;
        gyroBiasOnline_.x += a * (gxRaw - gyroBiasOnline_.x);
        gyroBiasOnline_.y += a * (gyRaw - gyroBiasOnline_.y);
        gyroBiasOnline_.z += a * (gzRaw - gyroBiasOnline_.z);
      }
    }
    else
    {
      if (zuptActive_)
      {
        DEBUG_LOG("ZUPT epoch end, bias=(%.3f, %.3f, %.3f) deg/s\r\n",
                  gyroBiasOnline_.x, gyroBiasOnline_.y, gyroBiasOnline_.z);
      }
      zuptStableCount_ = 0;
      zuptActive_ = false;
    }
  }
#endif

  template class AhrsBase<Mpu9250::Mpu9250Spi>;
  template class AhrsBase<Mpu9250::Mpu9250I2c>;
}