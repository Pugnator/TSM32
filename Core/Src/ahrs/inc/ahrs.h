#pragma once
#include "settings.h"

#if !defined(MEMS_ENABLED) || MEMS_ENABLED
/* Pull in only the chosen MPU9250 transport. The unselected .cc file is
 * dropped from the Makefile build set, so its symbols and HAL deps are
 * not linked at all. The bus header (spi.h / i2c.h) is included here so
 * the peripheral handle (hspi1 / hi2c1) is visible to IMU_BUS_HANDLE
 * users; imu_spi.h / imu_i2c.h provide weak HAL stubs for the case
 * where the peripheral header is absent. */
#if IMU_USE_SPI
#if __has_include("spi.h")
#include "spi.h"
#endif
#include "imu_spi.h"
#elif IMU_USE_I2C
#if __has_include("i2c.h")
#include "i2c.h"
#endif
#include "imu_i2c.h"
#endif
#endif

#include "imu_base.h"

#include "math3d.h"
#include <utility>
#include <memory>

#define DEG2RAD(x) (x * (M_PI / 180.f))
#define RAD2DEG(x) (x * (180.f / M_PI))

#if MEMS_ENABLED
namespace Imu
{
#if IMU_USE_SPI
  using Bus = Mpu9250::Mpu9250Spi;
  using BusHandleType = SPI_HandleTypeDef;
  static constexpr const char *kBusName = "SPI";
#define IMU_BUS_HANDLE (&hspi1)
#elif IMU_USE_I2C
  using Bus = Mpu9250::Mpu9250I2c;
  using BusHandleType = I2C_HandleTypeDef;
  static constexpr const char *kBusName = "I2C";
#define IMU_BUS_HANDLE (&hi2c1)
#endif
}
#endif

namespace Ahrs
{

  template <typename MpuType>
  class AhrsBase : public MpuType
  {
  public:
    template <typename... Args>
    AhrsBase(Args &&...args) : MpuType(std::forward<Args>(args)...)
    {
      magMaxX_ = 0;
      magMaxY_ = 0;
      magMaxZ_ = 0;
      magMinX_ = 0;
      magMinY_ = 0;
      magMinZ_ = 0;

      accMaxX_ = 0;
      accMaxY_ = 0;
      accMaxZ_ = 0;
      accMinX_ = 0;
      accMinY_ = 0;
      accMinZ_ = 0;

      gyroMaxX_ = 0;
      gyroMaxY_ = 0;
      gyroMaxZ_ = 0;
      gyroMinX_ = 0;
      gyroMinY_ = 0;
      gyroMinZ_ = 0;
      lastTimeUpdated_ = 0;
      sampleFreq_ = static_cast<float>(AHRS_UPDATE_RATE);
      gyroBiasOnline_ = VectorFloat();
      zuptStableCount_ = 0;
      zuptActive_ = false;
    }

    void madgwick6DoF(Quaternion &q, VectorFloat &g, VectorFloat &a);
    void madgwick9DoF(Quaternion &q, VectorFloat &g, VectorFloat &a, VectorFloat &m);

    float getHeadingAngle();
    VectorInt16 getYawPitchRollD();
    VectorFloat getYawPitchRollR();

    VectorFloat &getLastAcceleration() { return acc_; };
    VectorFloat &getLastGyro() { return gyro_; };
    VectorFloat &getLastMagnetometer() { return mag_; };
    void getEuler(float *output, Quaternion &q);

    void getGravity(VectorFloat &vectorOut, Quaternion &q);
    void getLinearAccel(VectorInt16 &vectorOut, VectorInt16 &vectorRaw, VectorFloat &gravity);

    void magAutoOffset(VectorFloat &axes);
    void accAutoOffset(VectorFloat &axes);
    void gyroAutoOffset(VectorFloat &axes);
    const Quaternion &sampleQuant();

  private:
    VectorFloat getYawPitchRoll();
#if ENABLE_ZUPT
    void updateGyroBiasIfStill();
#endif

    float accOffsetX_;
    float accOffsetY_;
    float accOffsetZ_;

    float gyroOffsetX_;
    float gyroOffsetY;
    float gyroOffsetZ_;

    float magOffsetX_;
    float magOffsetY_;
    float magOffsetZ_;

    float magMaxX_;
    float magMaxY_;
    float magMaxZ_;
    float magMinX_;
    float magMinY_;
    float magMinZ_;

    float accMaxX_;
    float accMaxY_;
    float accMaxZ_;
    float accMinX_;
    float accMinY_;
    float accMinZ_;

    float gyroMaxX_;
    float gyroMaxY_;
    float gyroMaxZ_;
    float gyroMinX_;
    float gyroMinY_;
    float gyroMinZ_;

    bool isCalibration_;

    Quaternion quan_;
    VectorFloat acc_;
    VectorFloat gyro_;
    VectorFloat mag_;
    uint32_t lastTimeUpdated_;
    float sampleFreq_;

    // ZUPT (zero-velocity update) state: a slow EMA of the gyro reading
    // captured while the bike is stationary, subtracted from every live
    // gyro sample before it reaches the Madgwick filter.
    VectorFloat gyroBiasOnline_;
    uint32_t zuptStableCount_;
    bool zuptActive_;
  };
}