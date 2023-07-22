#pragma once
#include "imu_spi.h"
#include "imu_base.h"
#include "antijam.h"

#include "math3d.h"
#include <utility>
#include <memory>

#define DEG2RAD(x) (x * (M_PI / 180.f))
#define RAD2DEG(x) (x * (180.f / M_PI))

class Eeprom;

namespace Ahrs
{

  template <typename MpuType>
  class AhrsBase : public MpuType
  {
  public:
    template <typename... Args>
    AhrsBase(Args &&...args) : MpuType(std::forward<Args>(args)...)
    {
      antiJam.reset(new MagneticJammingDetector);
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
    }

    bool staticCalibration(Eeprom *mem = nullptr);
    bool loadCalibration(Eeprom *mem);

    void madgwick6DoF(Quaternion &q, VectorFloat &g, VectorFloat &a);
    void madgwick9DoF(Quaternion &q, VectorFloat &g, VectorFloat &a, VectorFloat &m);
    void mahony9DoF(Quaternion &q, VectorFloat &g, VectorFloat &a, VectorFloat &m);

    float getHeadingAngle();
    VectorInt16 getYawPitchRollD();
    VectorFloat getYawPitchRollR();
    void getEuler(float *output, Quaternion &q);

    void getGravity(VectorFloat &vectorOut, Quaternion &q);
    void getLinearAccel(VectorInt16 &vectorOut, VectorInt16 &vectorRaw, VectorFloat &gravity);

    void magAutoOffset(VectorFloat &axes);
    void accAutoOffset(VectorFloat &axes);
    void gyroAutoOffset(VectorFloat &axes);
    const Quaternion &sampleQuant();

  private:
    std::unique_ptr<MagneticJammingDetector> antiJam;

    VectorFloat getYawPitchRoll();

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
  };
}