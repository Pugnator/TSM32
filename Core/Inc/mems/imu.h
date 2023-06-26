#pragma once
#include <memory>

#include "acc.h"
#include "gyro.h"
#include "mag.h"
#include "types.h"
#include "math3d.h"

namespace AHRS
{

  struct InertialMeasurementUnit
  {
    InertialMeasurementUnit()
    {
      acc_.reset(new Acc);
      gyro_.reset(new Gyro);
      mag_.reset(new Mag);
    }

    void configure();

  private:
    struct Acc
    {
      Acc()
      {
      }
      VectorInt16 read();

    private:
      std::unique_ptr<Accelerometer> instance_;
    };
    struct Gyro
    {
      Gyro()
      {
      }
      VectorInt16 read();

    private:
      std::unique_ptr<Gyroscope> instance_;
    };
    struct Mag
    {
      Mag()
      {
      
      }
      VectorInt16 read();

    private:
      std::unique_ptr<Magnetometer> instance_;
    };

  private:
    std::unique_ptr<Mag> mag_;
    std::unique_ptr<Acc> acc_;
    std::unique_ptr<Gyro> gyro_;
  };
}