#pragma once
#include <memory>

#include "acc.h"
#include "gyro.h"
#include "mag.h"
#include "types.h"

namespace MEMS
{

  struct Mems
  {
    Mems()
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
      Axes3D read();

    private:
      std::unique_ptr<Accelerometer> instance_;
    };
    struct Gyro
    {
      Gyro()
      {
      }
      Axes3D read();

    private:
      std::unique_ptr<Gyroscope> instance_;
    };
    struct Mag
    {
      Mag()
      {
      
      }
      Axes3D read();

    private:
      std::unique_ptr<Magnetometer> instance_;
    };

  private:
    std::unique_ptr<Mag> mag_;
    std::unique_ptr<Acc> acc_;
    std::unique_ptr<Gyro> gyro_;
  };
}