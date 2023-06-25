#pragma once

namespace MEMS
{
  typedef struct Axes3D
  {
    float x;
    float y;
    float z;

  } Axes3D;

  enum class DeviceType
  {
    BMI270,
    MPU9250
  };

  enum class Ascale : int
  {
    Scale2G = 0,
    Scale4G = 1,
    Scale8G = 2,
    Scale16G = 3
  };

  enum class Gscale : int
  {
    Scale250DPS = 0,
    Scale500DPS = 1,
    Scale1000DPS = 2,
    Scale2000DPS = 3
  };

  enum class Mscale
  {
    MFS_14BITS = 0, // 0.6 mG per LSB
    MFS_16BITS      // 0.15 mG per LSB
  };
}