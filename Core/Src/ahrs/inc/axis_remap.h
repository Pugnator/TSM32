#pragma once

#include "math3d.h"

namespace Ahrs
{
  /* The MPU is mounted 180 degrees about body Z: sensor X points backward,
   * sensor Y points left, and sensor Z points up.  This proper rotation has
   * determinant +1 and is valid for both vectors and angular velocity. */
  inline VectorFloat sensorToBodyZ180(float x, float y, float z, float scale)
  {
    return VectorFloat(-x * scale, -y * scale, z * scale);
  }
}
