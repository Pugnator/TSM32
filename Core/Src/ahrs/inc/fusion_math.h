#pragma once

#include "math3d.h"
#include <cmath>

namespace Ahrs
{
namespace detail
{
  inline bool normalizeGradient(float &s0, float &s1, float &s2, float &s3)
  {
    const float normSq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
    if (!(normSq > 1e-12f) || !std::isfinite(normSq))
      return false;

    const float recipNorm = FAST_INV_SQRT(normSq);
    if (!std::isfinite(recipNorm))
      return false;

    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    return true;
  }
}
}
