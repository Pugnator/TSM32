#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <bit>
#include <limits>
#include <cmath>
#include "ahrs_config.h"

#if SPEED_MATH
#define FAST_ASIN(x) _fastAsin(x)
#define FAST_ATAN2(x, y) _fastAtan2(x, y)
#define FAST_SQRT(x) _fastSqrt(x)
#define FAST_INV_SQRT(x) _fastInvSqrt(x)
#else
#define FAST_ASIN(x) asin(x)
#define FAST_ATAN2(x, y) atan2(x, y)
#define FAST_SQRT(x) sqrt(x)
#define FAST_INV_SQRT(x) (1.0 / sqrt(x))
#endif

static inline float _fastAsin(float x)
{
  // The polynomial approximation is only valid on [-1, +1]; outside that
  // range it diverges rapidly.  Floating-point rounding in upstream gravity-
  // vector normalisation routinely produces inputs like 1.0000001f, so clamp
  // explicitly before evaluating the polynomial.
  if (x >= 1.0f)
    return static_cast<float>(M_PI_2);
  if (x <= -1.0f)
    return -static_cast<float>(M_PI_2);

  const float c1 = 1.5707288f;  // Polynomial coefficient 1
  const float c2 = -0.2121144f; // Polynomial coefficient 2
  const float c3 = 0.0742610f;  // Polynomial coefficient 3
  const float c4 = -0.0187293f; // Polynomial coefficient 4

  float y = x * (fabsf(x) * (fabsf(x) * (c4 * fabsf(x) + c3) + c2) + c1);

  return y;
}

static inline float _fastAtan2(float y, float x)
{
  const float ONEQTR_PI = M_PI / 4.0f;
  const float THRQTR_PI = 3.0f * M_PI / 4.0f;
  float r, angle;
  float absY = fabsf(y) + 1e-10f; // Small offset to avoid division by zero

  if (x < 0.0f)
  {
    r = (x + absY) / (absY - x);
    angle = THRQTR_PI;
  }
  else
  {
    r = (x - absY) / (x + absY);
    angle = ONEQTR_PI;
  }

  angle += (0.1963f * r * r - 0.9817f) * r;

  if (y < 0.0f)
    return -angle; // negate if in quad III or IV
  else
    return angle;
}

static inline float _fastInvSqrt(float x)
{
  // Quake III fast inverse square root.  The bit-level reinterpretation must
  // go through memcpy() to stay defined under strict aliasing -- the previous
  // long* cast was UB and could be reordered or constant-folded incorrectly
  // by -O3 / -flto even with -fno-strict-aliasing on this translation unit.
  const float halfx = 0.5f * x;
  std::uint32_t i;
  std::memcpy(&i, &x, sizeof(i));
  i = 0x5f3759dfu - (i >> 1);
  float y;
  std::memcpy(&y, &i, sizeof(y));
  y = y * (1.5f - (halfx * y * y)); // one Newton iteration
  return y;
}

// Log base 2 approximation followed by one Newton-Raphson refinement.
static inline float _fastSqrt(float z)
{
  if (z <= 0.0f)
    return 0.0f;

  std::uint32_t i;
  std::memcpy(&i, &z, sizeof(i));
  i -= 1u << 23; // Subtract 2^m.
  i >>= 1;       // Divide by 2.
  i += 1u << 29; // Add ((b + 1) / 2) * 2^m.

  float y;
  std::memcpy(&y, &i, sizeof(y));

  // One Newton-Raphson iteration brings the ~5% bit-hack approximation
  // down to better than 0.1% relative error.  y_{n+1} = 0.5 * (y + z/y).
  y = 0.5f * (y + z / y);
  return y;
}

struct Quaternion
{
  float w;
  float x;
  float y;
  float z;

  Quaternion()
  {
    w = 1.0f;
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
  }

  Quaternion(float nw, float nx, float ny, float nz)
  {
    w = nw;
    x = nx;
    y = ny;
    z = nz;
  }

  void zeroRollAndPitch()
  {
    x = 0.0f;
    y = 0.0f;
  }

  bool isNormalized()
  {
    float nE = abs(w * w + x * x + y * y + z * z - 1.f);
    if (nE > 0.01f)
      return false;

    return true;
  }

  Quaternion getProduct(Quaternion q)
  {
    return Quaternion(
        w * q.w - x * q.x - y * q.y - z * q.z,  // new w
        w * q.x + x * q.w + y * q.z - z * q.y,  // new x
        w * q.y - x * q.z + y * q.w + z * q.x,  // new y
        w * q.z + x * q.y - y * q.x + z * q.w); // new z
  }

  Quaternion getConjugate()
  {
    return Quaternion(w, -x, -y, -z);
  }

  float getMagnitude()
  {
    return FAST_SQRT(w * w + x * x + y * y + z * z);
  }

  bool normalize()
  {
    float m = getMagnitude();
    if (m == 0.)
      return false;
    w /= m;
    x /= m;
    y /= m;
    z /= m;
    return true;
  }

  Quaternion getNormalized()
  {
    Quaternion r(w, x, y, z);
    if (r.normalize())
      return Quaternion();

    return r;
  }
};

struct VectorInt16
{
  int16_t x;
  int16_t y;
  int16_t z;

  VectorInt16()
  {
    x = 0;
    y = 0;
    z = 0;
  }

  VectorInt16(int16_t nx, int16_t ny, int16_t nz)
  {
    x = nx;
    y = ny;
    z = nz;
  }

  // Euclidean magnitude formula
  float getMagnitude()
  {
    return FAST_SQRT(x * x + y * y + z * z);
  }

  bool normalize()
  {
    float m = getMagnitude();
    if (!m)
      return false;

    x *= m;
    y *= m;
    z *= m;
    return true;
  }

  bool isNormalized()
  {
    float nE = abs(x * x + y * y + z * z - 1.f);
    if (nE > 0.01f)
      return false;

    return true;
  }

  VectorInt16 getNormalized()
  {
    VectorInt16 r(x, y, z);
    if (!r.normalize())
      return VectorInt16();
    return r;
  }

  void rotate(Quaternion *q)
  {
    // http://www.cprogramming.com/tutorial/3d/quaternions.html
    // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
    // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
    // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

    // P_out = q * P_in * conj(q)
    // - P_out is the output vector
    // - q is the orientation quaternion
    // - P_in is the input vector (a*aReal)
    // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
    Quaternion p(0, x, y, z);

    // quaternion multiplication: q * p, stored back in p
    p = q->getProduct(p);

    // quaternion multiplication: p * conj(q), stored back in p
    p = p.getProduct(q->getConjugate());

    // p quaternion is now [0, x', y', z']
    x = p.x;
    y = p.y;
    z = p.z;
  }

  VectorInt16 getRotated(Quaternion *q)
  {
    VectorInt16 r(x, y, z);
    r.rotate(q);
    return r;
  }
};

struct VectorFloat
{
  float x;
  float y;
  float z;

  VectorFloat()
  {
    x = 0;
    y = 0;
    z = 0;
  }

  VectorFloat(float nx, float ny, float nz)
  {
    x = nx;
    y = ny;
    z = nz;
  }

  float getMagnitude()
  {
    return FAST_SQRT(x * x + y * y + z * z);
  }

  void normalize()
  {
    float m = getMagnitude();
    if (m == 0.0f)
      return;
    x /= m;
    y /= m;
    z /= m;
  }

  VectorFloat getNormalized()
  {
    VectorFloat r(x, y, z);
    r.normalize();
    return r;
  }

  bool isNormalized()
  {
    float nE = abs(x * x + y * y + z * z - 1.f);
    if (nE > 0.01f)
      return false;

    return true;
  }

  void rotate(Quaternion *q)
  {
    Quaternion p(0, x, y, z);

    // quaternion multiplication: q * p, stored back in p
    p = q->getProduct(p);

    // quaternion multiplication: p * conj(q), stored back in p
    p = p.getProduct(q->getConjugate());

    // p quaternion is now [0, x', y', z']
    x = p.x;
    y = p.y;
    z = p.z;
  }

  VectorFloat getRotated(Quaternion *q)
  {
    VectorFloat r(x, y, z);
    r.rotate(q);
    return r;
  }
};