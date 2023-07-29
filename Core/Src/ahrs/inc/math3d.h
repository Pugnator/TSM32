#pragma once
#include <cstdint>
#include <cstdlib>
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
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;             // Type punning to reinterpret the bits of 'y' as a long integer
  i = 0x5f3759df - (i >> 1);        // Magic number calculation for initial approximation
  y = *(float *)&i;                 // Type punning to reinterpret the bits of 'i' as a float
  y = y * (1.5f - (halfx * y * y)); // Refining the approximation
  return y;
}

// Log base 2 approximation and Newton's Method
static inline float _fastSqrt(float z)
{
  union
  {
    float f;
    uint32_t i;
  } val = {z}; /* Convert type, preserving bit pattern */
  /*
   * To justify the following code, prove that
   *
   * ((((val.i / 2^m) - b) / 2) + b) * 2^m = ((val.i - 2^m) / 2) + ((b + 1) / 2) * 2^m)
   *
   * where
   *
   * b = exponent bias
   * m = number of mantissa bits
   */
  val.i -= 1 << 23; /* Subtract 2^m. */
  val.i >>= 1;      /* Divide by 2. */
  val.i += 1 << 29; /* Add ((b + 1) / 2) * 2^m. */

  return val.f; /* Interpret again as float */
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