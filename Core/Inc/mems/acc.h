#pragma once
#include "math3d.h"

namespace AHRS
{
  struct Accelerometer
  {
    virtual ~Accelerometer() = default;
    virtual void configure() = 0;
    virtual VectorInt16 readData() = 0;
    virtual void readReg(uint8_t reg, uint8_t *buf, size_t len, uint32_t timeout_ms) = 0;
    virtual void writeReg(uint8_t reg, uint8_t byte) = 0;
    virtual void writeReg(uint8_t reg, uint8_t *buf, size_t len) = 0;
    virtual void calibrate() = 0;
  };
}