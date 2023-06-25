#pragma once
#include "i2c.h"

namespace MEMS
{  
  struct Magnetometer
  {
    virtual ~Magnetometer() = default;
    virtual void configure() = 0;
    virtual void calibrate() = 0;
    virtual Axes3D readData() = 0;
    virtual void readReg(uint8_t reg, uint8_t byte) = 0;
    virtual void readReg(uint8_t reg, uint8_t *buf, size_t len, uint32_t timeout_ms) = 0;
    virtual void writeReg(uint8_t reg, uint8_t byte) = 0;
    virtual void writeReg(uint8_t reg, uint8_t *buf, size_t len) = 0;
  };

  struct Mpu9250MagI2C : Magnetometer
  {
    Mpu9250MagI2C(I2C_HandleTypeDef *i2c){};
    Axes3D readData(){};
    void readReg(uint8_t reg, uint8_t byte){};
    void readReg(uint8_t reg, uint8_t *buf, size_t len, uint32_t timeout_ms){};
    void writeReg(uint8_t reg, uint8_t byte){};
    void writeReg(uint8_t reg, uint8_t *buf, size_t len){};
  };
}