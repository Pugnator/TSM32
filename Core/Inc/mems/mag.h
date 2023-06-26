#pragma once
#include "i2c.h"
#include "spi.h"

namespace AHRS
{


  enum class MagnetometerError
  {
    Ok,
    ReadError,
    WriteError,
    NotDetected,
    NotImplemented,
  };
  struct Magnetometer
  {
    virtual ~Magnetometer() = default;
    virtual const MagnetometerError configure() = 0;
    virtual const MagnetometerError calibrate() = 0;
    virtual VectorFloat readData() = 0;
    virtual const MagnetometerError readReg(uint8_t reg, uint8_t byte) = 0;
    virtual const MagnetometerError readReg(uint8_t reg, uint8_t *buf, size_t len, uint32_t timeout_ms) = 0;
    virtual const MagnetometerError writeReg(uint8_t reg, uint8_t byte) = 0;
    virtual const MagnetometerError writeReg(uint8_t reg, uint8_t *buf, size_t len) = 0;
  };

  struct Mpu9250MagI2C : Magnetometer
  {
    Mpu9250MagI2C(I2C_HandleTypeDef *bus);
    VectorFloat readData();
    const MagnetometerError readReg(uint8_t reg, uint8_t byte);
    const MagnetometerError readReg(uint8_t reg, uint8_t *buf, size_t len, uint32_t timeout_ms);
    const MagnetometerError writeReg(uint8_t reg, uint8_t byte);
    const MagnetometerError writeReg(uint8_t reg, uint8_t *buf, size_t len);

  private:
    I2C_HandleTypeDef *bus_;
  };

  struct Mpu9250MagSPI : Magnetometer
  {
    Mpu9250MagSPI(SPI_HandleTypeDef *bus);
    VectorFloat readData();
    const MagnetometerError readReg(uint8_t reg, uint8_t byte);
    const MagnetometerError readReg(uint8_t reg, uint8_t *buf, size_t len, uint32_t timeout_ms);
    const MagnetometerError writeReg(uint8_t reg, uint8_t byte);
    const MagnetometerError writeReg(uint8_t reg, uint8_t *buf, size_t len);

  private:
    SPI_HandleTypeDef *bus_;
  };
}