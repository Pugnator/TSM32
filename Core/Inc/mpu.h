#pragma once

#include "i2c.h"
#include "trace.h"
#include "i2c_er.h"
#include <math.h>
#include "mpudefs.h"
#include <memory>

// The missile knows where it is because it knows where it's not.

/*
Magnetic Declination: +11° 40'
Declination is POSITIVE (EAST)
Inclination: 71° 37'
Magnetic field strength: 52788.7 nT
*/

#define MAGNETIC_DECLINATION 11.0f

#define MS_TO_S 1000
#define G_TO_MS2 9.8115
#define DEG_TO_RAD (M_PI / 180)
#define RAD_TO_DEG (180 / M_PI)
#define FREQUENCY 125                                    // интервал сэмплирования = 8 мс
#define GYRO_SENSITIVITY 65.5                            // чувствительность гироскопа (см. datasheet Gyro_Sensitivity)
#define SENS_TO_DEG (1 / (GYRO_SENSITIVITY * FREQUENCY)) // макрос преобразования показаний датчика в градусы
#define SENS_TO_RAD SENS_TO_DEG *DEG_TO_RAD              // макрос преобразования показаний датчика в радианы

#define MPU9250_I2C_ADDR 0xD0
#define MPU9250_I2C_ADDR_MAG 0x0C << 1

typedef union axes
{
  float x;
  float y;
  float z;

} axes;

/// 0 0 0 BIT MODE3 MODE2 MODE1 MODE0
#define AKM_CONT_MODE_8 0b0001001
#define AKM_PWDWN_MODE 0b0000000
#define AKM_RESET 0b0000001

class kalmanFilter
{
public:
  kalmanFilter(float mea_e = 2, float est_e = 2, float q = 0.01);
  ~kalmanFilter(){};

  float updateEstimate(float mea);

private:
  float _err_measure;
  float _err_estimate;
  float _q;
  float _current_estimate;
  float _last_estimate;
  float _kalman_gain;
};

class MPU9250
{
public:
  MPU9250(I2C_HandleTypeDef *);
  ~MPU9250();

public:
  bool ok();
  bool ready();

  void printall();

  void selfTest();

  bool readAccel();
  axes readMag();
  bool readGyro();

  float getAzimuth();

  void scanBus();

private:
  bool writeRegMpu(uint8_t reg, uint8_t *byte, size_t len);  
  bool writeRegMpu(uint8_t reg, uint8_t &&byte);
  bool readRegMpu(uint8_t reg, uint8_t *byte, size_t len = 1);

  bool writeRegMag(uint8_t reg, uint8_t *byte, size_t len);
  bool writeRegMag(uint8_t reg, uint8_t &&byte);
  bool readRegMag(uint8_t reg, uint8_t *byte, size_t len = 1);
  void resetMag();

  void reset();
  bool initAcc();
  bool initMag();

  I2C_HandleTypeDef *i2c;

  std::unique_ptr<kalmanFilter> aKalman;
  std::unique_ptr<kalmanFilter> gKalman;
  std::unique_ptr<kalmanFilter> mKalman;

  bool _ok;
  float aMult;
  float gMult;
  float mMult;

  float adjX;
  float adjY;
  float adjZ;
};
