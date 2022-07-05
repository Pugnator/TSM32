#pragma once

#include "i2c.h"
#include "trace.h"
#include "i2c_er.h"
#include <math.h>
#include "mpudefs.h"

// The missile knows where it is because it knows where it's not.

/*
Magnetic Declination: +11° 40'
Declination is POSITIVE (EAST)
Inclination: 71° 37'
Magnetic field strength: 52788.7 nT
*/

#define MAGNETIC_DECLINATION 11

#define MS_TO_S 1000
#define G_TO_MS2 9.8115
#define DEG_TO_RAD (M_PI / 180)
#define RAD_TO_DEG (180 / M_PI)
#define FREQUENCY 125                                    // интервал сэмплирования = 8 мс
#define GYRO_SENSITIVITY 65.5                            // чувствительность гироскопа (см. datasheet Gyro_Sensitivity)
#define SENS_TO_DEG (1 / (GYRO_SENSITIVITY * FREQUENCY)) // макрос преобразования показаний датчика в градусы
#define SENS_TO_RAD SENS_TO_DEG *DEG_TO_RAD              // макрос преобразования показаний датчика в радианы
#define AK8963_ADDR 0x0C << 1

#define MPU9250_I2C_ADDR 0xD0
#define MPU9250_I2C_ADDR_MAG (0x0C << 1)

class MPU9250
{
public:
  MPU9250(I2C_HandleTypeDef *);
  ~MPU9250();

public:
  bool ok();
  bool ready();

  bool readA();
  bool readM();
  bool readG();

private:
  bool write(uint8_t reg, uint8_t byt);
  bool read(uint8_t reg, uint8_t *byte);
  bool read(uint8_t reg, uint8_t *byte, size_t len);
  void reset();
  bool initacc();
  bool initmag();

  void kalmanInit(float mea_e, float est_e, float q);
  float updateEstimate(float mea);
  void setMeasurementError(float mea_e);
  void setEstimateError(float est_e);
  void setProcessNoise(float q);
  float getKalmanGain();
  float getEstimateError();

  I2C_HandleTypeDef *i2c;
  
  bool _ok;
  float aMult; 
  float gMult; 
  float mMult;

  float _err_measure;
  float _err_estimate;
  float _q;
  float _current_estimate;
  float _last_estimate;
  float _kalman_gain;
};
