#pragma once

#include "i2c.h"
#include "trace.h"
#include "i2c_er.h"
#include "mpudefs.h"
#include <math.h>
#include <memory>

/*
Missile Guidance For Dummies

The missile knows where it is at all times. It knows this because it knows where it isn't.
By subtracting where it is from where it isn't, or where it isn't from where it is (whichever is greater),
it obtains a difference, or deviation. The guidance subsystem uses deviations to generate
corrective commands to drive the missile from a position where it is to a position where it isn't,
and arriving at a position where it wasn't, it now is. Consequently, the position where it is,
is now the position that it wasn't, and it follows that the position that it was, is now the position that it isn't.

In the event that the position that it is in is not the position that it wasn't,
the system has acquired a variation, the variation being the difference between where the missile is,
and where it wasn't. If variation is considered to be a significant factor, it too may be corrected by the GEA.
However, the missile must also know where it was.

The missile guidance computer scenario works as follows.
Because a variation has modified some of the information the missile has obtained,
it is not sure just where it is. However, it is sure where it isn't, within reason,
and it knows where it was. It now subtracts where it should be from where it wasn't,
or vice-versa, and by differentiating this from the algebraic sum of where it shouldn't be,
and where it was, it is able to obtain the deviation and its variation, which is called error.
*/

/*
Magnetic Declination: +11° 40'
Declination is POSITIVE (EAST)
Inclination: 71° 37'
Magnetic field strength: 52788.7 nT
*/

// Moscow
// #define MAGNETIC_DECLINATION 11.0f
// Belgrade
#define MAGNETIC_DECLINATION +5.46f

#define G_TO_MS2 9.8115
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)
#define FREQUENCY 125
#define GYRO_SENSITIVITY 65.5
#define SENS_TO_DEG (1 / (GYRO_SENSITIVITY * FREQUENCY))
#define SENS_TO_RAD SENS_TO_DEG *DEG_TO_RAD

#define MPU9250_I2C_ADDR 0xD0
#define MPU9250_I2C_ADDR_MAG 0x0C << 1

#define AK8963_CALIBRATION_LOOPS 1000
extern float az;
extern float initialAzimuth;

typedef struct Axis3D
{
  float x;
  float y;
  float z;

} Axis3D;

typedef enum
{
  MAG_MODE_PD,
  MAG_MODE_SINGLE,
  MAG_MODE_CONT_8HZ,
  MAG_MODE_CONT_100HZ,
  MAG_MODE_SELF_TEST,
  MAG_MODE_FUSE_ROM,
} magMode;

#define AKM_CONT_MODE_8 0b0001001
#define AKM_PWDWN_MODE 0b0000000
#define AKM_RESET 0b0000001

class MPU9250
{
public:
  MPU9250(I2C_HandleTypeDef *);
  ~MPU9250();

public:
  bool ok();
  bool ready();

  void selfTest();
  void magCalibration();

  bool readAccel();
  Axis3D readMag();
  Axis3D readGyro();

  float getHeadingAngle();  

  void scanBus();

private:
  bool writeRegMpu(uint8_t reg, uint8_t *byte, size_t len);
  bool writeRegMpu(uint8_t reg, uint8_t &&byte);
  bool readRegMpu(uint8_t reg, uint8_t *byte, size_t len = 1);

  bool writeRegMag(uint8_t reg, uint8_t *byte, size_t len);
  bool writeRegMag(uint8_t reg, uint8_t &&byte);
  bool readRegMag(uint8_t reg, uint8_t *byte, size_t len = 1, uint32_t timeout_ms = 1000);
  bool magRST();
  bool magSetMode(magMode mode);
  

  void reset();
  bool initAcc();
  bool initMag();

  float kalmanFilter(float val);

  bool _ok;
  float corrX;
  float corrY;
  float corrZ;
  float magOffsetX;
  float magOffsetY;
  float magOffsetZ;
  float mScaleX;
  float mScaleY;
  float mScaleZ;

  float aMult;
  float gMult;
  float gSensF;

  I2C_HandleTypeDef *i2c;
};

extern std::unique_ptr<MPU9250> ahrs;