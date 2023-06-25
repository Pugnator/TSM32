#pragma once

#include "i2c.h"
#include "spi.h"
#include "trace.h"
#include "i2c_er.h"
#include "mpudefs.h"
#include <math.h>
#include <mems/math3d.h>
#include <memory>
#include <array>

#include "eeprom.h"

#include "stm32f4xx_hal.h"

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

/*
PC14-OSC32_IN.GPIO_Label=NSS
PA7.Signal=SPI1_MOSI
PA6.Signal=SPI1_MISO
PA5.Signal=SPI1_SCK
*/

//#define IMU_I2C_MODE
#define IMU_SPI_MODE

#ifdef IMU_SPI_MODE
#define MPU_SPI_PORT hspi1
#define MPU_CS_Pin GPIO_PIN_2
#define MPU_CS_GPIO_Port GPIOB
#endif

#define MAGNETIC_DECLINATION +5.46f

#define MPU9250_I2C_ADDR 0xD0
#define MPU9250_I2C_ADDR_MAG (0x0C << 1)
#define MPU9250_SPI_ADDR_MAG 0x0C

#define DMP_PACKET_SIZE 32
#define DMP_FIFO_SIZE 512

extern const uint8_t dmpFirmware[];
extern const uint32_t dmpFirmwareSize;

enum class InterruptSource : int
{
  NoInterrupt = 0,
  FreeFall,
  MotionDetection,
  ZeroMotionDetection,
  FifoOverflow,
  MasterInterrupt,
  DmpInterrupt,
  DataReady
};

typedef struct Axes3D
{
  float x;
  float y;
  float z;
} Axes3D;

typedef enum magMode
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
  MPU9250(I2C_HandleTypeDef *, bool dmpEnable = false);
  ~MPU9250();

public:
  bool ok();
  InterruptSource interruptStatus();

  bool startDMP();

  bool staticCalibration(Eeprom *mem = nullptr);
  bool loadCalibration(Eeprom *mem);

  bool readMagAxis(Axes3D &result);
  bool readAccelAxis(Axes3D &result);
  bool readGyroAxis(Axes3D &result);

  float getHeadingAngle();
  float getTemperature();

  uint16_t fifoDataReady();
  bool fifoReset();
  uint16_t fifoRead();

  void getQuaternion(Quaternion &q);
  void getYawPitchRoll(float *ypr, Quaternion &q);
  void yprToRadians(float *ypr, float *xyz);
  void yprToDegrees(float *ypr, float *xyz);
  void getEuler(float *output, Quaternion &q);

  void getAccel(VectorInt16 &v);
  void getGravity(VectorFloat &v, Quaternion &q);
  void getLinearAccel(VectorInt16 &v, VectorInt16 &vRaw, VectorFloat &gravity);

  bool dmpEnabled();

  bool enableDMP(bool enable);
  bool resetDMP();

private:
  void selfTest();

  void parseDmpPacket();

  bool mpuWrite(uint8_t address, uint8_t *byte, size_t len);
  bool mpuWrite(uint8_t address, uint8_t byte);
  bool mpuRead(uint8_t address, uint8_t *byte, size_t len = 1);

  void mpuSelect();
  void mpuDeselect();

  bool magWrite(uint8_t address, uint8_t *byte, size_t len);
  bool magWrite(uint8_t address, uint8_t byte);
  bool magRead(uint8_t address, uint8_t *byte, size_t len = 1, uint32_t timeout_ms = 1000);
  bool magWriteRegI2c(uint8_t address, uint8_t *byte, size_t len);
  bool magWriteRegI2c(uint8_t address, uint8_t byte);
  bool magReadRegI2c(uint8_t address, uint8_t *byte, size_t len = 1, uint32_t timeout_ms = 1000);
  bool magWriteRegSpi(uint8_t address, uint8_t *byte, size_t len);
  bool magWriteRegSpi(uint8_t address, uint8_t byte);
  bool magReadRegSpi(uint8_t address, uint8_t *byte, size_t len = 1, uint32_t timeout_ms = 1000);
  bool magReset();
  bool magSetMode(magMode mode);

  bool setup();
  bool configureAccelerometer();
  bool configureGyroscope();
  bool configureMagnetometer();

  void gyroReset();
  void gyroAutoOffset(Axes3D &axes);
  void accAutoOffset(Axes3D &axes);
  void magAutoOffset(Axes3D &axes);

  float magFactoryCorrX;
  float magFactoryCorrY;
  float magFactoryCorrZ;

  float magOffsetX;
  float magOffsetY;
  float magOffsetZ;

  float magScaleX;
  float magScaleY;
  float magScaleZ;

  float accOffsetX;
  float accOffsetY;
  float accOffsetZ;

  float gyroOffsetX;
  float gyroOffsetY;
  float gyroOffsetZ;

  float magMaxX;
  float magMaxY;
  float magMaxZ;
  float magMinX;
  float magMinY;
  float magMinZ;

  float accMaxX;
  float accMaxY;
  float accMaxZ;
  float accMinX;
  float accMinY;
  float accMinZ;

  float gyroMaxX;
  float gyroMaxY;
  float gyroMaxZ;
  float gyroMinX;
  float gyroMinY;
  float gyroMinZ;

  float aMult;
  float gMult;
  float gSensF;

  float chipTemperature_;

  bool ok_;
  bool isCalibration_;
  bool dmpEnabled_;

  int16_t gyro[3];
  int16_t accel[3];
  float mag[3];
  int32_t quat[4];
  uint8_t fifoBuffer[DMP_FIFO_SIZE];
  I2C_HandleTypeDef *i2c;
};

class AttitudeHeadingReferenceSystem
{
  AttitudeHeadingReferenceSystem();
};