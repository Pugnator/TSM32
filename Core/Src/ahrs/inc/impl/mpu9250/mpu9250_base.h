#pragma once

#include "trace.h"
#include "mpudefs.h"
#include "../../math3d.h"
#include "imu_base.h"
#include <array>
#include <math.h>

#include "stm32f1xx_hal.h"

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

#define EEPROM_MPU_CALIB_ADDRESS 0

#define MAGNETIC_DECLINATION +5.46f

#define MPU9250_I2C_ADDR 0xD0
#define MPU9250_I2C_ADDR_MAG (0x0C << 1)
#define MPU9250_I2C_SLAVE_ADDR_MAG 0x0C

#define DMP_PACKET_SIZE 32
#define DMP_FIFO_SIZE 512

namespace Mpu9250
{
  extern const uint8_t dmpFirmware[];
  extern const uint32_t dmpFirmwareSize;

  enum class MagMode
  {
    MasterMode = 0,
    SlaveMode,
  };

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

  enum class IoError
  {
    Ok,
    ReadError,
    WriteError,
    NotDetected,
    NotImplemented,
  };

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

  class Mpu9250base : public ImuBase::ImuAbstract
  {
  public:
    ~Mpu9250base(){};

  public:
    bool ok();
    InterruptSource interruptStatus();

    bool readMagAxis(VectorFloat &result, bool blocking = true);
    bool readAccelAxis(VectorFloat &result);
    bool readGyroAxis(VectorFloat &result);
    bool setMagnetometerAsSlave();
    float getTemperature();

    uint16_t fifoDataReady();
    bool fifoReset();
    uint16_t fifoRead();
    uint32_t captureTime();

    void getDmpQuaternion(Quaternion &q);
    void getDmpAccel(VectorInt16 &vectorOut);

    bool dmpEnabled();
    bool startDMP();
    bool enableDMP(bool enable);
    bool resetDMP();

  protected:
    virtual bool mpuWrite(uint8_t address, uint8_t *byte, uint32_t len) { return false; };
    virtual bool mpuWrite(uint8_t address, uint8_t byte) { return false; };
    virtual bool mpuRead(uint8_t address, uint8_t *byte, uint32_t len = 1) { return false; };
    // Implemented in this class
    bool mpuWriteBit(uint8_t address, uint8_t bitNumber, bool state);
    bool mpuWriteBits(uint8_t address, uint8_t startBit, uint8_t length, uint8_t data);
    // Implemented in this class
    bool mpuReadBit(uint8_t address, uint8_t bitNumber, bool *state);
    bool mpuReadBits(uint8_t address, uint8_t startBit, uint8_t length, uint8_t *data);

    virtual bool magWrite(uint8_t address, uint8_t *byte, uint32_t len) { return false; };
    virtual bool magWrite(uint8_t address, uint8_t byte) { return false; };
    virtual bool magRead(uint8_t address, uint8_t *byte, uint32_t len = 1, uint32_t timeout_ms = 1000) { return false; };
    virtual bool setup() = 0;
    void selfTest();

    void parseDmpPacket();

    bool magReset();
    void gyroReset();

    bool magSetMode(magMode mode);

    bool configureAccelerometer();
    bool configureGyroscope();
    bool configureMagnetometer();

    bool gyroCurrentBias();
    bool accelCurrentBias();
    bool magCurrentBias();

    float magFactoryCorrX;
    float magFactoryCorrY;
    float magFactoryCorrZ;

    float magScaleX;
    float magScaleY;
    float magScaleZ;

    float aMult;
    float gMult;
    float gSensF;
    
    VectorInt16 magOffset;

    float chipTemperature_;

    bool ok_;
    bool useDmp_;
    MagMode magMode_;

    int16_t gyro[3];
    int16_t accel[3];
    float mag[3];
    int32_t quat[4];
    uint32_t captureTime_;
    uint8_t fifoBuffer[DMP_FIFO_SIZE];
  };
}