#include "tsm.h"
#include "mpu.h"
#include "settings.h"

#ifdef __cplusplus
extern "C" {
#endif

#if MEMS_ENABLED

#define MPU9250_ADDRESS 0x69

uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

void mpu_select()
{
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
}

void mpu_deselect()
{
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
}

uint32_t mpu_writereg(uint8_t WriteAddr, uint8_t WriteData)
{
  mpu_select();
  HAL_SPI_Transmit(&hspi1, &WriteAddr, 1, 100);
  uint8_t temp_val = 0;
  HAL_SPI_TransmitReceive(&hspi1, &WriteAddr, &temp_val, 1, 100);
  mpu_deselect();
  return temp_val;
}

uint32_t mpu_readreg(uint8_t WriteAddr, uint8_t WriteData)
{
  return mpu_writereg(WriteAddr | 0x80, WriteData);
}

void mpu_readregs(uint8_t ReadAddr, uint8_t *ReadBuf, uint32_t Bytes)
{
  mpu_select();
  uint8_t raddr = ReadAddr | 0x80;
  HAL_SPI_Transmit(&hspi1, &raddr, 1, 100);
  for (uint32_t i = 0; i < Bytes; i++)
  {
    uint8_t temp_val = 0;
    uint8_t waddr = 0x00;
    HAL_SPI_TransmitReceive(&hspi1, &waddr, &temp_val, 1, 100);
    ReadBuf[i] = temp_val;
  }

  mpu_deselect();
  //HAL_DelayMicroseconds(50);
}

uint32_t mpu_whoami()
{
  uint32_t response = 0;
  for (uint_fast8_t i = 100; i >= 0 && !response; i--)
  {
    response = mpu_readreg(MPU9250_WHO_AM_I, 0x00);
    HAL_Delay(1);
  }

  return response;
}

uint32_t AK8963_whoami()
{
  return AK8963ReadByte(MPU9250_AK8963_WIA);
}

uint8_t AK8963ReadByte(uint8_t reg)
{
  mpu_writereg(MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
  mpu_writereg(MPU9250_I2C_SLV0_REG, reg);                                  //I2C slave 0 register address from where to begin data transfer
  return mpu_readreg(MPU9250_I2C_SLV0_CTRL, 0x81);                          //Read 1 byte from the magnetometer
}

uint32_t mems_read_temp()
{
  uint8_t response[2];
  mpu_readregs(MPU9250_TEMP_OUT_H, response, 2);
  int16_t bit_data = ((int16_t)response[0] << 8) | response[1];
  float data = (float)bit_data;
  uint32_t temperature = ((data - 21) / 333.87) + 21;
  mpu_deselect();
  return temperature;
}

void print_hex(uint8_t *buf, uint8_t size)
{
  Print("\r\n");
  for (uint8_t i = 0; i < size; i++)
  {
    if (0 == i % 8)
    {
      Print("\r\n");
    }
    PrintF("0x%02X ", buf[size]);
  }
  Print("\r\n");
}

void mems_read_all()
{
  PrintF("MPU temp is %uC\r\n", mems_read_temp());
  uint8_t response[21];
  mpu_readregs(MPU9250_ACCEL_XOUT_H, response, 21);
  print_hex(response, 21);
}

void mems_reset()
{
  mpu_writereg(MPU9250_PWR_MGMT_1, 0x80);
  HAL_Delay(100);
}

#endif

void mems_setup()
{
#if MEMS_ENABLED
  Print("*** MEMS setup start ***\r\n");
  PrintF("MPU9250 - 0x%X\r\n", mpu_whoami());
  PrintF("AK8963 - 0x%X\r\n", AK8963_whoami());
  PrintF("MPU temp is %uC\r\n", mems_read_temp());
  PrintF("X offset - 0x%X\r\n", mpu_readreg(MPU9250_SELF_TEST_X_ACCEL, 0x00));
  //mpu_writereg(MPU9250_XA_OFFSET_L, 0x01);
  //PrintF("X offset - 0x%X\r\n", mpu_readreg(MPU9250_XA_OFFSET_L, 0x00));
  Print("*** MEMS setup end ***\r\n");
#else
  Print("MEMS disabled\r\n");
#endif
}

#ifdef __cplusplus
}
#endif