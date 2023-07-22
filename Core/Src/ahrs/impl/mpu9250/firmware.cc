
#include <cstdint>
#include "mpu9250_base.h"

/*
Embedded MotionDriver 6.12 Firmware instance. 
https://www.invensense.com/developers/software-downloads/#sla_content_45 

Jump address 0x0400
*/

/* The mounting matrix below tells the MPL how to rotate the raw
   data from the driver(s). The matrix below reflects the axis
   orientation of the MPU-6050 on the nav6 circuit board.
*/

/* for your understanding this is the Gyro orentation Hard coded into the MPU With this instance of the DMP
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1
                                         };
*/


/* This is a pre Configured Version of the Embedded MotionDriver 6.12 Firmware instance. 
 *  https://www.invensense.com/developers/software-downloads/#sla_content_45 (requires account)
 *  This is based on the Latest release as of 21st of June 2019
 *  NOTE: This firmware instance is intended to be used with this class and may not work as 
 *  expected if used with other mpu6050 configuration programs.
 *  
 *  It's a miracle! The DMP is Pre Configured to work!!!!!
 *  Actually I used another program to load Configure, test and verify that the DMP was working. 
 *  Then I simply read the DMP Firmware back using view_DMP_firmware_Instance() function
 *  I triggered this near the beginning of the program. I don't believe there is a problem 
 *  with triggering it after it has ran but you can see that the DMP firmware changes over time.
 *  as the MPU6050 modifies [portions of the firmware memory block while active.
 */

namespace Mpu9250
{
   const uint8_t dmpFirmware[] = 
   {
   /* bank # 0 */
   0x00, 0xF8, 0xF6, 0x2A, 0x3F, 0x68, 0xF5, 0x7A, 0x00, 0x06, 0xFF, 0xFE, 0x00, 0x03, 0x00, 0x00,  //0-15
   0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,  //16-31
   0x03, 0x0C, 0x30, 0xC3, 0x0A, 0x74, 0x56, 0x2D, 0x0D, 0x62, 0xDB, 0xC7, 0x16, 0xF4, 0xBA, 0x02,  //32
   0x38, 0x83, 0xF8, 0x83, 0x30, 0x00, 0xF8, 0x83, 0x25, 0x8E, 0xF8, 0x83, 0x30, 0x00, 0xF8, 0x83,  //48
   0xFF, 0xFF, 0xFF, 0xFF, 0x0C, 0xBD, 0xD8, 0x11, 0x24, 0x00, 0x04, 0x00, 0x1A, 0x82, 0x79, 0xA1,  //64  
   0x00, 0x36, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x38, 0x83, 0x6F, 0xA2,  //80
   0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,  //96
   0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,  //112
   0x1F, 0xA4, 0xE8, 0xE4, 0xFF, 0xF5, 0xDC, 0xB9, 0x00, 0x5B, 0x79, 0xCF, 0x1F, 0x3F, 0x78, 0x76,  //128
   0x00, 0x86, 0x7C, 0x5A, 0x00, 0x86, 0x23, 0x47, 0xFA, 0xB9, 0x86, 0x31, 0x00, 0x74, 0x87, 0x8A,  //144
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x05, 0xFF, 0xFF, 0xE9, 0xA8, 0x00, 0x00, 0x21, 0x82,  //160
   0xFA, 0xB8, 0x4D, 0x46, 0xFF, 0xFA, 0xDF, 0x3D, 0xFF, 0xFF, 0xB2, 0xB3, 0x00, 0x00, 0x00, 0x00,  //176
   0x3F, 0xFF, 0xBA, 0x98, 0x00, 0x5D, 0xAC, 0x08, 0x00, 0x0A, 0x63, 0x78, 0x00, 0x01, 0x46, 0x21,  //192
   0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x42, 0xB5, 0x00, 0x06, 0x00, 0x64, 0x00, 0x64, 0x00, 0x06,  //208
   0x14, 0x06, 0x02, 0x9F, 0x0F, 0x47, 0x91, 0x32, 0xD9, 0x0E, 0x9F, 0xC9, 0x1D, 0xCF, 0x4C, 0x34,  //224
   0x3B, 0xB6, 0x7A, 0xE8, 0x00, 0x64, 0x00, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFE,  //240 - 255
   /* bank # 1 */
   0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  //256
   0x00, 0x07, 0x00, 0x00, 0xFF, 0xF1, 0x00, 0x00, 0xFA, 0x46, 0x00, 0x00, 0xA2, 0xB8, 0x00, 0x00,
   0x10, 0x00, 0x00, 0x00, 0x04, 0xD6, 0x00, 0x00, 0x04, 0xCC, 0x00, 0x00, 0x04, 0xCC, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x06, 0x00, 0x02, 0x00, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x05, 0x00, 0x64, 0x00, 0x20, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x03, 0x00,
   0x00, 0x00, 0x00, 0x32, 0xF8, 0x98, 0x00, 0x00, 0xFF, 0x65, 0x00, 0x00, 0x83, 0x0F, 0x00, 0x00,
   0x00, 0x06, 0x00, 0x00, 0xFF, 0xF1, 0x00, 0x00, 0xFA, 0x46, 0x00, 0x00, 0xA2, 0xB8, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0D, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x02, 0x00, 0x00,
   0x00, 0x01, 0xFB, 0x83, 0x00, 0x7C, 0x00, 0x00, 0xFB, 0x15, 0xFC, 0x00, 0x1F, 0xB4, 0xFF, 0x83,
   0x00, 0x00, 0x00, 0x01, 0x00, 0x65, 0x00, 0x07, 0x00, 0x64, 0x03, 0xE8, 0x00, 0x64, 0x00, 0x28,
   0x00, 0x00, 0xFF, 0xFD, 0x00, 0x00, 0x00, 0x00, 0x16, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x10, 0x00, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF4, 0x00, 0x00, 0x10, 0x00,
   /* bank # 2 */
   0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00, //512
   0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x00, 0x03, 0x00, 0x05, 0xBA, 0xC6, 0x00, 0x47, 0x78, 0xA2, //FIFO rate
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
   0x00, 0x00, 0x23, 0xBB, 0x00, 0x2E, 0xA2, 0x5B, 0x00, 0x00, 0x05, 0x68, 0x00, 0x0B, 0xCF, 0x49,
   0x00, 0x04, 0xFF, 0xFD, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x64, 0x00, 0x07, 0x00, 0x08, 0x00, 0x06, 0x00, 0x06, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x2E, 0xA2, 0x5B, 0x00, 0x00, 0x05, 0x68, 0x00, 0x0B, 0xCF, 0x49, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xF8, 0xF6, 0x2A, 0x3F, 0x68, 0xF5, 0x7A, 0x00, 0x04, 0xFF, 0xFD, 0x00, 0x02, 0x00, 0x00,
   0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x0E,
   0xFF, 0xFF, 0xFF, 0xCF, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xFF, 0xFF, 0xFF, 0x9C,
   0x00, 0x00, 0x43, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x64,
   0xFF, 0xE5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   /* bank # 3 */
   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //768
   0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x24, 0x26, 0xD3,
   0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x10, 0x00, 0x96, 0x00, 0x3C,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x9E, 0x65, 0x5D,
   0x0C, 0x0A, 0x4E, 0x68, 0xCD, 0xCF, 0x77, 0x09, 0x50, 0x16, 0x67, 0x59, 0xC6, 0x19, 0xCE, 0x82,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x71, 0x1C,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xD7, 0x84, 0x00, 0x03, 0x00, 0x00, 0x00,
   0x00, 0x11, 0xDC, 0x47, 0x03, 0x00, 0x00, 0x00, 0xC7, 0x93, 0x8F, 0x9D, 0x1E, 0x1B, 0x1C, 0x19,
   0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0E, 0xDF, 0xA4, 0x38, 0x1F, 0x9E, 0x65, 0x5D,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x71, 0x1C, 0x02, 0x03, 0x18, 0x85, 0x00, 0x00, 0x40, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x3F, 0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xF4, 0xC9, 0xFF, 0xFF, 0xBC, 0xF0, 0x00, 0x01, 0x0C, 0x0F,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xF5, 0xB7, 0xBA, 0xB3, 0x67, 0x7D, 0xDF, 0x7E, 0x72, 0x90, 0x2E, 0x55, 0x4C, 0xF6, 0xE6, 0x88,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   /* bank # 4 */
   0xD8, 0xDC, 0xB4, 0xB8, 0xB0, 0xD8, 0xB9, 0xAB, 0xF3, 0xF8, 0xFA, 0xB3, 0xB7, 0xBB, 0x8E, 0x9E, //1024
   0xAE, 0xF1, 0x32, 0xF5, 0x1B, 0xF1, 0xB4, 0xB8, 0xB0, 0x80, 0x97, 0xF1, 0xA9, 0xDF, 0xDF, 0xDF,
   0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0x4C, 0xCD, 0x6C, 0xA9, 0x0C, 0xC9, 0x2C, 0x97, 0xF1, 0xA9,
   0x89, 0x26, 0x46, 0x66, 0xB2, 0x89, 0x99, 0xA9, 0x2D, 0x55, 0x7D, 0xB0, 0xB0, 0x8A, 0xA8, 0x96,
   0x36, 0x56, 0x76, 0xF1, 0xBA, 0xA3, 0xB4, 0xB2, 0x80, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB2, 0x83,
   0x98, 0xBA, 0xA3, 0xF0, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xB2, 0xB9, 0xB4, 0x98, 0x83, 0xF1,
   0xA3, 0x29, 0x55, 0x7D, 0xBA, 0xB5, 0xB1, 0xA3, 0x83, 0x93, 0xF0, 0x00, 0x28, 0x50, 0xF5, 0xB2,
   0xB6, 0xAA, 0x83, 0x93, 0x28, 0x54, 0x7C, 0xF1, 0xB9, 0xA3, 0x82, 0x93, 0x61, 0xBA, 0xA2, 0xDA,
   0xDE, 0xDF, 0xDB, 0x81, 0x9A, 0xB9, 0xAE, 0xF5, 0x60, 0x68, 0x70, 0xF1, 0xDA, 0xBA, 0xA2, 0xDF,
   0xD9, 0xBA, 0xA2, 0xFA, 0xB9, 0xA3, 0x82, 0x92, 0xDB, 0x31, 0xBA, 0xA2, 0xD9, 0xBA, 0xA2, 0xF8,
   0xDF, 0x85, 0xA4, 0xD0, 0xC1, 0xBB, 0xAD, 0x83, 0xC2, 0xC5, 0xC7, 0xB8, 0xA2, 0xDF, 0xDF, 0xDF,
   0xBA, 0xA0, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xAA, 0xB3, 0x8D, 0xB4, 0x98, 0x0D, 0x35,
   0x5D, 0xB2, 0xB6, 0xBA, 0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A,
   0xB8, 0xAA, 0x87, 0x2C, 0x54, 0x7C, 0xBA, 0xA4, 0xB0, 0x8A, 0xB6, 0x91, 0x32, 0x56, 0x76, 0xB2,
   0x84, 0x94, 0xA4, 0xC8, 0x08, 0xCD, 0xD8, 0xB8, 0xB4, 0xB0, 0xF1, 0x99, 0x82, 0xA8, 0x2D, 0x55,
   0x7D, 0x98, 0xA8, 0x0E, 0x16, 0x1E, 0xA2, 0x2C, 0x54, 0x7C, 0x92, 0xA4, 0xF0, 0x2C, 0x50, 0x78,
   /* bank # 5 */
   0xF1, 0x84, 0xA8, 0x98, 0xC4, 0xCD, 0xFC, 0xD8, 0x0D, 0xDB, 0xA8, 0xFC, 0x2D, 0xF3, 0xD9, 0xBA, //1280
   0xA6, 0xF8, 0xDA, 0xBA, 0xA6, 0xDE, 0xD8, 0xBA, 0xB2, 0xB6, 0x86, 0x96, 0xA6, 0xD0, 0xF3, 0xC8,
   0x41, 0xDA, 0xA6, 0xC8, 0xF8, 0xD8, 0xB0, 0xB4, 0xB8, 0x82, 0xA8, 0x92, 0xF5, 0x2C, 0x54, 0x88,
   0x98, 0xF1, 0x35, 0xD9, 0xF4, 0x18, 0xD8, 0xF1, 0xA2, 0xD0, 0xF8, 0xF9, 0xA8, 0x84, 0xD9, 0xC7,
   0xDF, 0xF8, 0xF8, 0x83, 0xC5, 0xDA, 0xDF, 0x69, 0xDF, 0x83, 0xC1, 0xD8, 0xF4, 0x01, 0x14, 0xF1,
   0xA8, 0x82, 0x4E, 0xA8, 0x84, 0xF3, 0x11, 0xD1, 0x82, 0xF5, 0xD9, 0x92, 0x28, 0x97, 0x88, 0xF1,
   0x09, 0xF4, 0x1C, 0x1C, 0xD8, 0x84, 0xA8, 0xF3, 0xC0, 0xF9, 0xD1, 0xD9, 0x97, 0x82, 0xF1, 0x29,
   0xF4, 0x0D, 0xD8, 0xF3, 0xF9, 0xF9, 0xD1, 0xD9, 0x82, 0xF4, 0xC2, 0x03, 0xD8, 0xDE, 0xDF, 0x1A,
   0xD8, 0xF1, 0xA2, 0xFA, 0xF9, 0xA8, 0x84, 0x98, 0xD9, 0xC7, 0xDF, 0xF8, 0xF8, 0xF8, 0x83, 0xC7,
   0xDA, 0xDF, 0x69, 0xDF, 0xF8, 0x83, 0xC3, 0xD8, 0xF4, 0x01, 0x14, 0xF1, 0x98, 0xA8, 0x82, 0x2E,
   0xA8, 0x84, 0xF3, 0x11, 0xD1, 0x82, 0xF5, 0xD9, 0x92, 0x50, 0x97, 0x88, 0xF1, 0x09, 0xF4, 0x1C,
   0xD8, 0x84, 0xA8, 0xF3, 0xC0, 0xF8, 0xF9, 0xD1, 0xD9, 0x97, 0x82, 0xF1, 0x49, 0xF4, 0x0D, 0xD8,
   0xF3, 0xF9, 0xF9, 0xD1, 0xD9, 0x82, 0xF4, 0xC4, 0x03, 0xD8, 0xDE, 0xDF, 0xD8, 0xF1, 0xAD, 0x88,
   0x98, 0xCC, 0xA8, 0x09, 0xF9, 0xD9, 0x82, 0x92, 0xA8, 0xF5, 0x7C, 0xF1, 0x88, 0x3A, 0xCF, 0x94,
   0x4A, 0x6E, 0x98, 0xDB, 0x69, 0x31, 0xDA, 0xAD, 0xF2, 0xDE, 0xF9, 0xD8, 0x87, 0x95, 0xA8, 0xF2,
   0x21, 0xD1, 0xDA, 0xA5, 0xF9, 0xF4, 0x17, 0xD9, 0xF1, 0xAE, 0x8E, 0xD0, 0xC0, 0xC3, 0xAE, 0x82,
   /* bank # 6 */
   0xC6, 0x84, 0xC3, 0xA8, 0x85, 0x95, 0xC8, 0xA5, 0x88, 0xF2, 0xC0, 0xF1, 0xF4, 0x01, 0x0E, 0xF1, // 1536
   0x8E, 0x9E, 0xA8, 0xC6, 0x3E, 0x56, 0xF5, 0x54, 0xF1, 0x88, 0x72, 0xF4, 0x01, 0x15, 0xF1, 0x98,
   0x45, 0x85, 0x6E, 0xF5, 0x8E, 0x9E, 0x04, 0x88, 0xF1, 0x42, 0x98, 0x5A, 0x8E, 0x9E, 0x06, 0x88,
   0x69, 0xF4, 0x01, 0x1C, 0xF1, 0x98, 0x1E, 0x11, 0x08, 0xD0, 0xF5, 0x04, 0xF1, 0x1E, 0x97, 0x02,
   0x02, 0x98, 0x36, 0x25, 0xDB, 0xF9, 0xD9, 0x85, 0xA5, 0xF3, 0xC1, 0xDA, 0x85, 0xA5, 0xF3, 0xDF,
   0xD8, 0x85, 0x95, 0xA8, 0xF3, 0x09, 0xDA, 0xA5, 0xFA, 0xD8, 0x82, 0x92, 0xA8, 0xF5, 0x78, 0xF1,
   0x88, 0x1A, 0x84, 0x9F, 0x26, 0x88, 0x98, 0x21, 0xDA, 0xF4, 0x1D, 0xF3, 0xD8, 0x87, 0x9F, 0x39,
   0xD1, 0xAF, 0xD9, 0xDF, 0xDF, 0xFB, 0xF9, 0xF4, 0x0C, 0xF3, 0xD8, 0xFA, 0xD0, 0xF8, 0xDA, 0xF9,
   0xF9, 0xD0, 0xDF, 0xD9, 0xF9, 0xD8, 0xF4, 0x0B, 0xD8, 0xF3, 0x87, 0x9F, 0x39, 0xD1, 0xAF, 0xD9,
   0xDF, 0xDF, 0xF4, 0x1D, 0xF3, 0xD8, 0xFA, 0xFC, 0xA8, 0x69, 0xF9, 0xF9, 0xAF, 0xD0, 0xDA, 0xDE,
   0xFA, 0xD9, 0xF8, 0x8F, 0x9F, 0xA8, 0xF1, 0xCC, 0xF3, 0x98, 0xDB, 0x45, 0xD9, 0xAF, 0xDF, 0xD0,
   0xF8, 0xD8, 0xF1, 0x8F, 0x9F, 0xA8, 0xCA, 0xF3, 0x88, 0x09, 0xDA, 0xAF, 0x8F, 0xCB, 0xF8, 0xD8,
   0xF2, 0xAD, 0x97, 0x8D, 0x0C, 0xD9, 0xA5, 0xDF, 0xF9, 0xBA, 0xA6, 0xF3, 0xFA, 0xF4, 0x12, 0xF2,
   0xD8, 0x95, 0x0D, 0xD1, 0xD9, 0xBA, 0xA6, 0xF3, 0xFA, 0xDA, 0xA5, 0xF2, 0xC1, 0xBA, 0xA6, 0xF3,
   0xDF, 0xD8, 0xF1, 0xBA, 0xB2, 0xB6, 0x86, 0x96, 0xA6, 0xD0, 0xCA, 0xF3, 0x49, 0xDA, 0xA6, 0xCB,
   0xF8, 0xD8, 0xB0, 0xB4, 0xB8, 0xD8, 0xAD, 0x84, 0xF2, 0xC0, 0xDF, 0xF1, 0x8F, 0xCB, 0xC3, 0xA8,
   /* bank # 7 */
   0xB2, 0xB6, 0x86, 0x96, 0xC8, 0xC1, 0xCB, 0xC3, 0xF3, 0xB0, 0xB4, 0x88, 0x98, 0xA8, 0x21, 0xDB, // 1792
   0x71, 0x8D, 0x9D, 0x71, 0x85, 0x95, 0x21, 0xD9, 0xAD, 0xF2, 0xFA, 0xD8, 0x85, 0x97, 0xA8, 0x28,
   0xD9, 0xF4, 0x08, 0xD8, 0xF2, 0x8D, 0x29, 0xDA, 0xF4, 0x05, 0xD9, 0xF2, 0x85, 0xA4, 0xC2, 0xF2,
   0xD8, 0xA8, 0x8D, 0x94, 0x01, 0xD1, 0xD9, 0xF4, 0x11, 0xF2, 0xD8, 0x87, 0x21, 0xD8, 0xF4, 0x0A,
   0xD8, 0xF2, 0x84, 0x98, 0xA8, 0xC8, 0x01, 0xD1, 0xD9, 0xF4, 0x11, 0xD8, 0xF3, 0xA4, 0xC8, 0xBB,
   0xAF, 0xD0, 0xF2, 0xDE, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xD8, 0xF1, 0xB8, 0xF6,
   0xB5, 0xB9, 0xB0, 0x8A, 0x95, 0xA3, 0xDE, 0x3C, 0xA3, 0xD9, 0xF8, 0xD8, 0x5C, 0xA3, 0xD9, 0xF8,
   0xD8, 0x7C, 0xA3, 0xD9, 0xF8, 0xD8, 0xF8, 0xF9, 0xD1, 0xA5, 0xD9, 0xDF, 0xDA, 0xFA, 0xD8, 0xB1,
   0x85, 0x30, 0xF7, 0xD9, 0xDE, 0xD8, 0xF8, 0x30, 0xAD, 0xDA, 0xDE, 0xD8, 0xF2, 0xB4, 0x8C, 0x99,
   0xA3, 0x2D, 0x55, 0x7D, 0xA0, 0x83, 0xDF, 0xDF, 0xDF, 0xB5, 0x91, 0xA0, 0xF6, 0x29, 0xD9, 0xFB,
   0xD8, 0xA0, 0xFC, 0x29, 0xD9, 0xFA, 0xD8, 0xA0, 0xD0, 0x51, 0xD9, 0xF8, 0xD8, 0xFC, 0x51, 0xD9,
   0xF9, 0xD8, 0x79, 0xD9, 0xFB, 0xD8, 0xA0, 0xD0, 0xFC, 0x79, 0xD9, 0xFA, 0xD8, 0xA1, 0xF9, 0xF9,
   0xF9, 0xF9, 0xF9, 0xA0, 0xDA, 0xDF, 0xDF, 0xDF, 0xD8, 0xA1, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xAC,
   0xDE, 0xF8, 0xAD, 0xDE, 0x83, 0x93, 0xAC, 0x2C, 0x54, 0x7C, 0xF1, 0xA8, 0xDF, 0xDF, 0xDF, 0xF6,
   0x9D, 0x2C, 0xDA, 0xA0, 0xDF, 0xD9, 0xFA, 0xDB, 0x2D, 0xF8, 0xD8, 0xA8, 0x50, 0xDA, 0xA0, 0xD0,
   0xDE, 0xD9, 0xD0, 0xF8, 0xF8, 0xF8, 0xDB, 0x55, 0xF8, 0xD8, 0xA8, 0x78, 0xDA, 0xA0, 0xD0, 0xDF,
   /* bank # 8 */
   0xD9, 0xD0, 0xFA, 0xF8, 0xF8, 0xF8, 0xF8, 0xDB, 0x7D, 0xF8, 0xD8, 0x9C, 0xA8, 0x8C, 0xF5, 0x30, // 2048
   0xDB, 0x38, 0xD9, 0xD0, 0xDE, 0xDF, 0xA0, 0xD0, 0xDE, 0xDF, 0xD8, 0xA8, 0x48, 0xDB, 0x58, 0xD9,
   0xDF, 0xD0, 0xDE, 0xA0, 0xDF, 0xD0, 0xDE, 0xD8, 0xA8, 0x68, 0xDB, 0x70, 0xD9, 0xDF, 0xDF, 0xA0,
   0xDF, 0xDF, 0xD8, 0xF1, 0xA8, 0x88, 0x90, 0x2C, 0x54, 0x7C, 0x98, 0xA8, 0xD0, 0x5C, 0x38, 0xD1,
   0xDA, 0xF2, 0xAE, 0x8C, 0xDF, 0xF9, 0xD8, 0xB0, 0x87, 0xA8, 0xC1, 0xC1, 0xB1, 0x88, 0xA8, 0xC6,
   0xF9, 0xF9, 0xDA, 0x36, 0xD8, 0xA8, 0xF9, 0xDA, 0x36, 0xD8, 0xA8, 0xF9, 0xDA, 0x36, 0xD8, 0xA8,
   0xF9, 0xDA, 0x36, 0xD8, 0xA8, 0xF9, 0xDA, 0x36, 0xD8, 0xF7, 0x8D, 0x9D, 0xAD, 0xF8, 0x18, 0xDA,
   0xF2, 0xAE, 0xDF, 0xD8, 0xF7, 0xAD, 0xFA, 0x30, 0xD9, 0xA4, 0xDE, 0xF9, 0xD8, 0xF2, 0xAE, 0xDE,
   0xFA, 0xF9, 0x83, 0xA7, 0xD9, 0xC3, 0xC5, 0xC7, 0xF1, 0x88, 0x9B, 0xA7, 0x7A, 0xAD, 0xF7, 0xDE,
   0xDF, 0xA4, 0xF8, 0x84, 0x94, 0x08, 0xA7, 0x97, 0xF3, 0x00, 0xAE, 0xF2, 0x98, 0x19, 0xA4, 0x88,
   0xC6, 0xA3, 0x94, 0x88, 0xF6, 0x32, 0xDF, 0xF2, 0x83, 0x93, 0xDB, 0x09, 0xD9, 0xF2, 0xAA, 0xDF,
   0xD8, 0xD8, 0xAE, 0xF8, 0xF9, 0xD1, 0xDA, 0xF3, 0xA4, 0xDE, 0xA7, 0xF1, 0x88, 0x9B, 0x7A, 0xD8,
   0xF3, 0x84, 0x94, 0xAE, 0x19, 0xF9, 0xDA, 0xAA, 0xF1, 0xDF, 0xD8, 0xA8, 0x81, 0xC0, 0xC3, 0xC5,
   0xC7, 0xA3, 0x92, 0x83, 0xF6, 0x28, 0xAD, 0xDE, 0xD9, 0xF8, 0xD8, 0xA3, 0x50, 0xAD, 0xD9, 0xF8,
   0xD8, 0xA3, 0x78, 0xAD, 0xD9, 0xF8, 0xD8, 0xF8, 0xF9, 0xD1, 0xA1, 0xDA, 0xDE, 0xC3, 0xC5, 0xC7,
   0xD8, 0xA1, 0x81, 0x94, 0xF8, 0x18, 0xF2, 0xB0, 0x89, 0xAC, 0xC3, 0xC5, 0xC7, 0xF1, 0xD8, 0xB8,
   /* bank # 9 */
   0xB4, 0xB0, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97, 0x28, 0x88, 0x9B, 0xF0, // 2304
   0x0C, 0x20, 0x14, 0x40, 0xB0, 0xB4, 0xB8, 0xF0, 0xA8, 0x8A, 0x9A, 0x28, 0x50, 0x78, 0xB7, 0x9B,
   0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xF1, 0xBB, 0xAB,
   0x88, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0xB3, 0x8B, 0xB8, 0xA8, 0x04, 0x28, 0x50, 0x78, 0xF1, 0xB0,
   0x88, 0xB4, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xBB, 0xAB, 0xB3, 0x8B, 0x02, 0x26, 0x46, 0x66, 0xB0,
   0xB8, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79, 0x8A, 0x24, 0x70, 0x59,
   0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68, 0x8A, 0x64, 0x48, 0x31,
   0x8B, 0x30, 0x49, 0x60, 0x88, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04, 0x28,
   0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66, 0xF0,
   0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xA9,
   0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60, 0x8C,
   0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76, 0x7E,
   0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0xD8, 0xB1, 0xB5, 0xB9, 0xA3, 0xDF, 0xDF, 0xDF, 0xAE, 0xD0,
   0xDF, 0xAA, 0xD0, 0xDE, 0xF2, 0xAB, 0xF8, 0xF9, 0xD9, 0xB0, 0x87, 0xC4, 0xAA, 0xF1, 0xDF, 0xDF,
   0xBB, 0xAF, 0xDF, 0xDF, 0xB9, 0xD8, 0xB1, 0xF1, 0xA3, 0x97, 0x8E, 0x60, 0xDF, 0xB0, 0x84, 0xF2,
   0xC8, 0xF8, 0xF9, 0xD9, 0xDE, 0xD8, 0x93, 0x85, 0xF1, 0x4A, 0xB1, 0x83, 0xA3, 0x08, 0xB5, 0x83,
   /* bank # 10 */
   0x9A, 0x08, 0x10, 0xB7, 0x9F, 0x10, 0xD8, 0xF1, 0xB0, 0xBA, 0xAE, 0xB0, 0x8A, 0xC2, 0xB2, 0xB6, // 2560
   0x8E, 0x9E, 0xF1, 0xFB, 0xD9, 0xF4, 0x1D, 0xD8, 0xF9, 0xD9, 0x0C, 0xF1, 0xD8, 0xF8, 0xF8, 0xAD, // 2576
   0x61, 0xD9, 0xAE, 0xFB, 0xD8, 0xF4, 0x0C, 0xF1, 0xD8, 0xF8, 0xF8, 0xAD, 0x19, 0xD9, 0xAE, 0xFB, // 2592
   0xDF, 0xD8, 0xF4, 0x16, 0xF1, 0xD8, 0xF8, 0xAD, 0x8D, 0x61, 0xD9, 0xF4, 0xF4, 0xAC, 0xF5, 0x9C, // 2608
   0x9C, 0x8D, 0xDF, 0x2B, 0xBA, 0xB6, 0xAE, 0xFA, 0xF8, 0xF4, 0x0B, 0xD8, 0xF1, 0xAE, 0xD0, 0xF8, // 2624
   0xAD, 0x51, 0xDA, 0xAE, 0xFA, 0xF8, 0xF1, 0xD8, 0xB9, 0xB1, 0xB6, 0xA3, 0x83, 0x9C, 0x08, 0xB9, // 2640
   0xB1, 0x83, 0x9A, 0xB5, 0xAA, 0xC0, 0xFD, 0x30, 0x83, 0xB7, 0x9F, 0x10, 0xB5, 0x8B, 0x93, 0xF2, // 2656
   0x02, 0x02, 0xD1, 0xAB, 0xDA, 0xDE, 0xD8, 0xF1, 0xB0, 0x80, 0xBA, 0xAB, 0xC0, 0xC3, 0xB2, 0x84, // 2672
   0xC1, 0xC3, 0xD8, 0xB1, 0xB9, 0xF3, 0x8B, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xB0, // 2688
   0x87, 0x9C, 0xB9, 0xA3, 0xDD, 0xF1, 0xB3, 0x8B, 0x8B, 0x8B, 0x8B, 0x8B, 0xB0, 0x87, 0x20, 0x28, // 2704
   0x30, 0x38, 0xB2, 0x8B, 0xB6, 0x9B, 0xF2, 0xA3, 0xC0, 0xC8, 0xC2, 0xC4, 0xCC, 0xC6, 0xA3, 0xA3, // 2720 This Has The Gyro Calibration enabled
   //0x30, 0x38, 0xB0, 0x80, 0xB4, 0x90, 0xF2, 0xA3, 0xC0, 0xC8, 0xC2, 0xC4, 0xCC, 0xC6, 0xA3, 0xA3, // 2720 This Has The Gyro Calibration Disabled
   0xA3, 0xF1, 0xB0, 0x87, 0xB5, 0x9A, 0x20, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC, 0xBA, 0xAC, 0xDF, 0xB9, // 2736  change 2742 from 0xD8 to 0x20 Including the DMP_FEATURE_TAP -- known issue in which if you do not enable DMP_FEATURE_TAP then the interrupts will be at 200Hz even if fifo rate
   0xA3, 0xFE, 0xF2, 0xAB, 0xC4, 0xAA, 0xF1, 0xDF, 0xDF, 0xBB, 0xAF, 0xDF, 0xDF, 0xA3, 0xA3, 0xA3, // 2752
   0xD8, 0xD8, 0xD8, 0xBB, 0xB3, 0xB7, 0xF1, 0xAA, 0xF9, 0xDA, 0xFF, 0xD9, 0x80, 0x9A, 0xAA, 0x28, // 2768
   0xB4, 0x80, 0x98, 0xA7, 0x20, 0xB7, 0x97, 0x87, 0xA8, 0x66, 0x88, 0xF0, 0x79, 0x51, 0xF1, 0x90, // 2784
   0x2C, 0x87, 0x0C, 0xA7, 0x81, 0x97, 0x62, 0x93, 0xF0, 0x71, 0x71, 0x60, 0x85, 0x94, 0x01, 0x29, // 2800
   /* bank # 11 */
   0x51, 0x79, 0x90, 0xA5, 0xF1, 0x28, 0x4C, 0x6C, 0x87, 0x0C, 0x95, 0x18, 0x85, 0x78, 0xA3, 0x83, // 2816
   0x90, 0x28, 0x4C, 0x6C, 0x88, 0x6C, 0xD8, 0xF3, 0xA2, 0x82, 0x00, 0xF2, 0x10, 0xA8, 0x92, 0x19,
   0x80, 0xA2, 0xF2, 0xD9, 0x26, 0xD8, 0xF1, 0x88, 0xA8, 0x4D, 0xD9, 0x48, 0xD8, 0x96, 0xA8, 0x39,
   0x80, 0xD9, 0x3C, 0xD8, 0x95, 0x80, 0xA8, 0x39, 0xA6, 0x86, 0x98, 0xD9, 0x2C, 0xDA, 0x87, 0xA7,
   0x2C, 0xD8, 0xA8, 0x89, 0x95, 0x19, 0xA9, 0x80, 0xD9, 0x38, 0xD8, 0xA8, 0x89, 0x39, 0xA9, 0x80,
   0xDA, 0x3C, 0xD8, 0xA8, 0x2E, 0xA8, 0x39, 0x90, 0xD9, 0x0C, 0xD8, 0xA8, 0x95, 0x31, 0x98, 0xD9,
   0x0C, 0xD8, 0xA8, 0x09, 0xD9, 0xFF, 0xD8, 0x01, 0xDA, 0xFF, 0xD8, 0x95, 0x39, 0xA9, 0xDA, 0x26,
   0xFF, 0xD8, 0x90, 0xA8, 0x0D, 0x89, 0x99, 0xA8, 0x10, 0x80, 0x98, 0x21, 0xDA, 0x2E, 0xD8, 0x89,
   0x99, 0xA8, 0x31, 0x80, 0xDA, 0x2E, 0xD8, 0xA8, 0x86, 0x96, 0x31, 0x80, 0xDA, 0x2E, 0xD8, 0xA8,
   0x87, 0x31, 0x80, 0xDA, 0x2E, 0xD8, 0xA8, 0x82, 0x92, 0xF3, 0x41, 0x80, 0xF1, 0xD9, 0x2E, 0xD8,
   0xA8, 0x82, 0xF3, 0x19, 0x80, 0xF1, 0xD9, 0x2E, 0xD8, 0x82, 0xAC, 0xF3, 0xC0, 0xA2, 0x80, 0x22,
   0xF1, 0xA6, 0x2E, 0xA7, 0x2E, 0xA9, 0x22, 0x98, 0xA8, 0x29, 0xDA, 0xAC, 0xDE, 0xFF, 0xD8, 0xA2,
   0xF2, 0x2A, 0xF1, 0xA9, 0x2E, 0x82, 0x92, 0xA8, 0xF2, 0x31, 0x80, 0xA6, 0x96, 0xF1, 0xD9, 0x00,
   0xAC, 0x8C, 0x9C, 0x0C, 0x30, 0xAC, 0xDE, 0xD0, 0xDE, 0xFF, 0xD8, 0x8C, 0x9C, 0xAC, 0xD0, 0x10,
   0xAC, 0xDE, 0x80, 0x92, 0xA2, 0xF2, 0x4C, 0x82, 0xA8, 0xF1, 0xCA, 0xF2, 0x35, 0xF1, 0x96, 0x88,
   0xA6, 0xD9, 0x00, 0xD8, 0xF1, 0xFF,
   };

   const uint32_t dmpFirmwareSize = sizeof(dmpFirmware) / sizeof(dmpFirmware[0]);
}