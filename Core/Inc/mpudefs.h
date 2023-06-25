#pragma once

#define MPU9250_DMP_JUMP_ADDRESS 0x0400

#define MPU9250_SELF_TEST_X_GYRO 0x00
#define MPU9250_SELF_TEST_Y_GYRO 0x01
#define MPU9250_SELF_TEST_Z_GYRO 0x02
#define MPU9250_SELF_TEST_X_ACCEL 0x0D
#define MPU9250_SELF_TEST_Y_ACCEL 0x0E
#define MPU9250_SELF_TEST_Z_ACCEL 0x0F
#define MPU9250_XG_OFFSET_H 0x13
#define MPU9250_XG_OFFSET_L 0x14
#define MPU9250_YG_OFFSET_H 0x15
#define MPU9250_YG_OFFSET_L 0x16
#define MPU9250_ZG_OFFSET_H 0x17
#define MPU9250_ZG_OFFSET_L 0x18
#define MPU9250_SMPLRT_DIV 0x19
#define MPU9250_CONFIG 0x1A
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_ACCEL_CONFIG_2 0x1D
#define MPU9250_LP_ACCEL_ODR 0x1E
#define MPU9250_WOM_THR 0x1F
#define MPU9250_FIFO_EN 0x23
#define MPU9250_I2C_MST_CTRL 0x24
#define MPU9250_I2C_SLV0_ADDR 0x25
#define MPU9250_I2C_SLV0_REG 0x26
#define MPU9250_I2C_SLV0_CTRL 0x27
#define MPU9250_I2C_SLV1_ADDR 0x28
#define MPU9250_I2C_SLV1_REG 0x29
#define MPU9250_I2C_SLV1_CTRL 0x2A
#define MPU9250_I2C_SLV2_ADDR 0x2B
#define MPU9250_I2C_SLV2_REG 0x2C
#define MPU9250_I2C_SLV2_CTRL 0x2D
#define MPU9250_I2C_SLV3_ADDR 0x2E
#define MPU9250_I2C_SLV3_REG 0x2F
#define MPU9250_I2C_SLV3_CTRL 0x30
#define MPU9250_I2C_SLV4_ADDR 0x31
#define MPU9250_I2C_SLV4_REG 0x32
#define MPU9250_I2C_SLV4_DO 0x33
#define MPU9250_I2C_SLV4_CTRL 0x34
#define MPU9250_I2C_SLV4_DI 0x35
#define MPU9250_I2C_MST_STATUS 0x36
#define MPU9250_INT_PIN_CFG 0x37
#define MPU9250_INT_ENABLE 0x38
#define MPU9250_INT_STATUS 0x3A
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_ACCEL_XOUT_L 0x3C
#define MPU9250_ACCEL_YOUT_H 0x3D
#define MPU9250_ACCEL_YOUT_L 0x3E
#define MPU9250_ACCEL_ZOUT_H 0x3F
#define MPU9250_ACCEL_ZOUT_L 0x40
#define MPU9250_TEMP_OUT_H 0x41
#define MPU9250_TEMP_OUT_L 0x42
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_GYRO_XOUT_L 0x44
#define MPU9250_GYRO_YOUT_H 0x45
#define MPU9250_GYRO_YOUT_L 0x46
#define MPU9250_GYRO_ZOUT_H 0x47
#define MPU9250_GYRO_ZOUT_L 0x48
#define MPU9250_EXT_SENS_DATA_00 0x49
#define MPU9250_EXT_SENS_DATA_01 0x4A
#define MPU9250_EXT_SENS_DATA_02 0x4B
#define MPU9250_EXT_SENS_DATA_03 0x4C
#define MPU9250_EXT_SENS_DATA_04 0x4D
#define MPU9250_EXT_SENS_DATA_05 0x4E
#define MPU9250_EXT_SENS_DATA_06 0x4F
#define MPU9250_EXT_SENS_DATA_07 0x50
#define MPU9250_EXT_SENS_DATA_08 0x51
#define MPU9250_EXT_SENS_DATA_09 0x52
#define MPU9250_EXT_SENS_DATA_10 0x53
#define MPU9250_EXT_SENS_DATA_11 0x54
#define MPU9250_EXT_SENS_DATA_12 0x55
#define MPU9250_EXT_SENS_DATA_13 0x56
#define MPU9250_EXT_SENS_DATA_14 0x57
#define MPU9250_EXT_SENS_DATA_15 0x58
#define MPU9250_EXT_SENS_DATA_16 0x59
#define MPU9250_EXT_SENS_DATA_17 0x5A
#define MPU9250_EXT_SENS_DATA_18 0x5B
#define MPU9250_EXT_SENS_DATA_19 0x5C
#define MPU9250_EXT_SENS_DATA_20 0x5D
#define MPU9250_EXT_SENS_DATA_21 0x5E
#define MPU9250_EXT_SENS_DATA_22 0x5F
#define MPU9250_EXT_SENS_DATA_23 0x60
#define MPU9250_I2C_SLV0_DO 0x63
#define MPU9250_I2C_SLV1_DO 0x64
#define MPU9250_I2C_SLV2_DO 0x65
#define MPU9250_I2C_SLV3_DO 0x66
#define MPU9250_I2C_MST_DELAY_CTRL 0x67
#define MPU9250_SIGNAL_PATH_RESET 0x68
#define MPU9250_MOT_DETECT_CTRL 0x69
#define MPU9250_USER_CTRL 0x6A
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_PWR_MGMT_2 0x6C
#define MPU9250_DMP_CTRL_1 0x6D
#define MPU9250_DMP_CTRL_2 0x6E
#define MPU9250_DMP_CTRL_3 0x6F
#define MPU9250_FW_START_1 0x70
#define MPU9250_FW_START_2 0x71
#define MPU9250_FIFO_COUNTH 0x72
#define MPU9250_FIFO_COUNTL 0x73
#define MPU9250_FIFO_R_W 0x74
#define MPU9250_WHO_AM_I 0x75
#define MPU9250_XA_OFFSET_H 0x77
#define MPU9250_XA_OFFSET_L 0x78
#define MPU9250_YA_OFFSET_H 0x7A
#define MPU9250_YA_OFFSET_L 0x7B
#define MPU9250_ZA_OFFSET_H 0x7D
#define MPU9250_ZA_OFFSET_L 0x7E

#define AK8963_WIA 0x00
#define AK8963_INFO 0x01
#define AK8963_ST1 0x02
#define AK8963_HXL 0x03
#define AK8963_HXH 0x04
#define AK8963_HYL 0x05
#define AK8963_HYH 0x06
#define AK8963_HZL 0x07
#define AK8963_HZH 0x08
#define AK8963_ST2 0x09
#define AK8963_CNTL1 0x0A
#define AK8963_CNTL2 0x0B
#define AK8963_RSV 0x0B
#define AK8963_ASTC 0x0C
#define AK8963_TS1 0x0D
#define AK8963_TS2 0x0E
#define AK8963_I2CDIS 0x0F
#define AK8963_ASAX 0x10
#define AK8963_ASAY 0x11
#define AK8963_ASAZ 0x12

/* ---------------------------------------------*/

#define MPU9250_ACCE_SENSITIVITY_FACTOR_2G 16384
#define MPU9250_ACCE_SENSITIVITY_FACTOR_4G 8192
#define MPU9250_ACCE_SENSITIVITY_FACTOR_8G 4096
#define MPU9250_ACCE_SENSITIVITY_FACTOR_16G 2048

/* ----------------------------------------------*/

#define MPU9250_GYRO_SENSITIVITY_FACTOR_250s 131
#define MPU9250_GYRO_SENSITIVITY_FACTOR_500s 65.5
#define MPU9250_GYRO_SENSITIVITY_FACTOR_1000s 32.8
#define MPU9250_GYRO_SENSITIVITY_FACTOR_2000s 16.4


/* These defines are copied from dmpDefaultMPU6050.c in the general MPL
   releases. These defines may change for each DMP image, so be sure to modify
   these values when switching to a new image.
   Image 6.12
*/
#define CFG_LP_QUAT             (2712)
#define END_ORIENT_TEMP         (1866)
#define CFG_27                  (2742)
#define CFG_20                  (2224)
#define CFG_23                  (2745)
#define CFG_FIFO_ON_EVENT       (2690)
#define END_PREDICTION_UPDATE   (1761)
#define CGNOTICE_INTR           (2620)
#define X_GRT_Y_TMP             (1358)
#define CFG_DR_INT              (1029)
#define CFG_AUTH                (1035)
#define UPDATE_PROP_ROT         (1835)
#define END_COMPARE_Y_X_TMP2    (1455)
#define SKIP_X_GRT_Y_TMP        (1359)
#define SKIP_END_COMPARE        (1435)
#define FCFG_3                  (1088)
#define FCFG_2                  (1066)
#define FCFG_1                  (1062)
#define END_COMPARE_Y_X_TMP3    (1434)
#define FCFG_7                  (1073)
#define FCFG_6                  (1106)
#define FLAT_STATE_END          (1713)
#define SWING_END_4             (1616)
#define SWING_END_2             (1565)
#define SWING_END_3             (1587)
#define SWING_END_1             (1550)
#define CFG_8                   (2718)
#define CFG_15                  (2727)
#define CFG_16                  (2746)
#define CFG_EXT_GYRO_BIAS       (1189)
#define END_COMPARE_Y_X_TMP     (1407)
#define DO_NOT_UPDATE_PROP_ROT  (1839)
#define CFG_7                   (1205)
#define FLAT_STATE_END_TEMP     (1683)
#define END_COMPARE_Y_X         (1484)
#define SKIP_SWING_END_1        (1551)
#define SKIP_SWING_END_3        (1588)
#define SKIP_SWING_END_2        (1566)
#define TILTG75_START           (1672)
#define CFG_6                   (2753)
#define TILTL75_END             (1669)
#define END_ORIENT              (1884)
#define CFG_FLICK_IN            (2573)
#define TILTL75_START           (1643)
#define CFG_MOTION_BIAS         (1208)
#define X_GRT_Y                 (1408)
#define TEMPLABEL               (2324)
#define CFG_ANDROID_ORIENT_INT  (1853)
#define CFG_GYRO_RAW_DATA       (2722)
#define X_GRT_Y_TMP2            (1379)

#define D_0_22                  (22+512) //534
#define D_0_24                  (24+512) //536

#define D_0_36                  (36)
#define D_0_52                  (52)
#define D_0_96                  (96)
#define D_0_104                 (104)
#define D_0_108                 (108)
#define D_0_163                 (163)
#define D_0_188                 (188)
#define D_0_192                 (192)
#define D_0_224                 (224)
#define D_0_228                 (228)
#define D_0_232                 (232)
#define D_0_236                 (236)

#define D_1_2                   (256 + 2) //258
#define D_1_4                   (256 + 4) //260
#define D_1_8                   (256 + 8) //264
#define D_1_10                  (256 + 10) //266
#define D_1_24                  (256 + 24) //280
#define D_1_28                  (256 + 28) //284
#define D_1_36                  (256 + 36) //292
#define D_1_40                  (256 + 40) //296
#define D_1_44                  (256 + 44) //300
#define D_1_72                  (256 + 72) //328
#define D_1_74                  (256 + 74) //330
#define D_1_79                  (256 + 79) //335
#define D_1_88                  (256 + 88) //344
#define D_1_90                  (256 + 90) //346
#define D_1_92                  (256 + 92) //348
#define D_1_96                  (256 + 96) //352
#define D_1_98                  (256 + 98) //354
#define D_1_106                 (256 + 106) //362
#define D_1_108                 (256 + 108) //364
#define D_1_112                 (256 + 112) //368
#define D_1_128                 (256 + 144) //400
#define D_1_152                 (256 + 12) //268
#define D_1_160                 (256 + 160) //416
#define D_1_176                 (256 + 176) //432
#define D_1_178                 (256 + 178) //434
#define D_1_218                 (256 + 218) //474
#define D_1_232                 (256 + 232) //488
#define D_1_236                 (256 + 236) //492
#define D_1_240                 (256 + 240) //496
#define D_1_244                 (256 + 244) //500
#define D_1_250                 (256 + 250) //506
#define D_1_252                 (256 + 252) //508
#define D_2_12                  (512 + 12) //524
#define D_2_96                  (512 + 96) //608
#define D_2_108                 (512 + 108) //620
#define D_2_208                 (512 + 208) //720
#define D_2_224                 (512 + 224) //736
#define D_2_236                 (512 + 236) //748
#define D_2_244                 (512 + 244) //756
#define D_2_248                 (512 + 248) //760
#define D_2_252                 (512 + 252) //764

#define CPASS_BIAS_X            (35 * 16 + 4) //564
#define CPASS_BIAS_Y            (35 * 16 + 8) //568
#define CPASS_BIAS_Z            (35 * 16 + 12) //572
#define CPASS_MTX_00            (36 * 16) //576
#define CPASS_MTX_01            (36 * 16 + 4) //580
#define CPASS_MTX_02            (36 * 16 + 8) //584
#define CPASS_MTX_10            (36 * 16 + 12) //588
#define CPASS_MTX_11            (37 * 16) //592
#define CPASS_MTX_12            (37 * 16 + 4) //596
#define CPASS_MTX_20            (37 * 16 + 8) //600
#define CPASS_MTX_21            (37 * 16 + 12) //604
#define CPASS_MTX_22            (43 * 16 + 12) //700
#define D_EXT_GYRO_BIAS_X       (61 * 16) //976
#define D_EXT_GYRO_BIAS_Y       (61 * 16) + 4 //980
#define D_EXT_GYRO_BIAS_Z       (61 * 16) + 8 //984
#define D_ACT0                  (40 * 16) //640
#define D_ACSX                  (40 * 16 + 4) //644
#define D_ACSY                  (40 * 16 + 8) //648
#define D_ACSZ                  (40 * 16 + 12) //652

#define FLICK_MSG               (45 * 16 + 4) //724
#define FLICK_COUNTER           (45 * 16 + 8) //728
#define FLICK_LOWER             (45 * 16 + 12) //732
#define FLICK_UPPER             (46 * 16 + 12) //748

#define D_AUTH_OUT              (992)
#define D_AUTH_IN               (996)
#define D_AUTH_A                (1000)
#define D_AUTH_B                (1004)

#define D_PEDSTD_BP_B           (768 + 0x1C) //796
#define D_PEDSTD_HP_A           (768 + 0x78) //888
#define D_PEDSTD_HP_B           (768 + 0x7C) //892
#define D_PEDSTD_BP_A4          (768 + 0x40) //832
#define D_PEDSTD_BP_A3          (768 + 0x44) //836
#define D_PEDSTD_BP_A2          (768 + 0x48) //840
#define D_PEDSTD_BP_A1          (768 + 0x4C) //844
#define D_PEDSTD_INT_THRSH      (768 + 0x68) //872
#define D_PEDSTD_CLIP           (768 + 0x6C) //876
#define D_PEDSTD_SB             (768 + 0x28) //808
#define D_PEDSTD_SB_TIME        (768 + 0x2C) //812
#define D_PEDSTD_PEAKTHRSH      (768 + 0x98) //920
#define D_PEDSTD_TIML           (768 + 0x2A) //810
#define D_PEDSTD_TIMH           (768 + 0x2E) //814
#define D_PEDSTD_PEAK           (768 + 0X94) //916
#define D_PEDSTD_STEPCTR        (768 + 0x60) //864
#define D_PEDSTD_TIMECTR        (964) // (768 + 0xC4)
#define D_PEDSTD_DECI           (768 + 0xA0) //928

#define D_HOST_NO_MOT           (976)
#define D_ACCEL_BIAS            (660)

#define D_ORIENT_GAP            (76)

#define D_TILT0_H               (48)
#define D_TILT0_L               (50)
#define D_TILT1_H               (52)
#define D_TILT1_L               (54)
#define D_TILT2_H               (56)
#define D_TILT2_L               (58)
#define D_TILT3_H               (60)
#define D_TILT3_L               (62)



#define DMP_CODE_SIZE           (3062)
#define DMP_START_ADDRESS       (0x0400) //1024


#define TAP_X               (0x01)
#define TAP_Y               (0x02)
#define TAP_Z               (0x04)
#define TAP_XYZ             (0x07)

#define TAP_X_UP            (0x01)
#define TAP_X_DOWN          (0x02)
#define TAP_Y_UP            (0x03)
#define TAP_Y_DOWN          (0x04)
#define TAP_Z_UP            (0x05)
#define TAP_Z_DOWN          (0x06)

#define ANDROID_ORIENT_PORTRAIT             (0x00)
#define ANDROID_ORIENT_LANDSCAPE            (0x01)
#define ANDROID_ORIENT_REVERSE_PORTRAIT     (0x02)
#define ANDROID_ORIENT_REVERSE_LANDSCAPE    (0x03)

#define DMP_INT_GESTURE     (0x01)
#define DMP_INT_CONTINUOUS  (0x02)

#define DMP_FEATURE_TAP             (0x001)
#define DMP_FEATURE_ANDROID_ORIENT  (0x002)
#define DMP_FEATURE_LP_QUAT         (0x004)
#define DMP_FEATURE_PEDOMETER       (0x008)
#define DMP_FEATURE_6X_LP_QUAT      (0x010)
#define DMP_FEATURE_GYRO_CAL        (0x020)
#define DMP_FEATURE_SEND_RAW_ACCEL  (0x040)
#define DMP_FEATURE_SEND_RAW_GYRO   (0x080)
#define DMP_FEATURE_SEND_CAL_GYRO   (0x100)
#define DMP_FEATURE_SEND_ANY_GYRO   (DMP_FEATURE_SEND_RAW_GYRO | DMP_FEATURE_SEND_CAL_GYRO)

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)


#define INV_WXYZ_QUAT       (0x100)
#define INV_FSR_2G  0
#define INV_FSR_4G 1
#define INV_FSR_8G 2
#define INV_FSR_16G 3



#define INT_SRC_TAP             (0x01)
#define INT_SRC_ANDROID_ORIENT  (0x08)

#define MAX_PACKET_LENGTH   (32)

#define DEFAULT_MPU_HZ    (100)
