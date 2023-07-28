#pragma once

#define DISABLE_CALIBRATION 1
#define DISABLE_MAGNETOMETER 1
#define MAGNETOMETER_BLOCKING_MODE 0
#define FIXED_AHRS_UPDATE_RATE 1

#if FIXED_AHRS_UPDATE_RATE
#define AHRS_UPDATE_RATE 100 // Hz
#endif