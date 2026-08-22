#pragma once

#define DISABLE_CALIBRATION 0
#define DISABLE_MAGNETOMETER 1
#define MAGNETOMETER_BLOCKING_MODE 1
#define FIXED_AHRS_UPDATE_RATE 1
#define SPEED_MATH 0

#if FIXED_AHRS_UPDATE_RATE
#define AHRS_UPDATE_RATE 100 // Hz
#endif

// MPU-9250 accelerometer DLPF (ACCEL_CONFIG_2, A_DLPF_CFG bits [2:0]).
// At a 1 kHz internal sample rate:
//   0x03 -> 41.0 Hz  (default, lets engine vibration through the passband)
//   0x04 -> 21.2 Hz
//   0x05 -> 10.2 Hz  (chosen for motorcycle: keeps the 12-67 Hz engine and
//                    20-80 Hz frame-resonance band out of the orientation
//                    integrator -- second-scale turn decisions don't need
//                    bandwidth higher than this)
#define ACCEL_DLPF_CFG 0x05

// Zero-velocity update (ZUPT) -- online gyro-bias refinement while the bike
// is stationary.  Compensates for thermal drift (~0.005 deg/s/degC on the
// MPU-9250 gyro -> ~0.3 deg/s across a 60 degC ride) and for boot-time
// vibration that contaminates the one-shot hardware bias calibration.
#define ENABLE_ZUPT 1
#define ZUPT_GYRO_THRESH_DPS 0.5f // max per-axis |gyro| to count as still
#define ZUPT_ACCEL_THRESH_G  0.03f // max ||accel|-1g| to count as still
#define ZUPT_HOLD_SAMPLES    50u  // require ~0.5 s of stillness at 100 Hz
#define ZUPT_BIAS_ALPHA      0.01f // EMA gain per stationary sample