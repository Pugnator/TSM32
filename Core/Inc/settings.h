#pragma once

/* Blink cadence: ramp (96/step * PWM_DUTY_DELAY) + TURN_OFF_DELAY +
 * TURN_OFF_PAUSE per cycle. Regular ~250+200+250 = 700 ms ~ 86 fpm,
 * hazard ~210+200+250 = 660 ms ~ 91 fpm - inside the SAE J590b
 * 60-120 fpm band (Fixes #68). */
#define PWM_ON_DUTY_STEP 4       // Increase to make a bulb turn on faster
#define PWM_HAZARD_DUTY_STEP 5   // Increase to make a bulb turn on faster in hazard mode
#define PWM_DUTY_DELAY 10        // Decrease to make a bulb turn on faster
#define TURN_OFF_DELAY 200       // Time for which a bulb should remain on
#define TURN_OFF_PAUSE 250       // Time for which a bulb should remain off after it was on
#define MAX_PRESS_WAIT_TIME 2000 // Maximum amount of time a timer can run
#define DEBOUNCE_MIN_TIME 100    // Minimum amount of time for which a button must be pressed to be considered a valid input
#define LONG_PRESS_TIME 1000     // Duration after which a button press will be considered a "long press"
#define OVERTAKE_BLINK_COUNT 5   // Number of blinks before automatic turn off
#define POST_TURN_BLINK_COUNT OVERTAKE_BLINK_COUNT // Full flashes after a detected turn
#define AZIMUTH_AVERAGE_COUNT 2  // Parameter for a Kalman filter
#define DLR_BRIGHTNESS_VALUE 10
#define VOLTAGE_DETECTION_THRESHOLD (15 * 1000)
/* Low-voltage debounce: avoid disabling DRL on short transients
 * (idle + stop lights, cranking). Only react to a sustained drop. */
#define LOW_VOLTAGE_DETECTION_THRESHOLD (5 * 60 * 1000)
/* Handler-level smoothing after each 128-conversion DMA burst. At the 500 ms
 * handler cadence this adds at most about four seconds of step latency. */
#define VOLTAGE_FILTER_WINDOW_SIZE 8u

#define BLINKER_TIMER htim4
#define BLINKER_TIMER_INSTANCE TIM4
#define J1850_EOF_TIMER htim3
#define J1850_EOF_TIMER_INSTANCE TIM3
#define J1850_IC_INSTANCE htim2
#define J1850_IC_TIMER_INSTANCE TIM2


#ifndef BLINKER_ENABLED
#define BLINKER_ENABLED 1
#endif
#ifndef J1850_ENABLED
#define J1850_ENABLED 1
#endif
#ifndef MEMS_ENABLED
#define MEMS_ENABLED 1
#endif

/* Set to 1 to log every J1850 frame on the bus (all sources/destinations).
 * Use this to capture real motorcycle traffic for analysis.
 * Disabled by default — produces heavy RTT output at idle. */
#define J1850_BUS_TRACE 1

/* IMU transport. The Makefile always passes -DIMU_USE_SPI/-DIMU_USE_I2C;
 * these fallbacks only matter for header-only consumers (IDE indexers,
 * host tools) and match the board default (SPI - MPU-9250 on SPI1). */
#ifndef IMU_USE_SPI
#define IMU_USE_SPI 1
#endif
#ifndef IMU_USE_I2C
#define IMU_USE_I2C 0
#endif
#if MEMS_ENABLED && (IMU_USE_SPI + IMU_USE_I2C) != 1
#error "Define exactly one of IMU_USE_SPI / IMU_USE_I2C"
#endif

#define STARTER_LOCK_ENABLE 1
#define STARTER_DISABLE_THRESHOLD (5 * 60 * 1000)
#define STARTER_UNLOCK_DISABLE 1

/* Immobilizer master switch. DISABLED for the v1.0 release: with the PIN gate
 * compiled out, the security module can never hold the starter relay off, so a
 * rider cannot be locked out by it. When 0, securityInit()/securityHandler()
 * are never called (the module's safe defaults leave the starter permitted),
 * PIN entry and the settings menu are inert, and the SIL-flash experiment is
 * off. Set to 1 to re-enable the full immobilizer. The watchdog reset-cause
 * fail-lock is independent of this switch and stays active. */
#define SECURITY_PIN_ENABLED 0

/* Security PIN (starter immobilizer, Core/Src/security.cc).
 * Entry: LEFT presses = digit value (1-9), RIGHT press commits the digit;
 * both buttons held >= 1 s toggles the hazard lights even while locked.
 * Hold both buttons during power-on to enter the settings menu
 * (menu item 1 = set/change PIN; committing an empty first digit clears it). */
#define SECURITY_PIN_LENGTH 4         /* digits, each 1-9                       */
#define SECURITY_ENTRY_TIMEOUT_MS 15000u /* inactivity aborts a partial entry   */
#define SECURITY_MAX_ATTEMPTS 5       /* wrong PINs before lockout              */
#define SECURITY_LOCKOUT_MS 30000u    /* lockout duration after max attempts    */
#define SECURITY_DEBOUNCE_MS 30u      /* button level debounce for PIN entry    */
#define SECURITY_CHORD_HAZARD_MS 1000u /* both-held time that toggles hazard    */

/* EXPERIMENTAL - flash the instrument-cluster security lamp (SIL) while the
 * immobilizer is locked and waiting for the PIN, by broadcasting an 0x89 SIL
 * frame toggled ~1 Hz. Unconfirmed on real hardware: the SIL is normally the
 * IPC's own broadcast (source 0x61), so the cluster may ignore a TSM-sourced
 * (0x40) frame or override it with its own "off". Bench-verify: watch the RTT
 * "SIL flash ->" lines against the lamp; if 0x40 is ignored, set
 * SECURITY_SIL_SRC to 0x61 to spoof the IPC's own source. header 0xC8 = pri 6,
 * matching the observed OEM key-lamp frames; CRC is appended by the TX path. */
#define SECURITY_FLASH_SIL 1
#define SECURITY_SIL_FLASH_MS 500u    /* half-period -> ~1 Hz flash             */
#define SECURITY_SIL_HDR  0xC8        /* priority-6 header                      */
#define SECURITY_SIL_SRC  0x40        /* our TSM address; try 0x61 if ignored   */

/* J1850-based engine-state starter lock (issue #54) */
#define ENGINE_RUNNING_RPM_MIN     700u  /* Below observed warm-idle floor (~729)   */
#define ENGINE_RUNNING_KPH_MIN    10u    /* KPH threshold to confirm bike is moving */
#define ENGINE_OFF_DEBOUNCE_MS    5000u  /* ms of RPM=0+KPH=0 before Off confirmed  */
#define J1850_SIGNAL_TIMEOUT_MS   10000u /* ms without valid RPM or KPH → voltage fallback */
#define AUTO_LIGHT_ENABLE 1

#define IMU_STARTUP_TIME (30 * 1000)

#define ADC_DMA_BUF_SIZE 128
/* Calibrated from bench measurement: raw=2882 @ 11.90V actual.
 * ADC_VCAL corrects for resistor tolerance in the voltage divider. */
#define ADC_VCAL 1.0893f
#define ADC_10V_VALUE   2422
#define ADC_11_1V_VALUE 2688
#define ADC_13_4V_VALUE 3245
#define ADC_14_3V_VALUE 3463

#define TURN_ANGLE_THRESHOLD 60
#define TURN_MAX_TIME_MS (5 * 60 * 1000)

void startupSettingsHandler();

/*
TIM2 J1850 input capture - 1us per tick (prescaler 63 @ 64 MHz)
TIM3 J1850 EOF one-shot  - 248us (1us tick, period 247)
TIM4 blinker             - 110ms per tick
*/

/*
Fuse8 for STM setup:
Buttons: up 0 0 dwn
port B : 0 0 up up 0 0 0 0 0
port A 00000000 down
leds 

*/
