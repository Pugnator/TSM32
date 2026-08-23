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
#define AZIMUTH_AVERAGE_COUNT 2  // Parameter for a Kalman filter
#define DLR_BRIGHTNESS_VALUE 10
#define VOLTAGE_DETECTION_THRESHOLD (15 * 1000)
/* Low-voltage debounce: avoid disabling DRL on short transients
 * (idle + stop lights, cranking). Only react to a sustained drop. */
#define LOW_VOLTAGE_DETECTION_THRESHOLD (5 * 60 * 1000)

#define USE_STATIC_ALLOC

#define BLINKER_TIMER htim4
#define BLINKER_TIMER_INSTANCE TIM4
#define J1850_EOF_TIMER htim3
#define J1850_EOF_TIMER_INSTANCE TIM3
#define J1850_IC_INSTANCE htim2
#define J1850_IC_TIMER_INSTANCE TIM2


#define BLINKER_ENABLED 1
#define J1850_ENABLED 1
#define MEMS_ENABLED 1

/* Set to 1 to log every J1850 frame on the bus (all sources/destinations).
 * Use this to capture real motorcycle traffic for analysis.
 * Disabled by default — produces heavy RTT output at idle. */
#define J1850_BUS_TRACE 1

/* IMU transport. The Makefile always passes -DIMU_USE_SPI/-DIMU_USE_I2C;
 * these fallbacks only matter for header-only consumers (IDE indexers,
 * host tools) and match the board default (SPI since the CubeMX project
 * dropped the I2C peripheral). */
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

/* J1850-based engine-state starter lock (issue #54) */
#define ENGINE_RUNNING_RPM_MIN    1000u  /* RPM threshold to consider engine on     */
#define ENGINE_RUNNING_KPH_MIN    10u    /* KPH threshold to confirm bike is moving */
#define ENGINE_OFF_DEBOUNCE_MS    5000u  /* ms of RPM=0+KPH=0 before Off confirmed  */
#define J1850_BUS_TIMEOUT_MS      10000u /* ms of bus silence → fall back to voltage FSM */
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