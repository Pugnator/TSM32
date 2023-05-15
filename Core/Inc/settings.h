#pragma once

#define PWM_ON_DUTY_STEP 2 // Increase to make a bulb turn on faster
#define PWM_HAZARD_DUTY_STEP 5 // Increase to make a bulb turn on faster in hazard mode
#define PWM_DUTY_DELAY 10 // Decrease to make a bulb turn on faster
#define TURN_OFF_DELAY 200 // Time for which a bulb should remain on
#define MAX_PRESS_WAIT_TIME 2000 // Maximum amount of time a timer can run
#define DEBOUNCE_MIN_TIME 100 // Minimum amount of time for which a button must be pressed to be considered a valid input
#define LONG_PRESS_TIME 1000 // Duration after which a button press will be considered a "long press"
#define OVERTAKE_BLINK_COUNT 5 // Number of blinks before automatic turn off
#define AZIMUTH_AVERAGE_COUNT 2 // Parameter for a Kalman filter

#define BLINKER_ENABLED 0
#define J1850_ENABLED 0
#define MEMS_ENABLED 1
#define STARTER_LOCK_ON 1

#define ADC_10V_VALUE 2865
#define ADC_13V_VALUE 3722
#define ADC_14_3V_VALUE 4095
#define ADC_STABLE_COUNT 1000

#ifdef __cplusplus
extern "C"
{
#endif
  extern uint8_t SIDEMARK_BRIGHTNESS;
#ifdef __cplusplus
}
#endif