#pragma once

/** \brief increase to make a bulb on faster */
#define PWM_ON_DUTY_STEP 2
/** \brief  increase to make a bulb on faster in hazard mode */
#define PWM_HAZARD_DUTY_STEP 5
/** \brief increase to make a bulb on faster */
#define PWM_DUTY_DELAY 10
/** \brief time for a bulb to be on */
#define TURN_OFF_DELAY 200
/** \brief de-bounce value */
#define MIN_PRESS_TIME 250
/** \brief number of blinks before auto-off */
#define OVERTAKE_BLINK_COUNT 5
/** \brief Parameter for a kalman filter */
#define AZIMUTH_AVERAGE_COUNT 2

#define BLINKER_ENABLED 1
#define J1850_ENABLED 0
#define MEMS_ENABLED 0
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