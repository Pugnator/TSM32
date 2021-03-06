#pragma once

#define PWM_ON_DUTY_STEP 2
#define PWM_HAZARD_DUTY_STEP 5
#define PWM_DUTY_DELAY 5
#define TURN_OFF_DELAY 200
#define OVERTAKE_BLINK_COUNT 5

#define J1850_ENABLED 1
#define MEMS_ENABLED 0
#define STARTER_LOCK_ON 1


#define ADC_10V_VALUE 2865
#define ADC_13V_VALUE 3722
#define ADC_14_3V_VALUE 4095

#ifdef __cplusplus
extern "C" {
#endif
extern uint8_t SIDEMARK_BRIGHTNESS;
#ifdef __cplusplus
}
#endif