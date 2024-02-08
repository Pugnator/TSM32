#pragma once

#define PWM_ON_DUTY_STEP 2       // Increase to make a bulb turn on faster
#define PWM_HAZARD_DUTY_STEP 5   // Increase to make a bulb turn on faster in hazard mode
#define PWM_DUTY_DELAY 10        // Decrease to make a bulb turn on faster
#define TURN_OFF_DELAY 200       // Time for which a bulb should remain on
#define TURN_OFF_PAUSE 400       // Time for which a bulb should remain off after it was on
#define MAX_PRESS_WAIT_TIME 2000 // Maximum amount of time a timer can run
#define DEBOUNCE_MIN_TIME 100    // Minimum amount of time for which a button must be pressed to be considered a valid input
#define LONG_PRESS_TIME 1000     // Duration after which a button press will be considered a "long press"
#define OVERTAKE_BLINK_COUNT 5   // Number of blinks before automatic turn off
#define AZIMUTH_AVERAGE_COUNT 2  // Parameter for a Kalman filter
#define DLR_BRIGHTNESS_VALUE 10
#define VOLTAGE_DETECTION_THRESHOLD 15 * 1000

#define USE_STATIC_ALLOC

#define BLINKER_TIMER htim4
#define BLINKER_TIMER_INSTANCE TIM4
#define J1850_EOF_TIMER htim3
#define J1850_EOF_TIMER_INSTANCE TIM3
#define J1850_IC_INSTANCE htim2
#define J1850_IC_TIMER_INSTANCE TIM2


#define BLINKER_ENABLED 1
#define J1850_ENABLED 0
#define MEMS_ENABLED 1
#define STARTER_LOCK_ENABLE 1
#define STARTER_DISABLE_THRESHOLD (5 * 60 * 1000)
#define STARTER_UNLOCK_DISABLE 1
#define AUTO_LIGHT_ENABLE 1

#define IMU_STARTUP_TIME (30 * 1000)

#define ADC_DMA_BUF_SIZE 128
#define ADC_10V_VALUE 2865
#define ADC_12_2V_VALUE 2919
#define ADC_12_8V_VALUE 3522
#define ADC_14_3V_VALUE 4095

#define TURN_ANGLE_THRESHOLD 60
#define TURN_MAX_TIME_MS 5 * 60 * 1000

void startupSettingsHandler();

/*
TIM5/2 J1850 - 1us per tick
TIM6/3 - 240us/0.28ms
TIM9/4 - 110ms/700ms
TIM11- 333ms
*/

/*
Fuse8 for STM setup:
Buttons: up 0 0 dwn
port B : 0 0 up up 0 0 0 0 0
port A 00000000 down
leds 

*/