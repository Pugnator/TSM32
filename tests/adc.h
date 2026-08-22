#pragma once
// Shadow: prevents real adc.h from pulling in ADC_HandleTypeDef / hadc1.
// Neither switch_ctrl.cc nor turn_ctrl.cc use the ADC handle directly.
