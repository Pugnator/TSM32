#pragma once
// Shadow: prevents real gpio.h from re-including main.h and declaring
// MX_GPIO_Init / HAL_GPIO_* prototypes (already covered by stubs.h).
