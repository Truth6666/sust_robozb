#include <stdint.h>
#include <math.h>

// Compatibility shim for legacy BSP init signature used by main.c
extern void BSP_Init(uint32_t Status);

void BSP_Init(unsigned int Status, float IMU_Heater_Rate, float Buzzer_Rate)
{
    (void)IMU_Heater_Rate;
    (void)Buzzer_Rate;
    BSP_Init((uint32_t)Status);
}

// Compatibility shim for CAN init guard flag expected by drv_can.cpp
volatile uint8_t init_finished = 1;

// CMSIS-DSP fallback used by drv_math.cpp when DSP library object is not linked
extern "C" float arm_sin_f32(float x)
{
    return sinf(x);
}
