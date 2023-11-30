/*
 * Sensor_Ultrasonico.c
 *
 *  Created on: Nov 28, 2023
 *      Author: Sergio
 */

#include <Sensor_Ultrasonico.h>
#include <main.h>

uint32_t pMillis;
uint32_t Value1;
uint32_t Value2;
uint16_t Distance;  // cm
// Initializes the ultrasonic sensor parameters.
// Configures the sensor (GPIO pins, timer, etc.).
void Sensor_Ultrasonico_Init(void) {
	uint32_t pMillis =0;
	uint32_t Value1 = 0;
	uint32_t Value2 = 0;
	uint16_t Distance  = 0;  // cm
}

// Retrieves the distance measured by the ultrasonic sensor.
// Uses GPIO pins for triggering and capturing echo signals.
// Requires a timer handle for accurate time measurements.
uint16_t Sensor_Ultrasonico_GetDistance(TIM_HandleTypeDef *htim1) {

	Sensor_Ultrasonico_Init();

    // Pulls the TRIG pin HIGH to trigger the ultrasonic pulse
    HAL_GPIO_WritePin(GPIOA, TRIG_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(htim1, 0);
    while (__HAL_TIM_GET_COUNTER(htim1) < 10);  // Waits for 10 microseconds

    // Pulls the TRIG pin LOW after triggering
    HAL_GPIO_WritePin(GPIOA, TRIG_Pin, GPIO_PIN_RESET);

    // Measures the time for the echo to return and calculates distance
    pMillis = HAL_GetTick(); // Records start time for timeout
    // Waits for the echo pin to go HIGH (start of echo signal)
    while (!(HAL_GPIO_ReadPin(GPIOA, ECHO_Pin)) && pMillis + 10 > HAL_GetTick());
    Value1 = __HAL_TIM_GET_COUNTER(htim1);

    pMillis = HAL_GetTick(); // Records start time for timeout
    // Waits for the echo pin to go LOW (end of echo signal)
    while ((HAL_GPIO_ReadPin(GPIOA, ECHO_Pin)) && pMillis + 50 > HAL_GetTick());
    Value2 = __HAL_TIM_GET_COUNTER(htim1);

    // Calculates the distance based on the time difference between echo signals
    Distance = (Value2 - Value1) * 0.034 / 2;
    HAL_Delay(250); // Delay before next measurement

    return Distance;
}


