/*
 * Sensor_Ultrasonico.h
 *
 *  Created on: Nov 28, 2023
 *      Author: Sergio
 */

#ifndef INC_SENSOR_ULTRASONICO_H_
#define INC_SENSOR_ULTRASONICO_H_

#include <stdint.h>             // Includes the standard integer types (e.g., uint32_t, uint16_t)
#include <stm32l4xx_hal.h>      // Includes the STM32 HAL library for STM32L4xx series




// Initializes the ultrasonic sensor parameters.
// Configures the sensor (GPIO pins, timer, etc.).
void Sensor_Ultrasonico_Init(void);

// Retrieves the distance measured by the ultrasonic sensor.
// Uses GPIO pins for triggering and capturing echo signals.
// Requires a timer handle for accurate time measurements.
uint16_t Sensor_Ultrasonico_GetDistance(TIM_HandleTypeDef *htim1);




#endif /* INC_SENSOR_ULTRASONICO_H_ */
