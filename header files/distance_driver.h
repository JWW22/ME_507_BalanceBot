/*
 * distance_driver.h
 *
 * This driver is meant to be used with the Adafruit US100 ultrasonic distance sensor
 *
 *  Created on: Jun 3, 2024
 *      Author: jwald
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct {
	UART_HandleTypeDef *uart; //uart channel we are using the sensor with
	uint8_t buffer[2]; //buffer to read the distance from
	uint8_t command; //command byte sent to the sensor to receive distance
	uint16_t distance; //final distance value in mm
	uint8_t error; //error flag
}distance;

void enable_distance(distance *p_dis);

void disable_distance(distance *p_dis);

uint16_t read_distance(distance *p_dis);

#endif /* INC_MOTOR_H_ */
