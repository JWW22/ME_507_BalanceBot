/*
 * distance_driver.c
 *
 * This driver is meant to be used with the Adafruit US100 ultrasonic distance sensor
 *
 *  Created on: Jun 3, 2024
 *      Author: jwald
 */
#include <distance_driver.h>
#include <stdint.h>

//function to enable the ultrasonic distance sensor
void enable_distance(distance *p_dis){
	HAL_UART_Init(p_dis->uart);
}

//function to disable the ultrasonic distance sensor
void disable_distance(distance *p_dis){
	HAL_UART_DeInit(p_dis->uart);
}

// function that returns the distance value read from the sensor
uint16_t read_distance(distance *p_dis){

	HAL_UART_Transmit(p_dis->uart, &p_dis->command, 1,1000);

	//if statement verifies that the mcu received the sensor data correctly
	if (HAL_UART_Receive(p_dis->uart, p_dis->buffer, 2, 1000) == HAL_OK) {

		// Combine the two bytes to get the distance in millimeters
		p_dis->error = 0;
		return p_dis->distance = (p_dis->buffer[0] << 8) | p_dis->buffer[1];
	// if the sensor data was not read correctly, raise the error flag for the while loop to process
	} else {
		p_dis->error = 1;
		return 0; // return nothing if the read didnt work
	}
}
