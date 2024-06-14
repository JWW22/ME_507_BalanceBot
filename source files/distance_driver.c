/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : distance_driver.c
  * @brief          : This driver is meant to be used with the Adafruit US100 ultra-sonic distance sensor.
  ******************************************************************************
  * @author Johnathan Waldmire, Peter Tomson
  * @date   June 3, 2024
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include <distance_driver.h>
#include <stdint.h>

/**
 * @brief  This function enables the US100 sensor.
 * @retval None
 * @param[in] Distance sensor Pointer
 */
void enable_distance(distance *p_dis){
	HAL_UART_Init(p_dis->uart);
}

/**
 * @brief  This function disables the US100 sensor.
 * @retval None
 * @param[in] Distance sensor Pointer
 */
void disable_distance(distance *p_dis){
	HAL_UART_DeInit(p_dis->uart);
}

/**
 * @brief  This function returns the distance value read from the sensor.
 * @retval The distance in millimeters
 * @param[in] Distance sensor Pointer
 */
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
		return 0; // return nothing if the read didn't work
	}
}
