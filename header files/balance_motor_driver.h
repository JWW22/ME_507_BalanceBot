/*
 * motor_driver.h
 *
 *  Created on: Apr 25, 2024
 *      Author: jwald
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"

struct {
	TIM_HandleTypeDef *tim;		// Pointer to pwm timer
	GPIO_TypeDef *GPIO_Port;    // Pointer to GPIO port
	uint16_t     GPIO_Pin_Pos; // GPIO positive pin number
	uint16_t     GPIO_Pin_Neg; // GPIO negative pin number
	uint32_t 	 *pwm;          // pwm timer channel
}typedef motor;

void enable_motor(motor *p_mot);

void disable_motor(motor *p_mot);

void set_duty_cycle(motor *p_mot, int32_t duty);

#endif /* INC_MOTOR_H_ */
