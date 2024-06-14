/*
 * rc_driver.h
 *
 *  Created on: May 17, 2024
 *      Author: jwald
 */

#ifndef INC_RC_DRIVER_H_
#define INC_RC_DRIVER_H_

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"

struct {
	TIM_HandleTypeDef *tim;
	uint32_t ch1;
	uint32_t ch2;
	uint32_t IC_Start;
	uint32_t IC_PWM;
}typedef RC_Signal;
//function that measures the active pulse width of the RC signal in microseconds
void RC_callback(RC_Signal *p_RC);

// function that saturates erroneous pulse width values then returns the pulse width in microseconds
uint32_t RC_Pulse(RC_Signal *p_RC);

#endif /* INC_RC_DRIVER_H_ */
