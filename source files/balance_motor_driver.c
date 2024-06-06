/*
 * motor_driver.c
 *
 *  Created on: Apr 25, 2024
 *      Author: jwald
 */
#include <balance_motor_driver.h>
#include <stdint.h>

//function to enable the motor channels to be pwm'd
void enable_motor(motor *p_mot){
		HAL_TIM_PWM_Start(p_mot->tim, (uint32_t)*p_mot->pwm);
}

//function to disable the motor channels
void disable_motor(motor *p_mot){
	HAL_TIM_PWM_Stop(p_mot->tim, (uint32_t)*p_mot->pwm);
}

// function to set the duty cycle
void set_duty_cycle(motor *p_mot, int32_t duty){

	if (duty > 0){
		__HAL_TIM_SET_COMPARE(p_mot->tim,p_mot->pwm,duty); // set the motor pwm
		HAL_GPIO_WritePin(p_mot->GPIO_Port, p_mot->GPIO_Pin_Pos, GPIO_PIN_SET); // set positive pin to spin in one direction
		HAL_GPIO_WritePin(p_mot->GPIO_Port, p_mot->GPIO_Pin_Neg, GPIO_PIN_RESET);
	}
	if (duty < 0){
		__HAL_TIM_SET_COMPARE(p_mot->tim,p_mot->pwm,-duty); // set the motor pwm
		HAL_GPIO_WritePin(p_mot->GPIO_Port, p_mot->GPIO_Pin_Pos, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(p_mot->GPIO_Port, p_mot->GPIO_Pin_Neg, GPIO_PIN_SET); // set negative pin to spin in other direction
	}
	if (duty == 0){
		__HAL_TIM_SET_COMPARE(p_mot->tim,p_mot->pwm,0); // turn of motor pwm
		HAL_GPIO_WritePin(p_mot->GPIO_Port, p_mot->GPIO_Pin_Pos, GPIO_PIN_SET);
		HAL_GPIO_WritePin(p_mot->GPIO_Port, p_mot->GPIO_Pin_Neg, GPIO_PIN_SET);
	}
}
