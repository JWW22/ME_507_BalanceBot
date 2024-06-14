/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : balance_motor_driver.c
  * @brief          : This driver is meant to control the motors used for balancing our bot.
  ******************************************************************************
  * @author Johnathan Waldmire, Peter Tomson
  * @date   April 25, 2024
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include <balance_motor_driver.h>
#include <stdint.h>

/**
 * @brief  This function enables the motor channels to be PWM'd.
 * @retval None
 * @param[in] Motor Pointer
 */

void enable_motor(motor *p_mot){
		HAL_TIM_PWM_Start(p_mot->tim, (uint32_t)*p_mot->pwm);
}

/**
 * @brief  This function disables the motor channels.
 * @retval None
 * @param[in] Motor Pointer
 */
void disable_motor(motor *p_mot){
	HAL_TIM_PWM_Stop(p_mot->tim, (uint32_t)*p_mot->pwm);
}

/**
 * @brief  This function sets the duty cycle.
 * @retval None
 * @param[in] Motor Pointer
 * @param[in] Duty Value
 */
void set_duty_cycle(motor *p_mot, int32_t duty){

	if (duty > 0){
		__HAL_TIM_SET_COMPARE(p_mot->tim,p_mot->pwm,duty); // set the motor PWM
		HAL_GPIO_WritePin(p_mot->GPIO_Port, p_mot->GPIO_Pin_Pos, GPIO_PIN_SET); // set positive pin to spin in one direction
		HAL_GPIO_WritePin(p_mot->GPIO_Port, p_mot->GPIO_Pin_Neg, GPIO_PIN_RESET);
	}
	if (duty < 0){
		__HAL_TIM_SET_COMPARE(p_mot->tim,p_mot->pwm,-duty); // set the motor PWM
		HAL_GPIO_WritePin(p_mot->GPIO_Port, p_mot->GPIO_Pin_Pos, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(p_mot->GPIO_Port, p_mot->GPIO_Pin_Neg, GPIO_PIN_SET); // set negative pin to spin in other direction
	}
	if (duty == 0){
		__HAL_TIM_SET_COMPARE(p_mot->tim,p_mot->pwm,0); // turn off motor PWM
		HAL_GPIO_WritePin(p_mot->GPIO_Port, p_mot->GPIO_Pin_Pos, GPIO_PIN_SET);
		HAL_GPIO_WritePin(p_mot->GPIO_Port, p_mot->GPIO_Pin_Neg, GPIO_PIN_SET);
	}
}
