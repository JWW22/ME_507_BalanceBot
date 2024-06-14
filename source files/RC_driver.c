/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : rc_driver.c
  * @brief          : This is a class that can be used to read a PWM signal from an RC controller.
  ******************************************************************************
  * @author Johnathan Waldmire, Peter Tomson
  * @date   May 17, 2024
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include <rc_driver.h>
#include <stdint.h>

/**
 * @brief  This function measures the active pulse width of the RC signal in microseconds.
 * @retval None
 * @param[in] RC Receiver Pointer
 */
void RC_callback(RC_Signal *p_RC){
	if (p_RC->tim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		p_RC->IC_Start = HAL_TIM_ReadCapturedValue(p_RC->tim, p_RC->ch1); // read the first value
		if (p_RC->IC_Start)
		{
			p_RC->IC_PWM = HAL_TIM_ReadCapturedValue(p_RC->tim, p_RC->ch2);
		}
	}
}

/**
 * @brief  This function saturates erroneous pulse width values then returns the pulse width in microseconds.
 * @retval PWM Value
 * @param[in] RC Receiver Pointer
 */

uint32_t RC_Pulse(RC_Signal *p_RC){
	if (p_RC->IC_PWM > 2000)
	{
		p_RC->IC_PWM = 2000;
		return p_RC->IC_PWM;
	}
	if (p_RC->IC_PWM < 	1000)
	{
		p_RC->IC_PWM = 1000;
		return p_RC->IC_PWM;
	}
	else
	{
		return p_RC->IC_PWM;
	}
}


