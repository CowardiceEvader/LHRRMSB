/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"

#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500

uint16_t servo_pwm[4] = {SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM};
/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          �������
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{
    while(1)
    {
        for(uint8_t i = 0; i < 4; i++)
        {
            servo_pwm[i] = SERVO_MIN_PWM;

            //limit the pwm
           //����pwm
            if(servo_pwm[i] < SERVO_MIN_PWM)
            {
                servo_pwm[i] = SERVO_MIN_PWM;
            }
            else if(servo_pwm[i] > SERVO_MAX_PWM)
            {
                servo_pwm[i] = SERVO_MAX_PWM;
            }

            servo_pwm_set(servo_pwm[i], i);
        }
        osDelay(10);
    }
}


