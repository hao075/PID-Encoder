/**
 * ******************************************************************************
 * @file           : pid_motor.h
 * @brief          : Header file for DC Motor Cascade PID Control Module
 * ******************************************************************************
 */

#ifndef PID_MOTOR_H
#define PID_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h" // Thay d?i dòng này tùy thu?c vào dòng chip STM32 b?n dùng (vd: stm32f4xx_hal.h)

/* Public Variables Extern -------------------------------------------------*/
extern float speedSetpoint;
extern float filteredSpeed;
extern float Kp_v, Ki_v, Kd_v;
extern float Kp_p, Ki_p, Kd_p;
extern uint8_t controlMode;
extern float overshoot;
extern uint32_t settlingTimeMs;

/* Function Prototypes -----------------------------------------------------*/
void PID_Motor_Init(TIM_HandleTypeDef *pwmTimer, TIM_HandleTypeDef *encTimer);
void PID_Motor_Process_1ms(void);
void Calculate_Cascade_PID(void);
void Motor_Control(float pwmOut);
void PID_Motor_HandleCommand(char *rxStr);

#ifdef __cplusplus
}
#endif

#endif /* PID_MOTOR_H */