#ifndef SV3_PC_COMM_H
#define SV3_PC_COMM_H

#include "stm32f1xx_hal.h"

// Ham gui du lieu tu STM32 len may tinh (Python)
void SV3_Send_GraphData(UART_HandleTypeDef *huart, float setpoint, float feedback);

// Ham nhan va phan tich lenh tu may tinh (Python) gui xuong
uint8_t SV3_Parse_PythonCommand(uint8_t rxChar, float *Kp, float *Ki, float *Kd, float *Sp);

#endif /* SV3_PC_COMM_H */