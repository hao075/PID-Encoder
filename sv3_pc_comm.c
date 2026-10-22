#include "sv3_pc_comm.h"
#include <stdio.h>
#include <stdlib.h>

// Bien dem noi bo (an khoi main.c)
static uint8_t rxBuffer[32];
static uint8_t rxIndex = 0;

void SV3_Send_GraphData(UART_HandleTypeDef *huart, float setpoint, float feedback) {
    char buf[64];
    int len = sprintf(buf, "SP:%.1f,PV:%.1f\r\n", (double)setpoint, (double)feedback);
    
    if (huart->gState == HAL_UART_STATE_READY) {
        HAL_UART_Transmit(huart, (uint8_t*)buf, len, 20);
    }
}

uint8_t SV3_Parse_PythonCommand(uint8_t rxChar, float *Kp, float *Ki, float *Kd, float *Sp) {
    if (rxChar == '\n' || rxChar == '\r') {
        if (rxIndex > 0) {
            rxBuffer[rxIndex] = '\0';
            char cmd = rxBuffer[0];
            float val = atof((char*)&rxBuffer[1]); 
            
            if (cmd == 'P' || cmd == 'p') *Kp = val;
            else if (cmd == 'I' || cmd == 'i') *Ki = val;
            else if (cmd == 'D' || cmd == 'd') *Kd = val;
            else if (cmd == 'S' || cmd == 's') {
                *Sp = val;
                rxIndex = 0;
                return 1; // Bao hieu Setpoint thay doi
            }
            rxIndex = 0;
        }
    } 
    else {
        if (rxIndex < sizeof(rxBuffer) - 1) {
            rxBuffer[rxIndex++] = rxChar;
        }
    }
    return 0;
}