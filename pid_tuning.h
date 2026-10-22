#ifndef __PID_TUNING_H
#define __PID_TUNING_H

#include "stm32f1xx_hal.h"
#include <math.h>


typedef struct {
    float overshoot;       // Ðo vot lo tính theo % (OV%)
    uint32_t settlingTime; // Thoi gian xác lap tính bang ms (Ts)
    float maxVal;          // Giá tri dinh PV lon nhat dat duoc
    uint32_t stepStartTick;// Moc thoi gian HAL_GetTick() khi bat dau lenh nhay bac
} PID_Performance_t;

/* --- Danh sách các hàm API công khai --- */


void PID_Tuning_Init(PID_Performance_t* perf);


void PID_Tuning_ResetStep(PID_Performance_t* perf);


void PID_Tuning_CalculatePerformance(float speedSetpoint, float filteredSpeed, PID_Performance_t* perf);

#endif /* __PID_TUNING_H */