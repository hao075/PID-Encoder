#include "pid_tuning.h"

 // Khoi tao ban dau cho bien quan ly chi tieu
void PID_Tuning_Init(PID_Performance_t* perf)
{
    perf->overshoot = 0.0f;
    perf->settlingTime = 0;
    perf->maxVal = 0.0f;
    perf->stepStartTick = 0;
}
  // Reset cac thong so do dac moi khi gui lenh Setpoint moi
 void PID_Tuning_ResetStep(PID_Performance_t* perf)
{
    perf->overshoot = 0.0f;
    perf->settlingTime = 0;
    perf->maxVal = 0.0f;
    perf->stepStartTick = HAL_GetTick(); 
}
 //Tính toán tu dong OV% và Ts trong luong lay mau / ngat Timer
  void PID_Tuning_CalculatePerformance(float speedSetpoint, float filteredSpeed, PID_Performance_t* perf)
{
    if (speedSetpoint == 0.0f) return;
    uint32_t elapsedTime = HAL_GetTick() - perf->stepStartTick;
    float absSP = fabsf(speedSetpoint);
    float absPV = fabsf(filteredSpeed);

    // 1. Cap nhat gia tri dinh lon nhat de tinh do vot lo (OV%)
    if (absPV > perf->maxVal) {
        perf->maxVal = absPV;
        if (perf->maxVal > absSP) {
            perf->overshoot = ((perf->maxVal - absSP) / absSP) * 100.0f;
        }
    }

    // 2. Tính thoi gian xac lap (Ts) khi PV di vào dai 90% - 110% SP (Bo qua 50ms quá do ban dau)
    if (elapsedTime > 50) {
        if (absPV >= (absSP * 0.90f) && absPV <= (absSP * 1.10f)) {
            if (perf->settlingTime == 0) {
                perf->settlingTime = elapsedTime; 
            }
        }
    }
}
