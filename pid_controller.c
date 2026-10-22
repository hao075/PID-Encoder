#include "pid_controller.h"

// ============================================================================
// THAM S? KH?I T?O VÀ BI?N N?I B? (Chu?n theo main.c)
// ============================================================================

// Chu k? ng?t ph?n c?ng Ts = 1ms (1kHz)
static const float Ts = 0.001f;

// Tham s? & Tr?ng thái Vòng T?c d? (Inner Loop)
static float Kp_v = 0.7f, Ki_v = 0.6f, Kd_v = 0.0f;
static float err_v = 0.0f, lastErr_v = 0.0f, inter_v = 0.0f;

// Tham s? & Tr?ng thái Vòng V? trí (Outer Loop)
static float Kp_p = 2.0f, Ki_p = 0.0f, Kd_p = 0.05f;
static float err_p = 0.0f, lastErr_p = 0.0f, inter_p = 0.0f;

// ============================================================================
// TH?C THI CÁC HÀM C?U HÌNH API
// ============================================================================

void PID_Init(void) {
    PID_Reset();
}

void PID_Reset(void) {
    err_v = 0.0f; lastErr_v = 0.0f; inter_v = 0.0f;
    err_p = 0.0f; lastErr_p = 0.0f; inter_p = 0.0f;
}

void PID_Set_Speed_Tunings(float kp, float ki, float kd) {
    Kp_v = kp; Ki_v = ki; Kd_v = kd;
}

void PID_Set_Position_Tunings(float kp, float ki, float kd) {
    Kp_p = kp; Ki_p = ki; Kd_p = kd;
}

// ============================================================================
// TH?C THI THU?T TOÁN CASCADE PID
// ============================================================================

void PID_Calculate_Cascade(uint8_t controlMode, 
                             float posSetpoint, 
                             float speedSetpoint, 
                             float currentPos, 
                             float currentSpeed, 
                             float *pwmOutput)
{
    float targetSpeed = 0.0f;

    // ------------------------------------------------------------------------
    // 1. VÒNG NGOÀI: ÐI?U KHI?N V? TRÍ (POSITION LOOP)
    // ------------------------------------------------------------------------
    if (controlMode == 1) 
    {
        // Sai s? v? trí
        err_p = posSetpoint - currentPos;

        // Anti-Windup v? trí (Clamping [-500, 500])
        inter_p += err_p * Ts;
        if (inter_p > 500.0f)  inter_p = 500.0f;
        if (inter_p < -500.0f) inter_p = -500.0f;

        // Ð?o hàm v? trí
        float deriv_p = (err_p - lastErr_p) / Ts;
        lastErr_p = err_p;

        // T?c d? d?t mong mu?n sinh ra t? Vòng V? trí
        targetSpeed = (Kp_p * err_p) + (Ki_p * inter_p) + (Kd_p * deriv_p);

        // Bão hòa t?c d? t?i da cho phép (300 xung/s)
        float maxAllowedSpeed = 300.0f;
        if (targetSpeed > maxAllowedSpeed)   targetSpeed = maxAllowedSpeed;
        if (targetSpeed < -maxAllowedSpeed)  targetSpeed = -maxAllowedSpeed;

        // Kh?ng ch? dao d?ng nh? (Hunting) quanh di?m d?t
        if (fabsf(posSetpoint - currentPos) < 2.0f && fabsf(currentSpeed) < 5.0f) 
        {
            targetSpeed = 0.0f;
        }
    }
    else 
    {
        // Ch? d? ch?y t?c d? tr?c ti?p
        targetSpeed = speedSetpoint;

        if (speedSetpoint == 0.0f) 
        {
            PID_Reset();
            *pwmOutput = 0.0f;
            return;
        }
    }

    // ------------------------------------------------------------------------
    // 2. VÒNG TRONG: ÐI?U KHI?N T?C Ð? (VELOCITY LOOP)
    // ------------------------------------------------------------------------
    // Sai s? t?c d?
    err_v = targetSpeed - currentSpeed;

    // Anti-Windup t?c d? (Clamping [-1000, 1000])
    inter_v += err_v * Ts;
    if (inter_v > 1000.0f)  inter_v = 1000.0f;
    if (inter_v < -1000.0f) inter_v = -1000.0f;

    // Ð?o hàm t?c d?
    float deriv_v = (err_v - lastErr_v) / Ts;
    lastErr_v = err_v;

    // Tín hi?u di?u khi?n thô
    float rawPwm = (Kp_v * err_v) + (Ki_v * inter_v) + (Kd_v * deriv_v);

    // ------------------------------------------------------------------------
    // 3. BÃO HÒA Ð?U RA PWM (OUTPUT SATURATION)
    // ------------------------------------------------------------------------
    if (rawPwm > 255.0f)   rawPwm = 255.0f;
    if (rawPwm < -255.0f)  rawPwm = -255.0f;

    // Tr? k?t qu? PWM bão hòa qua con tr?
    *pwmOutput = rawPwm;
}