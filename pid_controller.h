#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// KHAI BÁO CÁC HÀM API TRUY XU?T MODULE PID
// ============================================================================

/**
 * @brief Kh?i t?o h? th?ng PID và dua các bi?n luu tr? v? tr?ng thái ban d?u
 */
void PID_Init(void);

/**
 * @brief Reset sai s? tích phân (Anti-Windup Reset) d? tránh gi?t c?c khi d?i di?m d?t
 */
void PID_Reset(void);

/**
 * @brief C?u hình thông s? Kp, Ki, Kd cho vòng T?c d? (Inner Loop)
 */
void PID_Set_Speed_Tunings(float kp, float ki, float kd);

/**
 * @brief C?u hình thông s? Kp, Ki, Kd cho vòng V? trí (Outer Loop)
 */
void PID_Set_Position_Tunings(float kp, float ki, float kd);

/**
 * @brief Hàm tính toán Cascade PID (Vòng v? trí bao ngoài vòng t?c d?)
 * 
 * @param controlMode   0: Ði?u khi?n T?c d?, 1: Ði?u khi?n V? trí
 * @param posSetpoint   Ði?m d?t V? trí (xung Encoder)
 * @param speedSetpoint Ði?m d?t T?c d? (xung/giây)
 * @param currentPos    T?ng s? xung Encoder th?c t? tích luy
 * @param currentSpeed  T?c d? ph?n h?i th?c t? dã l?c EMA (xung/giây)
 * @param pwmOutput     Con tr? nh?n tín hi?u PWM bão hòa ([-255, 255])
 */
void PID_Calculate_Cascade(uint8_t controlMode, 
                             float posSetpoint, 
                             float speedSetpoint, 
                             float currentPos, 
                             float currentSpeed, 
                             float *pwmOutput);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */