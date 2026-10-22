#ifndef I2C_LCD_H_
#define I2C_LCD_H_

#include "stm32f1xx_hal.h" // Thay d?i dòng này n?u b?n dùng dòng chip khác (ví d? stm32f4xx_hal.h)

// Kh?i t?o LCD (truy?n vào handle c?a I2C, ví d? &hi2c1)
void lcd_init (void);

// G?i l?nh di?u khi?n
void lcd_send_cmd (char cmd);

// G?i d? li?u (ký t?)
void lcd_send_data (char data);

// In chu?i ký t?
void lcd_send_string (char *str);

// Ð?t v? trí con tr? (hàng 0-1, c?t 0-15)
void lcd_goto_XY (int row, int col);

// Xóa màn hình
void lcd_clear (void);

#endif /* I2C_LCD_H_ */