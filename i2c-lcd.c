#include "i2c-lcd.h"

extern I2C_HandleTypeDef hi2c1; // Ð?m b?o tên hi2c1 kh?p v?i c?u hình trong main.c c?a b?n

#define SLAVE_ADDRESS_LCD 0x4E // Ð?a ch? I2C c?a LCD (thu?ng là 0x27 ho?c 0x3F d?ch sang trái 1 bit thành 0x4E ho?c 0x7E)

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (cmd & 0xf0);
  data_l = ((cmd << 4) & 0xf0);
  data_t[0] = data_u | 0x0C;  // en=1, rs=0
  data_t[1] = data_u | 0x08;  // en=0, rs=0
  data_t[2] = data_l | 0x0C;  // en=1, rs=0
  data_t[3] = data_l | 0x08;  // en=0, rs=0
  HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *)data_t, 4, 100);
}

void lcd_send_data (char data)
{
  char data_u, data_l;
  uint8_t data_t[4];
  data_u = (data & 0xf0);
  data_l = ((data << 4) & 0xf0);
  data_t[0] = data_u | 0x0D;  // en=1, rs=1
  data_t[1] = data_u | 0x09;  // en=0, rs=1
  data_t[2] = data_l | 0x0D;  // en=1, rs=1
  data_t[3] = data_l | 0x09;  // en=0, rs=1
  HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *)data_t, 4, 100);
}

void lcd_init (void)
{
  HAL_Delay(50);
  lcd_send_cmd (0x30);
  HAL_Delay(5);
  lcd_send_cmd (0x30);
  HAL_Delay(1);
  lcd_send_cmd (0x30);
  HAL_Delay(10);
  lcd_send_cmd (0x20);  // 4-bit mode
  HAL_Delay(10);

  lcd_send_cmd (0x28); // Function set --> 4-bit mode, 2 line, 5x8 dots
  HAL_Delay(1);
  lcd_send_cmd (0x08); // Display on/off control --> Display off, cursor off
  HAL_Delay(1);
  lcd_send_cmd (0x01); // Clear display
  HAL_Delay(2);
  lcd_send_cmd (0x06); // Entry mode set --> Increment cursor
  HAL_Delay(1);
  lcd_send_cmd (0x0C); // Display on, cursor off
}

void lcd_send_string (char *str)
{
  while (*str) lcd_send_data (*str++);
}

void lcd_goto_XY(int row, int col)
{
    uint8_t pos;
    if (row == 1)
    {
        pos = 0x80 + col; // Hàng 1 bat dau tu dia chi 0x80
    }
    else if (row == 2)
    {
        pos = 0xC0 + col; // Hàng 2 bat dau tu dia chi 0xC0
    }
    lcd_send_cmd(pos);
}

void lcd_clear (void)
{
  lcd_send_cmd(0x01);
  HAL_Delay(2);
}