/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// --- 1. BIẾN ĐO LƯỜNG VỊ TRÍ & TỐC ĐỘ ---
int32_t totalEncoderTicks = 0;   // Tổng số xung tích lũy (Vị trí)
float currentSpeed = 0.0f;       // Tốc độ tức thời (Xung/s)
float filteredSpeed = 0.0f;      // Tốc độ sau lọc EMA
const float alpha = 0.03f;       // Hệ số lọc EMA

// --- 2. THAM SỐ PID TỐC ĐỘ (VÒNG TRONG) ---
float Kp_v = 0.7f, Ki_v = 0.6f, Kd_v = 0.0f;
float err_v = 0, lastErr_v = 0, inter_v = 0;
float speedSetpoint = 0.0f;      // Tốc độ đặt

// --- 3. THAM SỐ PID VỊ TRÍ (VÒNG NGOÀI) ---
float Kp_p = 2.0f, Ki_p = 0.0f, Kd_p = 0.05f;
float err_p = 0, lastErr_p = 0, inter_p = 0;
float posSetpoint = 0.0f;        // Vị trí đặt
uint8_t controlMode = 0;         // 0: Điều khiển Tốc độ, 1: Điều khiển Vị trí

// --- 4. BIẾN ĐÁNH GIÁ ĐÁP ỨNG (OVERSHOOT & SETTLING TIME) ---
float maxVal = 0.0f;
float overshoot = 0.0f;          // Độ vọt lố (%)
uint32_t settlingTimeMs = 0;     // Thời gian xác lập (ms)
uint32_t stepStartTick = 0;

// Chu kỳ ngắt Ts = 1ms (1kHz)
const float Ts = 0.001f;

// Nhận UART
uint8_t rxBuffer[16];
uint8_t rxData;
uint8_t rxIndex = 0;
uint32_t lastDisplayTick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Calculate_Cascade_PID(void);
void Motor_Control(float pwmOut);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); 
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       
  HAL_UART_Receive_IT(&huart1, &rxData, 1);        

  HAL_Delay(100);
  lcd_init();
  lcd_goto_XY(1, 0);
  lcd_send_string("PID CONTROL 1kHz");
  HAL_Delay(1000);
  lcd_clear();

  // Bật ngắt Timer 3 (1kHz)
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t lastDisplayTick = 0;
  while (1)
  {
		/* USER CODE BEGIN WHILE */
  if (HAL_GetTick() - lastDisplayTick >= 200)
    {
      lastDisplayTick = HAL_GetTick();

      char buf1[17], buf2[17];

      // HÀNG 1: SP VÀ PV (Chuỗi 16 ô chuẩn khít LCD)
      snprintf(buf1, sizeof(buf1), "SP:%-4.0f  PV:%-4.0f", (double)speedSetpoint, (double)filteredSpeed);

      // HÀNG 2: OV% VÀ Ts (Chuỗi 16 ô chuẩn khít LCD)
      snprintf(buf2, sizeof(buf2), "OV:%-2.0f%%  Ts:%-3lums", (double)overshoot, (unsigned long)settlingTimeMs);

      // Hiển thị lên LCD
      lcd_goto_XY(1, 0);
      lcd_send_string(buf1);
      lcd_goto_XY(2, 0);
      lcd_send_string(buf2);

      // Gửi dữ liệu UART lên PC (Serial Plotter/Hercules)
      char uartBuf[64];
      int len = sprintf(uartBuf, "SP:%.1f,PV:%.1f\r\n", (double)speedSetpoint, (double)filteredSpeed);
      
      if (huart1.gState == HAL_UART_STATE_READY) {
        HAL_UART_Transmit(&huart1, (uint8_t*)uartBuf, len, 20);
      }
    }
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if (htim->Instance == TIM3) // Ngắt 1ms
  {
    static uint8_t sampleDivider = 0;
    
    // 1. Đọc bám vị trí liên tục mỗi 1ms (để vòng vị trí X không bị trễ)
    int16_t rawCount = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    int16_t diff = -rawCount;
    totalEncoderTicks += diff;

    // Accumulate ticks cho tính tốc độ
    static int32_t speedTickAcc = 0;
    speedTickAcc += diff;

    // 2. Chỉ tính Tốc độ và PID mỗi 5ms (Chia tần số tính toán xuống 200Hz để tín hiệu chuẩn đét)
    sampleDivider++;
    if (sampleDivider >= 5) 
    {
      sampleDivider = 0;

      // Tốc độ (Xung/giây) tính trong khoảng 5ms = 0.005s
      currentSpeed = ((float)speedTickAcc / 5.0f) * 1000.0f; 
      speedTickAcc = 0;

      // Lọc EMA siêu phẳng
      filteredSpeed = (0.03f * currentSpeed) + (0.97f * filteredSpeed);

      // Tính toán PID
      Calculate_Cascade_PID();
    }
  }
}

void Calculate_Cascade_PID(void)
{
 float targetSpeed = speedSetpoint;

  if (speedSetpoint == 0.0f)
  {
    inter_v = 0.0f;
    lastErr_v = 0.0f;
    settlingTimeMs = 0;
    maxVal = 0;
    overshoot = 0;
    Motor_Control(0.0f);
    return;
  }

  // --- VÒNG TỐC ĐỘ (SPEED LOOP) ---
  err_v = targetSpeed - filteredSpeed;

  inter_v += err_v * Ts;

  if (inter_v > 1000.0f) inter_v = 1000.0f;
  if (inter_v < -1000.0f) inter_v = -1000.0f;

  float deriv_v = (err_v - lastErr_v) / Ts;
  lastErr_v = err_v;

  float pwmOut = (Kp_v * err_v) + (Ki_v * inter_v) + (Kd_v * deriv_v);

  if (pwmOut > 255.0f) pwmOut = 255.0f;
  if (pwmOut < -255.0f) pwmOut = -255.0f;

  Motor_Control(pwmOut);

  // --- TÍNH OVERSHOOT VÀ SETTLING TIME CHUẨN XÁC ---
  uint32_t elapsedTime = HAL_GetTick() - stepStartTick;

  float absSP = fabsf(speedSetpoint);
  float absPV = fabsf(filteredSpeed);

  // 1. Cập nhật đỉnh vọt lố maxVal
  if (absPV > maxVal) {
    maxVal = absPV;
    if (maxVal > absSP) {
      overshoot = ((maxVal - absSP) / absSP) * 100.0f;
    }
  }

  // 2. Chỉ tính Ts nếu đã khởi chạy > 50ms và PV đi vào dải 90% - 110% SP
  if (elapsedTime > 50) 
  {
    if (absPV >= (absSP * 0.90f) && absPV <= (absSP * 1.10f)) 
    {
      if (settlingTimeMs == 0) {
        settlingTimeMs = elapsedTime; // Bắt chính xác thời gian xác lập thực tế
      }
    }
  }
}
void Motor_Control(float pwmOut)
{
 if (pwmOut == 0.0f || speedSetpoint == 0.0f)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    return;
  }

  float minPwm = 30.0f;

  if (pwmOut > 0.0f)
  {
    // QUAY THUẬN
    if (pwmOut < minPwm) pwmOut = minPwm;
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)pwmOut);
  }
  else
  {
    // QUAY NGHỊCH (ĐẢO CHIỀU)
    float absPwm = -pwmOut; // Lấy giá trị tuyệt đối số dương
    if (absPwm < minPwm) absPwm = minPwm;
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)absPwm);
  }
}

// Xử lý ngắt nhận UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (rxData == '\n' || rxData == '\r')
    {
      if (rxIndex > 0)
      {
        rxBuffer[rxIndex] = '\0';
        char cmd = rxBuffer[0];
      float val = atof((char*)&rxBuffer[1]);
        if (cmd == 'P' || cmd == 'p') Kp_v = val;
        else if (cmd == 'I' || cmd == 'i') Ki_v = val;
        else if (cmd == 'D' || cmd == 'd') Kd_v = val;
        
        // --- 1. NHẬN LỆNH ĐẶT TỐC ĐỘ (S) ---
        else if (cmd == 'S' || cmd == 's') { 
          speedSetpoint = val; 
          controlMode = 0; 
          
          // XÓA SẠCH TÍCH PHÂN CŨ ĐỂ TRÁNH BÃỎ HÒA PID (ANTI-WINDUP)
          inter_v = 0; 
          lastErr_v = 0;
          
          maxVal = 0; 
          overshoot = 0; 
          settlingTimeMs = 0; 
          stepStartTick = HAL_GetTick();

          // NẾU NHẬP S0: ÉP DỪNG ĐỘNG CƠ NGAY
          if (val == 0.0f) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
          }
        }
        
        // --- 2. NHẬN LỆNH ĐẶT VỊ TRÍ (X) ---
        else if (cmd == 'X' || cmd == 'x') { 
          posSetpoint = val; 
          controlMode = 1; 
          
          // Xóa tích phân cả 2 vòng
          inter_p = 0; 
          lastErr_p = 0;
          inter_v = 0; 
          lastErr_v = 0;

          maxVal = 0; 
          overshoot = 0; 
          settlingTimeMs = 0; 
          stepStartTick = HAL_GetTick();

          if (val == 0.0f) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
          }
        }
        
        rxIndex = 0; // Reset chỉ số buffer
      }
    }
    else {
      if (rxIndex < sizeof(rxBuffer) - 1) rxBuffer[rxIndex++] = rxData;
    }
    
    // KÍCH HOẠT LẠI NGẮT NHẬN KÝ TỰ TIẾP THEO
    HAL_UART_Receive_IT(&huart1, &rxData, 1);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
