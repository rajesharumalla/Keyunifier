/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : IR Signal Recorder with Button Control - FIXED VERSION
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_i2c.h"
#include "i2c.h"
#include "keypad.h"
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

I2S_HandleTypeDef hi2s3;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

typedef enum {
    STATE_BOOT1,
    STATE_BOOT2,
    STATE_MAIN_MENU,
    STATE_IR_MENU,
    STATE_RF_MENU,
    STATE_RFID_MENU,
    STATE_RECORD,
    STATE_TRANSMIT
} SystemState;

SystemState currentState = STATE_BOOT1;
char lastKey = 0;
SystemState previousState = -1;

// IR Signal Storage
#define MAX_IR_PULSES 200
uint32_t ir_pulses[MAX_IR_PULSES];
uint16_t ir_pulse_count = 0;
volatile uint8_t is_recording = 0;
volatile uint8_t is_transmitting = 0;

// Input Capture Variables
uint32_t capture_val1 = 0;
uint32_t capture_val2 = 0;
uint8_t edge_count = 0;
uint32_t last_edge_time = 0;

// Button Detection Variables
#define DOUBLE_CLICK_TIMEOUT 300  // ms
volatile uint32_t button_press_time = 0;
volatile uint8_t button_press_count = 0;
volatile uint8_t button_event = 0; // 0=none, 1=single, 2=double
volatile uint32_t last_button_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM2_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void Start_IR_Recording(void);
void Stop_IR_Recording(void);
void Transmit_IR_Signal(void);
void LED_Blink(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t times);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void DrawScreen(SystemState state)
{
    LCD_Clear();

    switch(state)
    {
        case STATE_MAIN_MENU:
            LCD_Set_Cursor(0,0);
            LCD_Send_String("1.IR   2.RF");
            LCD_Set_Cursor(1,0);
            LCD_Send_String("3.RFID/NFC");
        break;

        case STATE_IR_MENU:
            LCD_Set_Cursor(0,0);
            LCD_Send_String("1.Record");
            LCD_Set_Cursor(1,0);
            LCD_Send_String("2.Transmit D:Back");
        break;

        case STATE_RF_MENU:
            LCD_Set_Cursor(0,0);
            LCD_Send_String("RF Module");
            LCD_Set_Cursor(1,0);
            LCD_Send_String("Under Dev  D:Back");
        break;

        case STATE_RFID_MENU:
            LCD_Set_Cursor(0,0);
            LCD_Send_String("RFID & NFC");
            LCD_Set_Cursor(1,0);
            LCD_Send_String("Under Dev  D:Back");
        break;

        case STATE_TRANSMIT:
            LCD_Set_Cursor(0,0);
            LCD_Send_String("A:Send");
            LCD_Set_Cursor(1,0);
            LCD_Send_String("D:Back");
        break;

        case STATE_RECORD:
            LCD_Set_Cursor(0,0);
            LCD_Send_String("Recording...");
        break;

        default:
        break;
    }
}

void I2C_Scan(void)
{
  for(uint8_t addr = 1; addr < 128; addr++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK)
    {
      volatile uint8_t found_address = addr;  // <-- put breakpoint here
    }
  }
}
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
  MX_I2S3_Init();
  MX_USB_HOST_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  I2C_Scan();
  // Enable DWT for microsecond timing
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Start Timer2 Input Capture with Interrupt
  // CRITICAL FIX: Use TIM_CHANNEL_1 not TIM_CHANNEL_2!
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  // System ready indication - blink LEDs
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

  // Green LED stays on to show system is ready

  LCD_Init();
  LCD_Clear();

  // Boot Screen 1
  LCD_Set_Cursor(0,0);
  LCD_Send_String("HI Welcome to");
  LCD_Set_Cursor(1,0);
  LCD_Send_String("Keyunifier");
  HAL_Delay(2000);

  // Boot Screen 2
  LCD_Clear();
  LCD_Set_Cursor(0,0);
  LCD_Send_String("Multiprotocol");
  LCD_Set_Cursor(1,0);
  LCD_Send_String("Signal Emulator");
  HAL_Delay(2000);

  LCD_Clear();
  currentState = STATE_MAIN_MENU;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      MX_USB_HOST_Process();

      static char lastKey = 0;
      char key = Keypad_GetKey();

      if(currentState != previousState)
      {
          DrawScreen(currentState);
          previousState = currentState;
      }

      // Detect NEW key press only
      if(key != 0 && key != lastKey)
      {
          lastKey = key;

          // Draw screen when state changes
          if(currentState != previousState)
          {
              DrawScreen(currentState);
              previousState = currentState;
          }

          switch(currentState)
          {
              case STATE_MAIN_MENU:
                  if(key == '1') currentState = STATE_IR_MENU;
                  else if(key == '2') currentState = STATE_RF_MENU;
                  else if(key == '3') currentState = STATE_RFID_MENU;
              break;

              case STATE_IR_MENU:
                  if(key == '1') currentState = STATE_RECORD;
                  else if(key == '2') currentState = STATE_TRANSMIT;
                  else if(key == 'D') currentState = STATE_MAIN_MENU;
              break;

              case STATE_RF_MENU:
                  if(key == 'D') currentState = STATE_MAIN_MENU;
              break;

              case STATE_RFID_MENU:
                  if(key == 'D') currentState = STATE_MAIN_MENU;
              break;
          }
      }
      else if(key == 0)
      {
          lastKey = 0;  // Reset when key released
      }

      HAL_Delay(50);
  }
  /* USER CODE END WHILE */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_8K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(output_GPIO_Port, output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : user_button_Pin */
  GPIO_InitStruct.Pin = user_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(user_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : output_Pin */
  GPIO_InitStruct.Pin = output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(output_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Start IR signal recording
  */
void Start_IR_Recording(void)
{
  ir_pulse_count = 0;
  edge_count = 0;
  last_edge_time = 0;
  is_recording = 1;

  // Turn ON orange LED to indicate recording
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);

  // Wait for IR signal (timeout after 10 seconds)
  uint32_t start_time = HAL_GetTick();

  while (is_recording && (HAL_GetTick() - start_time < 10000)) {
    HAL_Delay(10);

    // Check if signal has ended (no edge for 100ms)
    if (ir_pulse_count > 0 && (HAL_GetTick() - last_edge_time > 100)) {
      break;
    }
  }

  Stop_IR_Recording();
}

/**
  * @brief  Stop IR signal recording
  */
void Stop_IR_Recording(void)
{
  is_recording = 0;

  // Turn OFF orange LED
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

  if (ir_pulse_count > 0) {
    // Blink orange LED 3 times to indicate recording complete
    LED_Blink(GPIOD, GPIO_PIN_13, 3);
  } else {
    // No signal captured - blink blue LED (error)
    LED_Blink(GPIOD, GPIO_PIN_15, 3);
  }

  // Turn green LED back on to show ready
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
}

/**
  * @brief  Transmit recorded IR signal
  */
void Transmit_IR_Signal(void)
{
  is_transmitting = 1;

  // Turn ON red LED during transmission
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  // Transmit each pulse
  for (uint16_t i = 0; i < ir_pulse_count; i++) {
    if (i % 2 == 0) {
      // Even index = HIGH pulse
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    } else {
      // Odd index = LOW pulse
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    }

    // Delay for pulse duration (in microseconds)
    uint32_t pulse_duration = ir_pulses[i];

    // Use busy wait for microsecond delays
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = pulse_duration * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < cycles);
  }

  // Ensure output is LOW after transmission
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  // Turn OFF red LED
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

  // Blink red LED 3 times to indicate transmission complete
  LED_Blink(GPIOD, GPIO_PIN_14, 3);

  // Turn green LED back on
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

  is_transmitting = 0;
}

/**
  * @brief  Blink LED specified number of times
  */
void LED_Blink(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t times)
{
  for (uint8_t i = 0; i < times; i++) {
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    HAL_Delay(150);
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(150);
  }
}

/**
  * @brief  Timer Input Capture Callback
  * CRITICAL FIX: Changed from TIM_CHANNEL_2 to TIM_CHANNEL_1!
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2 && is_recording && !is_transmitting) {

    last_edge_time = HAL_GetTick();

    if (edge_count == 0) {
      // First edge - capture value
      capture_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      edge_count = 1;

    } else {
      // Second edge - calculate pulse width
      capture_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

      uint32_t pulse_width;
      if (capture_val2 >= capture_val1) {
        pulse_width = capture_val2 - capture_val1;
      } else {
        // Handle timer overflow
        pulse_width = (0xFFFFFFFF - capture_val1) + capture_val2;
      }

      // Store pulse width if space available and pulse is valid
      if (ir_pulse_count < MAX_IR_PULSES && pulse_width > 50) {
        ir_pulses[ir_pulse_count++] = pulse_width;
      }

      // Reset for next edge
      capture_val1 = capture_val2;
    }
  }
}

/**
  * @brief  External Interrupt Callback (Button)
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0) {
    uint32_t current_time = HAL_GetTick();

    // Debounce - ignore if pressed within 50ms
    if (current_time - last_button_time < 50) {
      return;
    }
    last_button_time = current_time;

    // User button pressed
    if (button_press_count == 0) {
      // First press
      button_press_time = current_time;
      button_press_count = 1;

    } else if (button_press_count == 1) {
      // Check if second press is within double-click timeout
      if ((current_time - button_press_time) < DOUBLE_CLICK_TIMEOUT) {
        // Double click detected
        button_event = 2;
        button_press_count = 0;
      }
    }
  }
}

/**
  * @brief  SysTick Callback - called every 1ms
  */
void HAL_SYSTICK_Callback(void)
{
  // Check for single click timeout
  if (button_press_count == 1) {
    uint32_t current_time = HAL_GetTick();
    if ((current_time - button_press_time) >= DOUBLE_CLICK_TIMEOUT) {
      // Single click detected
      button_event = 1;
      button_press_count = 0;
    }
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
