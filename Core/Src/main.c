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
#include "adc.h"
#include "bdma.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "types.h"
#include "ctrl.h"
#include "handlers.h"

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

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */

uint16_t spi1_rx_buf[16] __attribute__((section(".dma_buffer"), aligned(32))); // X
uint16_t spi1_tx_buf[16] __attribute__((section(".dma_buffer"), aligned(32)));
uint16_t spi2_rx_buf[16] __attribute__((section(".dma_buffer"), aligned(32))); // Y
uint16_t spi2_tx_buf[16] __attribute__((section(".dma_buffer"), aligned(32)));
uint16_t spi4_rx_buf[16] __attribute__((section(".dma_buffer"), aligned(32))); // A
uint16_t spi4_tx_buf[16] __attribute__((section(".dma_buffer"), aligned(32)));
uint16_t spi6_rx_buf[16] __attribute__((section(".bdma_buffer"), aligned(32))); // C
uint16_t spi6_tx_buf[16] __attribute__((section(".bdma_buffer"), aligned(32)));

uint8_t spi3_rx_buf[sizeof(SPIPacket)] __attribute__((section(".dma_buffer"), aligned(32))); // from raspi
  // Raspberry SPI3
uint8_t spi3_tx_buf_active[sizeof(SPIPacket)] __attribute__((section(".dma_buffer"), aligned(32)));
uint8_t spi3_tx_buf_staging[sizeof(SPITxPacket)] __attribute__((section(".dma_buffer"), aligned(32)));

// uint16_t spi4_single_buf[16] __attribute__((section(".dma_buffer"), aligned(32)));

volatile uint8_t current_axis_idx = 0; 
float current_values[3];

uint8_t machine_state = INIT;
Axis axis_X, axis_Y;
Stepper axis_Z1, axis_Z2, axis_A1, axis_A2, axis_C;
Encoder enc_rot_X, enc_rot_Y, enc_rot_Z, enc_rot_A, enc_rot_C, enc_rot_F;
Encoder enc_lin_X, enc_lin_Y;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_Delay(250);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_Delay(500);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM15_Init();
  MX_TIM17_Init();
  MX_SPI6_Init();
  MX_TIM23_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  enc_rot_X.g_ratio = 27.0f;
  enc_rot_Y.g_ratio = 24.0f;
  enc_rot_Z.g_ratio = 1.0f;
  enc_rot_A.g_ratio = 1.0f;
  enc_rot_C.g_ratio = 1.0f;
  enc_rot_F.g_ratio = 1.0f;

  /*
   __  __             _     
   \ \/ /   __ ___  _(_)___ 
    \  /   / _` \ \/ / / __|
    /  \  | (_| |>  <| \__ \
   /_/\_\  \__,_/_/\_\_|___/
                            
  */
  axis_X._enc_rot = &enc_rot_X; 
  axis_X._enc_lin = &enc_lin_X;
  axis_X._pwm_register = &TIM1->CCR1;
  axis_X._enc_rot -> _offset = 0.0f;
  PID_init(&axis_X._pid_pos, 100.0f, 0.01f, 0.001f, 100.0f, 1000.0f); // 300mm/min -> 5mm/s
  PID_init(&axis_X._pid_vel, 40.0f, 0.01f, 0.001f, 8000.0f, 1000.0f);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  /*
   __   __     _          _     
   \ \ / /    / \   __  _(_)___ 
    \ V /    / _ \  \ \/ / / __|
     | |    / ___ \  >  <| \__ \
     |_|   /_/   \_\/_/\_\_|___/
                                
  */
  axis_Y._enc_rot = &enc_rot_Y;
  axis_Y._enc_lin = &enc_lin_Y;
  axis_Y._pwm_register = &TIM1->CCR3;
  axis_Y._enc_rot -> _offset = 0.0f;
  PID_init(&axis_Y._pid_pos, 60.0f, 0.009f, 0.001f, 75.0f, 500.0f);
  // PID_init(&axis_Y._pid_vel, 366.4f, 0.0916f, 0.000916f, 3000.0f);
  PID_init(&axis_Y._pid_vel, 18.0f, 0.003f, 0.001f, 10000.0f, 500.0f);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  /*
    _____     _          _     
   |__  /    / \   __  _(_)___ 
     / /    / _ \  \ \/ / / __|
    / /_   / ___ \  >  <| \__ \
   /____| /_/   \_\/_/\_\_|___/
                               
  */
  axis_Z1._enc_rot = &enc_rot_Z;
  axis_Z1._enc_rot -> _offset = 0.0f;
  axis_Z1._enc_rot -> _converted_value = 0.0f;
  axis_Z1._target = 0.0f;
  axis_Z1.steps_per_unit = 400.0f;
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1); 

  axis_Z2._enc_rot = &enc_rot_Z;
  axis_Z2._enc_rot -> _offset = 0.0f;
  axis_Z2._enc_rot -> _converted_value = 0.0f;
  axis_Z2._target = 0.0f;
  axis_Z2.steps_per_unit = 400.0f;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /*
       _         _          _     
      / \       / \   __  _(_)___ 
     / _ \     / _ \  \ \/ / / __|
    / ___ \   / ___ \  >  <| \__ \
   /_/   \_\ /_/   \_\/_/\_\_|___/
                                  
  */
  axis_A1._enc_rot = &enc_rot_A;
  axis_A1._enc_rot -> _offset = 0.0f;
  axis_A1.steps_per_unit = 8.888889f;
  axis_A1._dir = 0;
  axis_A1._in_position = 0;
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

  axis_A2._enc_rot = &enc_rot_A;
  axis_A2._enc_rot -> _offset = 0.0f;
  axis_A2.steps_per_unit = 8.888889f;
  axis_A2._dir = 1; // reverse direction
  axis_A2._in_position = 0;
  HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);  // HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1); // period
  // HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2); // duty

  /*
     ____      _          _     
    / ___|    / \   __  _(_)___ 
   | |       / _ \  \ \/ / / __|
   | |___   / ___ \  >  <| \__ \
    \____| /_/   \_\/_/\_\_|___/
                                
  */
  axis_C._enc_rot = &enc_rot_C;
  axis_C._enc_rot -> _offset = 0.0f;
  axis_C.steps_per_unit = 8.888889f;
  axis_C._dir = 1;
  axis_C._in_position = 0;
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  // HAL_TIM_IC_Start(&htim23, TIM_CHANNEL_1); // period
  // HAL_TIM_IC_Start(&htim23, TIM_CHANNEL_2); // duty
  
  HAL_Delay(2);
  update_rotary_encoder(&enc_rot_A, spi4_rx_buf[0], 0.002);
  update_rotary_encoder(&enc_rot_C, spi6_rx_buf[0], 0.002);
  enc_rot_Z._converted_value = 0.0f; // homing needed
  
  uint16_t cmd_clear = 0x4001;
  uint16_t cmd_read  = 0xFFFF;
  uint16_t dummy     = 0;

  /*
    ____  ____ ___ _   ___       _ _   
   / ___||  _ \_ _/ | |_ _|_ __ (_) |_ 
   \___ \| |_) | || |  | || '_ \| | __|
    ___) |  __/| || |  | || | | | | |_ 
   |____/|_|  |___|_| |___|_| |_|_|\__|
                                       
  */
  HAL_GPIO_WritePin(SPI1_CSS_GPIO_Port, SPI1_CSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&cmd_clear, (uint8_t*)&dummy, 1, 100);
  HAL_GPIO_WritePin(SPI1_CSS_GPIO_Port, SPI1_CSS_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(SPI1_CSS_GPIO_Port, SPI1_CSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&cmd_read, (uint8_t*)&dummy, 1, 100);
  HAL_GPIO_WritePin(SPI1_CSS_GPIO_Port, SPI1_CSS_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(SPI1_CSS_GPIO_Port, SPI1_CSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&cmd_read, (uint8_t*)&dummy, 1, 100);
  HAL_GPIO_WritePin(SPI1_CSS_GPIO_Port, SPI1_CSS_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  spi1_tx_buf[0] = 0xFFFF;
  SCB_CleanDCache_by_Addr((uint32_t*)spi1_tx_buf, sizeof(spi1_tx_buf));

  /*
    ____  ____ ___ ____    ___       _ _   
   / ___||  _ \_ _|___ \  |_ _|_ __ (_) |_ 
   \___ \| |_) | |  __) |  | || '_ \| | __|
    ___) |  __/| | / __/   | || | | | | |_ 
   |____/|_|  |___|_____| |___|_| |_|_|\__|
                                           
  */
  HAL_GPIO_WritePin(SPI2_CSS_GPIO_Port, SPI2_CSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&cmd_clear, (uint8_t*)&dummy, 1, 100);
  HAL_GPIO_WritePin(SPI2_CSS_GPIO_Port, SPI2_CSS_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(SPI2_CSS_GPIO_Port, SPI2_CSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&cmd_read, (uint8_t*)&dummy, 1, 100);
  HAL_GPIO_WritePin(SPI2_CSS_GPIO_Port, SPI2_CSS_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(SPI2_CSS_GPIO_Port, SPI2_CSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&cmd_read, (uint8_t*)&dummy, 1, 100);
  HAL_GPIO_WritePin(SPI2_CSS_GPIO_Port, SPI2_CSS_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  spi2_tx_buf[0] = 0xFFFF;
  SCB_CleanDCache_by_Addr((uint32_t*)spi2_tx_buf, sizeof(spi2_tx_buf));
  
  enc_rot_Y._last_raw_pos = (float)(dummy & 0x3FFF) * (360.0f / 16384.0f);
  enc_rot_X._last_raw_pos = (float)(dummy & 0x3FFF) * (360.0f / 16384.0f);
  
  /*
    ____  ____ ___ _____   ___       _ _   
   / ___||  _ \_ _|___ /  |_ _|_ __ (_) |_ 
   \___ \| |_) | |  |_ \   | || '_ \| | __|
    ___) |  __/| | ___) |  | || | | | | |_ 
   |____/|_|  |___|____/  |___|_| |_|_|\__|
                                           
  */
  memset(spi3_tx_buf_active, 0, sizeof(spi3_tx_buf_active));
  spi3_tx_buf_active[0] = 0xBB; 
    SCB_CleanDCache_by_Addr((uint32_t*)spi3_tx_buf_active, sizeof(spi3_tx_buf_active));
  SCB_CleanDCache_by_Addr((uint32_t*)spi3_rx_buf, sizeof(spi3_rx_buf));
  if (HAL_SPI_TransmitReceive_DMA(&hspi3, spi3_tx_buf_active, spi3_rx_buf, sizeof(SPIPacket)) != HAL_OK) {
      // Se fallisce l'avvio del DMA, accende i LED e si ferma per debug
      Error_Handler();
  }

  /*
    ____  ____ ___ _  _     ___       _ _   
   / ___||  _ \_ _| || |   |_ _|_ __ (_) |_ 
   \___ \| |_) | || || |_   | || '_ \| | __|
    ___) |  __/| ||__   _|  | || | | | | |_ 
   |____/|_|  |___|  |_|   |___|_| |_|_|\__|
                                            
  */
  HAL_GPIO_WritePin(SPI4_CSS_GPIO_Port, SPI4_CSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi4, (uint8_t*)&cmd_clear, (uint8_t*)&dummy, 1, 100);
  HAL_GPIO_WritePin(SPI4_CSS_GPIO_Port, SPI4_CSS_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(SPI4_CSS_GPIO_Port, SPI4_CSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi4, (uint8_t*)&cmd_read, (uint8_t*)&dummy, 1, 100);
  HAL_GPIO_WritePin(SPI4_CSS_GPIO_Port, SPI4_CSS_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(SPI4_CSS_GPIO_Port, SPI4_CSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi4, (uint8_t*)&cmd_read, (uint8_t*)&dummy, 1, 100);
  HAL_GPIO_WritePin(SPI4_CSS_GPIO_Port, SPI4_CSS_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  spi4_tx_buf[0] = 0xFFFF;
  SCB_CleanDCache_by_Addr((uint32_t*)spi4_tx_buf, sizeof(spi4_tx_buf));
  
  enc_rot_A._last_raw_pos = (float)(dummy & 0x3FFF) * (360.0f / 16384.0f);
  if (enc_rot_A._last_raw_pos > 180.0f) enc_rot_A._turns = -1;

  /*
    ____  ____ ___ __     ___       _ _                                              
   / ___||  _ \_ _/ /_   |_ _|_ __ (_) |_                                            
   \___ \| |_) | | '_ \   | || '_ \| | __|                                           
    ___) |  __/| | (_) |  | || | | | | |_                                            
   |____/|_|  |___\___/  |___|_| |_|_|\__|                                           
                                                                                     
  */
  HAL_GPIO_WritePin(SPI6_CSS_GPIO_Port, SPI6_CSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi6, (uint8_t*)&cmd_clear, (uint8_t*)&dummy, 1, 100);
  HAL_GPIO_WritePin(SPI6_CSS_GPIO_Port, SPI6_CSS_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(SPI6_CSS_GPIO_Port, SPI6_CSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi6, (uint8_t*)&cmd_read, (uint8_t*)&dummy, 1, 100);
  HAL_GPIO_WritePin(SPI6_CSS_GPIO_Port, SPI6_CSS_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(SPI6_CSS_GPIO_Port, SPI6_CSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi6, (uint8_t*)&cmd_read, (uint8_t*)&dummy, 1, 100);
  HAL_GPIO_WritePin(SPI6_CSS_GPIO_Port, SPI6_CSS_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  spi6_tx_buf[0] = 0xFFFF;
  SCB_CleanDCache_by_Addr((uint32_t*)spi6_tx_buf, sizeof(spi6_tx_buf));
  enc_rot_C._last_raw_pos = (float)(dummy & 0x3FFF) * (360.0f / 16384.0f);
  if (enc_rot_C._last_raw_pos > 180.0f) enc_rot_C._turns = -1;

  /*
    _     _____ ____        ____                       
   | |   | ____|  _ \ ___  |  _ \  __ _ _ __   ___ ___ 
   | |   |  _| | | | / __| | | | |/ _` | '_ \ / __/ _ \
   | |___| |___| |_| \__ \ | |_| | (_| | | | | (_|  __/
   |_____|_____|____/|___/ |____/ \__,_|_| |_|\___\___|
                                                       
  */

  HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
  HAL_Delay(500);
  HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
  HAL_Delay(500);
  HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
  HAL_Delay(500);

  machine_state = RUN;

  axis_X._target_pos = 0.0f;
  axis_X._target_vel = 0.0f;
  axis_Y._target_pos = -0.0f;
  axis_Y._target_vel = 0.0f;
  axis_A1._target = 20.0f;
  axis_A2._target = 20.0f;
  axis_C._target = 10.0f;
  axis_Z1._target = 0.0f;
  axis_Z2._target = 0.0f;

  // let's start bitches
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_Delay(10);
  HAL_GPIO_WritePin(EN_STEPPERS_GPIO_Port, EN_STEPPERS_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char serial_buf[128];
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    
    __disable_irq();
    // uint16_t raw_y = spi2_rx_buf[0];
    float pos_y = enc_lin_Y._converted_value;
    float pos_z = enc_rot_Z._converted_value;
    float pos_a = enc_rot_A._converted_value;
    float psos_c = enc_rot_C._converted_value;
    __enable_irq();

    // char err_y = (raw_y & 0x4000) ? 'E' : 'O';

  sprintf(serial_buf, "pos_y: %d | pos_z: %d | pos_a: %d | pos_c: %d | z target: %d\r\n", (int)pos_y, (int)pos_z, (int)pos_a, (int)psos_c, (int)axis_Z1._target);
    // sprintf(serial_buf, "RAW Y[0]: 0x%04X (%c) | Pos Y: %d | LIN Y: %d\r\n", raw_y, err_y, (int)pos_y, (int)pos_lin);
    
    HAL_UART_Transmit(&huart1, (uint8_t*)serial_buf, strlen(serial_buf), 10);
    HAL_Delay(100);
  
    

  }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
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
