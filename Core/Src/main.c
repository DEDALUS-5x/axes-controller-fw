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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "types.h"
#include "ctrl.h"

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

uint16_t spi2_rx_buf[1] __attribute__((aligned(32))); // Encoder Rotativo (SPI2)
uint16_t spi4_rx_buf[1] __attribute__((aligned(32))); // Corrente Allegro (SPI4)

Encoder enc_rot_X = {0};
Encoder enc_lin_X = {0};
Axis axis_X;

float current_sense_X = 0.0f;

/* USER CODE END PV */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
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
  /* USER CODE BEGIN 2 */

  axis_X._feedback = &enc_rot_X; // rot per il loop di velocità
  axis_X._pwm_register = &TIM1->CCR1;

  PID_init(&axis_X._pid, 1.2f, 0.01f, 0.05f, 1000.0f);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // enc lineare

  HAL_SPI_Receive_DMA(&hspi2, (uint8_t*)spi2_rx_buf, 1); // encoder rotativo
  HAL_SPI_Receive_DMA(&hspi4, (uint8_t*)spi4_rx_buf, 1); // sensore allegro

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // pwm per ibt4
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_Base_Start_IT(&htim6);

  HAL_GPIO_WritePin(SPI4_CSS_X_GPIO_Port, SPI4_CSS_X_Pin, GPIO_PIN_RESET); // Attiva Allegro
  HAL_GPIO_WritePin(SPI2_CSS_GPIO_Port, SPI2_CSS_Pin, GPIO_PIN_RESET); // Attiva Encoder

  HAL_GPIO_WritePin(EN_STEPPERS_GPIO_Port, EN_STEPPERS_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
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

void motor_command(Axis *axis, TIM_HandleTypeDef *htim, uint32_t channel1, uint32_t channel2) {
    if (axis -> _pid._output > 0) {
        __HAL_TIM_SET_COMPARE(htim, channel1, (uint32_t)fabsf(axis -> _pid._output));
        __HAL_TIM_SET_COMPARE(htim, channel2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(htim, channel1, 0);
        __HAL_TIM_SET_COMPARE(htim, channel2, (uint32_t)fabsf(axis -> _pid._output));
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

  if (htim->Instance == TIM6) {
    const float dt = 0.0001f;

    enc_lin_X._raw_value = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    enc_lin_X._converted_value = (float)enc_lin_X._raw_value * 0.001f; 

    SCB_InvalidateDCache_by_Addr((uint32_t*)spi2_rx_buf, sizeof(spi2_rx_buf));
    enc_rot_X._raw_value = spi2_rx_buf[0] & 0x3FFF; // 14 bit AS5048A
    enc_rot_X._converted_value = (float)enc_rot_X._raw_value * (360.0f / 16384.0f);

    SCB_InvalidateDCache_by_Addr((uint32_t*)spi4_rx_buf, sizeof(spi4_rx_buf));
    current_sense_X = (float)spi4_rx_buf[0] * (5.0f / 4096.0f); // conv  volt/amp

    PID_Compute(&axis_X, dt);

    motor_command(&axis_X, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2);

    static uint32_t freq_divider = 0;
  if (++freq_divider >= 5000) { // debug
      HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
      freq_divider = 0;
}
  }


    //if (htim->Instance == TIM6)
    // {
        /* 1. ACQUISIZIONE DATI (SINCRONIZZAZIONE) */
        // Sull'H7, i dati scritti dal DMA in RAM non sono visti subito dalla CPU per via della Cache.
        // Dobbiamo invalidare la cache per le zone di memoria dove scrivono i DMA (Encoder, ADC).
        // SCB_InvalidateDCache_by_Addr((uint32_t*)dma_buffer_encoders, sizeof(dma_buffer_encoders));

        /* 2. CHIAMATA AI MODULI DI CALCOLO */
        // Esempio: Aggiornamento posizione assi
        // Update_Axis_Position(&Axis_X);
        
        /* 3. CALCOLO PID */
        // Esempio: calcolo PID per asse X
        // Axis_X.pid.output = Compute_PID(&Axis_X.pid, Axis_X.feedback->converted_value);
        
        /* 4. AGGIORNAMENTO ATTUATORI (PWM) */
        // *Axis_X.pwm_register = (uint32_t)Axis_X.pid.output;
//    }

}

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
