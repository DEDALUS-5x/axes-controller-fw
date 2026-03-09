/*

  _   _                 _ _               
 | | | | __ _ _ __   __| | | ___ _ __ ___ 
 | |_| |/ _` | '_ \ / _` | |/ _ \ '__/ __|
 |  _  | (_| | | | | (_| | |  __/ |  \__ \
 |_| |_|\__,_|_| |_|\__,_|_|\___|_|  |___/
                                          

*/

#include "handlers.h"
#include <math.h>


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        const float dt = 0.0001f;

        // spi1 daisy chain + invalidare cahce
        SCB_InvalidateDCache_by_Addr((uint32_t*)spi1_rx_buf, sizeof(spi1_rx_buf));
        
        // DMA buffer: [0]=EncX, [1]=EncY1, [2]=EncY2
        enc_rot_X._converted_value = (float)(spi1_rx_buf[0] & 0x3FFF) * (360.0f / 16384.0f);
        enc_rot_Y1._converted_value = (float)(spi1_rx_buf[1] & 0x3FFF) * (360.0f / 16384.0f);
        enc_rot_Y2._converted_value = (float)(spi1_rx_buf[2] & 0x3FFF) * (360.0f / 16384.0f);

        // spi4 read
        current_axis_idx = 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // CS x
        HAL_SPI_Receive_DMA(&hspi4, (uint8_t*)spi4_single_buf, 1);

        PID_Compute(&axis_X, dt);
        PID_Compute(&axis_Y1, dt);
        PID_Compute(&axis_Y2, dt);

        motor_command(&axis_X, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2);
        motor_command(&axis_Y1, &htim1, TIM_CHANNEL_3, TIM_CHANNEL_4);
        motor_command(&axis_Y2, &htim8, TIM_CHANNEL_1, TIM_CHANNEL_2);
    }
}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI4) {

    SCB_InvalidateDCache_by_Addr((uint32_t*)spi4_single_buf, sizeof(spi4_single_buf));
      
    current_values[current_axis_idx] = (float)spi4_single_buf[0] * (5.0f / 4096.0f);

    // CS high of the read one
    if (current_axis_idx == 0) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

    else if (current_axis_idx == 1) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);

    else if (current_axis_idx == 2) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

    current_values[current_axis_idx] = (float)spi4_single_buf[0] * (5.0f / 4096.0f);

    current_axis_idx++;

    // next CS
    if (current_axis_idx == 1) {

      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // y1
      HAL_SPI_Receive_DMA(&hspi4, (uint8_t*)spi4_single_buf, 1);
    } else if (current_axis_idx == 2) {

      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // y2
      HAL_SPI_Receive_DMA(&hspi4, (uint8_t*)spi4_single_buf, 1);
    }
  }
}
