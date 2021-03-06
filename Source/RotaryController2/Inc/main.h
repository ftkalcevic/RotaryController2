/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern void SetDisplaySPI();
extern void SetEEPROMSPI();
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern CRC_HandleTypeDef hcrc;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_LED_ONBOARD_Pin GPIO_PIN_13
#define GPIO_LED_ONBOARD_GPIO_Port GPIOC
#define GPIO_DISPLAY_CS_Pin GPIO_PIN_15
#define GPIO_DISPLAY_CS_GPIO_Port GPIOC
#define GPIO_ENCODER2_A_Pin GPIO_PIN_0
#define GPIO_ENCODER2_A_GPIO_Port GPIOA
#define GPIO_ENCODER2_B_Pin GPIO_PIN_1
#define GPIO_ENCODER2_B_GPIO_Port GPIOA
#define GPIO_ENCODER2_Z_Pin GPIO_PIN_2
#define GPIO_ENCODER2_Z_GPIO_Port GPIOA
#define GPIO_LED_RUNBUTTON_Pin GPIO_PIN_3
#define GPIO_LED_RUNBUTTON_GPIO_Port GPIOA
#define GPIO_RUNBUTTON_Pin GPIO_PIN_4
#define GPIO_RUNBUTTON_GPIO_Port GPIOA
#define GPIO_DISPLAY_SPI_SCK_Pin GPIO_PIN_5
#define GPIO_DISPLAY_SPI_SCK_GPIO_Port GPIOA
#define GPIO_DISPLAY_SPI_MISO_Pin GPIO_PIN_6
#define GPIO_DISPLAY_SPI_MISO_GPIO_Port GPIOA
#define GPIO_DISPLAY_SPI_MOSI_Pin GPIO_PIN_7
#define GPIO_DISPLAY_SPI_MOSI_GPIO_Port GPIOA
#define GPIO_BUZZER_Pin GPIO_PIN_0
#define GPIO_BUZZER_GPIO_Port GPIOB
#define GPIO_BACKLIGHT_IN2_Pin GPIO_PIN_1
#define GPIO_BACKLIGHT_IN2_GPIO_Port GPIOB
#define GPIO_BOOT1_Pin GPIO_PIN_2
#define GPIO_BOOT1_GPIO_Port GPIOB
#define GPIO_BACKLIGHT_IN1_Pin GPIO_PIN_10
#define GPIO_BACKLIGHT_IN1_GPIO_Port GPIOB
#define GPIO_EEPROM_CS_Pin GPIO_PIN_11
#define GPIO_EEPROM_CS_GPIO_Port GPIOB
#define GPIO_KEYPAD_C_Pin GPIO_PIN_12
#define GPIO_KEYPAD_C_GPIO_Port GPIOB
#define GPIO_KEYPAD_D_Pin GPIO_PIN_13
#define GPIO_KEYPAD_D_GPIO_Port GPIOB
#define GPIO_KEYPAD_4_Pin GPIO_PIN_14
#define GPIO_KEYPAD_4_GPIO_Port GPIOB
#define GPIO_KEYPAD_3_Pin GPIO_PIN_15
#define GPIO_KEYPAD_3_GPIO_Port GPIOB
#define GPIO_ENCODER1_A_Pin GPIO_PIN_8
#define GPIO_ENCODER1_A_GPIO_Port GPIOA
#define GPIO_ENCODER1_B_Pin GPIO_PIN_9
#define GPIO_ENCODER1_B_GPIO_Port GPIOA
#define GPIO_ENCODER1_Z_Pin GPIO_PIN_10
#define GPIO_ENCODER1_Z_GPIO_Port GPIOA
#define GPIO_ENCODER_OUT_Z_Pin GPIO_PIN_11
#define GPIO_ENCODER_OUT_Z_GPIO_Port GPIOA
#define GPIO_ENCODER_OUT_B_Pin GPIO_PIN_12
#define GPIO_ENCODER_OUT_B_GPIO_Port GPIOA
#define GPIO_ENCODER_OUT_A_Pin GPIO_PIN_15
#define GPIO_ENCODER_OUT_A_GPIO_Port GPIOA
#define GPIO_STEPPER_ENABLE_Pin GPIO_PIN_3
#define GPIO_STEPPER_ENABLE_GPIO_Port GPIOB
#define GPIO_STEPPER_STEP_Pin GPIO_PIN_4
#define GPIO_STEPPER_STEP_GPIO_Port GPIOB
#define GPIO_STEPPER_DIR_Pin GPIO_PIN_5
#define GPIO_STEPPER_DIR_GPIO_Port GPIOB
#define GPIO_KEYPAD_A_Pin GPIO_PIN_6
#define GPIO_KEYPAD_A_GPIO_Port GPIOB
#define GPIO_KEYPAD_B_Pin GPIO_PIN_7
#define GPIO_KEYPAD_B_GPIO_Port GPIOB
#define GPIO_KEYPAD_1_Pin GPIO_PIN_8
#define GPIO_KEYPAD_1_GPIO_Port GPIOB
#define GPIO_KEYPAD_2_Pin GPIO_PIN_9
#define GPIO_KEYPAD_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
