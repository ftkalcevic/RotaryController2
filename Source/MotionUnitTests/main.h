#pragma once

typedef enum
{
	GPIO_PIN_RESET = 0U,
	GPIO_PIN_SET
} GPIO_PinState;

struct TIM_HandleTypeDef {};
extern TIM_HandleTypeDef htim2;


extern void __disable_irq();
extern void __enable_irq();
extern void enableTimerInterupt(bool state);

#define	__HAL_TIM_DISABLE(htim)
#define	__HAL_TIM_GET_COUNTER(htim2)	0
#define	__HAL_TIM_ENABLE(htim)
#define	__HAL_TIM_SET_COUNTER(htim2, n)
#define	HAL_GPIO_WritePin(Port, Pin, state)
#define	__HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE)	enableTimerInterupt(true);
#define	__HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE)	enableTimerInterupt(false);
#define	__HAL_TIM_DISABLE(htim)



#define GPIO_LED_ONBOARD_Pin 1
#define GPIO_LED_ONBOARD_GPIO_Port 1
#define GPIO_DISPLAY_CS_Pin 1
#define GPIO_DISPLAY_CS_GPIO_Port 1
#define GPIO_ENCODER2_A_Pin 1
#define GPIO_ENCODER2_A_GPIO_Port 1
#define GPIO_ENCODER2_B_Pin 1
#define GPIO_ENCODER2_B_GPIO_Port 1
#define GPIO_ENCODER2_Z_Pin 1
#define GPIO_ENCODER2_Z_GPIO_Port 1
#define GPIO_LED_RUNBUTTON_Pin 1
#define GPIO_LED_RUNBUTTON_GPIO_Port 1
#define GPIO_RUNBUTTON_Pin 1
#define GPIO_RUNBUTTON_GPIO_Port 1
#define GPIO_DISPLAY_SPI_SCK_Pin 1
#define GPIO_DISPLAY_SPI_SCK_GPIO_Port 1
#define GPIO_DISPLAY_SPI_MISO_Pin 1
#define GPIO_DISPLAY_SPI_MISO_GPIO_Port 1
#define GPIO_DISPLAY_SPI_MOSI_Pin 1
#define GPIO_DISPLAY_SPI_MOSI_GPIO_Port 1
#define GPIO_BUZZER_Pin 1
#define GPIO_BUZZER_GPIO_Port 1
#define GPIO_BACKLIGHT_IN1_Pin 1
#define GPIO_BACKLIGHT_IN1_GPIO_Port 1
#define GPIO_BOOT1_Pin 1
#define GPIO_BOOT1_GPIO_Port 1
#define GPIO_BACKLIGHT_IN2_Pin 1
#define GPIO_BACKLIGHT_IN2_GPIO_Port 1
#define GPIO_EEPROM_CS_Pin 1
#define GPIO_EEPROM_CS_GPIO_Port 1
#define GPIO_KEYPAD_4_Pin 1
#define GPIO_KEYPAD_4_GPIO_Port 1
#define GPIO_KEYPAD_3_Pin 1
#define GPIO_KEYPAD_3_GPIO_Port 1
#define GPIO_KEYPAD_2_Pin 1
#define GPIO_KEYPAD_2_GPIO_Port 1
#define GPIO_KEYPAD_1_Pin 1
#define GPIO_KEYPAD_1_GPIO_Port 1
#define GPIO_ENCODER1_A_Pin 1
#define GPIO_ENCODER1_A_GPIO_Port 1
#define GPIO_ENCODER1_B_Pin 1
#define GPIO_ENCODER1_B_GPIO_Port 1
#define GPIO_ENCODER1_Z_Pin 1
#define GPIO_ENCODER1_Z_GPIO_Port 1
#define GPIO_ENCODER_OUT_Z_Pin 1
#define GPIO_ENCODER_OUT_Z_GPIO_Port 1
#define GPIO_ENCODER_OUT_B_Pin 1
#define GPIO_ENCODER_OUT_B_GPIO_Port 1
#define GPIO_ENCODER_OUT_A_Pin 1
#define GPIO_ENCODER_OUT_A_GPIO_Port 1
#define GPIO_STEPPER_DIR_Pin 1
#define GPIO_STEPPER_DIR_GPIO_Port 1
#define GPIO_STEPPER_STEP_Pin 1
#define GPIO_STEPPER_STEP_GPIO_Port 1
#define GPIO_STEPPER_ENABLE_Pin 1
#define GPIO_STEPPER_ENABLE_GPIO_Port 1
#define GPIO_KEYPAD_D_Pin 1
#define GPIO_KEYPAD_D_GPIO_Port 1
#define GPIO_KEYPAD_C_Pin 1
#define GPIO_KEYPAD_C_GPIO_Port 1
#define GPIO_KEYPAD_B_Pin 1
#define GPIO_KEYPAD_B_GPIO_Port 1
#define GPIO_KEYPAD_A_Pin 1
#define GPIO_KEYPAD_A_GPIO_Port 1

#define countof(x)		(sizeof(x)/sizeof((x)[0]))
#define sign(x)			((x)<0 ? -1 : 1)

