#pragma  once

#include "main.h"
#include "common.h"

class Keys
{
	const int DEBOUNCE_COUNTER = 20;	// 20ms
	const int FIRST_KEY_TIMEOUT = 600;	// 600ms
	const int SUBSEQUENT_KEY_TIMEOUT = 100;	// 100ms
	
	uint16_t rows[4];
	uint16_t cols[4];
	uint32_t pressed;
	uint32_t debounceCounter;
	uint32_t autoRepeatCounter;
	uint32_t last_tick;
	
	void SetRowOutput(uint16_t pin)
	{
		HAL_GPIO_WritePin(GPIO_KEYPAD_A_GPIO_Port, pin, GPIO_PIN_RESET);
		
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIO_KEYPAD_A_GPIO_Port, &GPIO_InitStruct);		
	}
	
	void SetRowInput(uint16_t pin)
	{
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIO_KEYPAD_A_GPIO_Port, &GPIO_InitStruct);
	}

public:
	enum KeyCodes
	{
		KEY_NONE    = 0,
		KEY_1		= 0x00001,
		KEY_2		= 0x00010,
		KEY_3		= 0x00100,
		KEY_4		= 0x01000,
		KEY_5		= 0x00002,
		KEY_6		= 0x00020,
		KEY_7		= 0x00200,
		KEY_8		= 0x02000,
		KEY_9		= 0x00008,
		KEY_10		= 0x00080,
		KEY_11		= 0x00800,
		KEY_12		= 0x08000,
		KEY_13		= 0x00004,
		KEY_14		= 0x00040,
		KEY_15		= 0x00400,
		KEY_16		= 0x04000,
		KEY_STOP	= 0x10000,
		KEY_PRESSED = 0x80000,
		KEY_RELEASED= 0x00000,
		
		KEY_NUM_7	= KEY_1,
		KEY_M1000	= KEY_1,
		KEY_NUM_8	= KEY_2,
		KEY_M100	= KEY_2,
		KEY_NUM_9	= KEY_3,
		KEY_M10		= KEY_3,
		KEY_LEFT	= KEY_4,
		KEY_M1		= KEY_4,
		KEY_PREV	= KEY_4,
		KEY_NUM_4	= KEY_5,
		KEY_P1000	= KEY_5,
		KEY_NUM_5	= KEY_6,
		KEY_P100	= KEY_6,
		KEY_NUM_6	= KEY_7,
		KEY_P10		= KEY_7,
		KEY_RIGHT	= KEY_8,
		KEY_P1		= KEY_8,
		KEY_NEXT	= KEY_8,
		KEY_NUM_1	= KEY_9,
		KEY_NUM_2	= KEY_10,
		KEY_NUM_3	= KEY_11,
		KEY_SPEED	= KEY_11,
		KEY_MODE	= KEY_12,
		KEY_GOTO	= KEY_13,
		KEY_NUM_0	= KEY_14,
		KEY_DECIMAL	= KEY_15,
		KEY_UNITS	= KEY_15,
		KEY_OK		= KEY_16,
		KEY_GO		= KEY_16,
		
	};
	
	Keys()
	{
		// Luckily, all pins are on port GPIOB
		rows[0] = GPIO_KEYPAD_A_Pin;
		rows[1] = GPIO_KEYPAD_B_Pin;
		rows[2] = GPIO_KEYPAD_C_Pin;
		rows[3] = GPIO_KEYPAD_D_Pin;
		
		cols[0] = GPIO_KEYPAD_1_Pin;
		cols[1] = GPIO_KEYPAD_2_Pin;
		cols[2] = GPIO_KEYPAD_3_Pin;
		cols[3] = GPIO_KEYPAD_4_Pin;
		
		pressed = 0;
		debounceCounter = 0;
		autoRepeatCounter = 0;
		last_tick = 0;
	}
	
	uint32_t ScanKeys()
	{
		uint32_t ret = KEY_NONE;
		
		// update every 1ms
		uint32_t tick = HAL_GetTick();
		if (tick == last_tick)
			return ret;
		
		last_tick = tick;

		uint32_t keystate = 0;
		uint32_t mask = 1;
		
		// Poll the keys
		for(int r = 0 ; r < countof(rows) ; r++)
		{
			// Program the row pin to output and set it low
			SetRowOutput(rows[r]);
			for (int c = 0; c < countof(cols); c++)
			{
				// read the columns
				if ( HAL_GPIO_ReadPin(GPIO_KEYPAD_1_GPIO_Port,cols[c]) == GPIO_PIN_RESET )	// key pressed - pulled down
					keystate |= mask;
				mask <<= 1;
			}
			// Restore row pin to input (Hi-Z)
			SetRowInput(rows[r]);
		}
		
		// Other keys
		if(HAL_GPIO_ReadPin(GPIO_RUNBUTTON_GPIO_Port, GPIO_RUNBUTTON_Pin) == GPIO_PIN_RESET)	
			keystate |= mask;
		mask <<= 1;
		
		// Debounce the press and release.
		if(debounceCounter)
		{
			if (pressed & keystate)	// still down
			{
				debounceCounter--;
				if (debounceCounter == 0)
				{
					// debounced
					ret = pressed | KEY_PRESSED;
					autoRepeatCounter = FIRST_KEY_TIMEOUT;
				}
			}
			else
			{
				debounceCounter = 0;		
			}
		}
		else if(autoRepeatCounter)
		{
			if (pressed & keystate)	// still down
				{
					autoRepeatCounter--;
					if (autoRepeatCounter == 0)
					{
						ret = pressed | KEY_PRESSED;
						autoRepeatCounter = SUBSEQUENT_KEY_TIMEOUT;
					}
				}
			else
			{
				ret = pressed | KEY_RELEASED;
				pressed = 0;
				debounceCounter = 0;
				autoRepeatCounter = 0;
			}
		}
		else if ( keystate )
		{
			pressed = keystate;
			debounceCounter = DEBOUNCE_COUNTER;
		}
		return ret;
	}
};