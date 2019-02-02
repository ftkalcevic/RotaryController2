#pragma  once

#include "main.h"

class Buzzer
{
	uint32_t count;
	
	void SetBuzzerOn()
	{
		HAL_GPIO_WritePin(GPIO_BUZZER_GPIO_Port, GPIO_BUZZER_Pin, GPIO_PIN_SET);
	}
	void SetBuzzerOff()
	{
		HAL_GPIO_WritePin(GPIO_BUZZER_GPIO_Port, GPIO_BUZZER_Pin, GPIO_PIN_RESET);
	}
		
public:
	Buzzer()
	{
		count = 0;
	}
	
	void Buzz(uint16_t ms)
	{
		count += ms;
		SetBuzzerOn();
	}

	void Update()
	{
		if (count)
		{
			count--;
			if (!count)
				SetBuzzerOff();
		}
	}
};