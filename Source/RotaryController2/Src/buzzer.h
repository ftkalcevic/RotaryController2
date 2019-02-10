#pragma  once

#include "main.h"

class Buzzer
{
	uint32_t count;
	uint32_t last_tick;
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
		last_tick = 0;
	}
	
	void Buzz(uint16_t ms)
	{
		count += ms;
		SetBuzzerOn();
	}

	void Alarm()
	{
		Buzz(500);
	}
	
	void Beep()
	{
		Buzz(150);
	}
	
	void Tick()
	{
		Buzz(5);
	}
	
	void Update()
	{
		if (count)
		{
			uint32_t tick = HAL_GetTick();
			if (tick == last_tick)
				return;
			last_tick = tick;
		
			count--;
			if (!count)
				SetBuzzerOff();
		}
	}
};