
#include "main.h"
#include <string.h>
#include <stdlib.h> 

#include "rotarycontroller.h"
//#include "display_vfd_M0420SD.h"
#include "display_oled_NHD_US2066.h"
#include "display.h"
#include "keys.h"
#include "buzzer.h"
#include "motion.h"

static DisplayOLEDNHDUS2066 displayDevice;
static Display<4,20> display(&displayDevice);
static Keys keys;
static Buzzer buzzer;
static Motion motion(&htim3);
static uint8_t activeDevice;

#define MAX_DEVICES		2
#define MAX_DIVISIONS	12

const char * const Name[MAX_DEVICES] = { "Rotary Table", "ER32 4th Axis" };

enum Mode: uint8_t
{
	emSplash,
	emJog,
	emDivision,
	emContinuous,
	emSynchronised,
	emSetup,
	//emTest,
	emMaxModes
};

enum Units : uint8_t
{
	Degrees = 0,
	Steps = 1
};

struct Eeprom
{
	struct Config
	{
		uint8_t SelectedDevice;
		struct DeviceConfig
		{
			uint32_t MotorStepsPerRevolution;		// steps per motor revolution (200*10 for gecko)
			uint32_t DeviceGearRatio;				// device gear ratio (5:1 ER32 axis, 90:1 rotary table) Dividend/Divisor?
			uint32_t Backlash;						// steps
			bool ReverseDirection;					// false=normal, true=reverse
			uint32_t SlowVelocityMax;				// steps/s
			uint32_t SlowAcceleration;				// steps/s/s
			uint32_t SlowSFM;						// Used to set Slow Max Vel - not available for "fast" - fast is jog like
			uint32_t SlowRadius;					// mm
			uint32_t FastVelocityMax;				// steps/s
			uint32_t FastAcceleration;				// steps/s/s
			bool SlowSpeedMode;						// true = slow, false = fast
			uint8_t DivisionCount;					// number of divisions in division mode (up to MAX_DIVISIONS)
			uint32_t Divisions[MAX_DIVISIONS];		
			Units DivisionsUnits;					// step or degrees
			Units LastUnits;						// "last units" - degrees, steps?
			uint32_t ContinuousSpeed;				// last continuous speed
		} DeviceConfig[MAX_DEVICES];
	
	} Config;
	
	uint32_t crc;
};

static_assert(((sizeof(Eeprom::Config) / 4) * 4) == sizeof(Eeprom::Config), "eeprom not multiple of 4 bytes");
static Eeprom eeprom;



static void ReadEeprom()
{
	// read eeprom
	// check crc
	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)&(eeprom.Config), sizeof(eeprom.Config)/4 );

	// if crc fails, use defaults
	if(crc != eeprom.crc)	
	{
		memset(&eeprom, 0, sizeof(eeprom));
		eeprom.Config.SelectedDevice = 0;
		eeprom.Config.DeviceConfig[0].MotorStepsPerRevolution = 200*10;			// steps per motor revolution (200*10 for gecko)
		eeprom.Config.DeviceConfig[0].DeviceGearRatio = 90;						// device gear ratio (5:1 ER32 axis, 90:1 rotary table) Dividend/Divisor?
		eeprom.Config.DeviceConfig[0].Backlash = 20;							// steps
		eeprom.Config.DeviceConfig[0].ReverseDirection = false;					// false=normal, true=reverse
		eeprom.Config.DeviceConfig[0].SlowVelocityMax = 1666;					// steps/s - stepper 50 rpm = 200*10 * 50 / 60 
		eeprom.Config.DeviceConfig[0].SlowAcceleration = 1666;					// steps/s/s
		eeprom.Config.DeviceConfig[0].SlowSFM = 0;								// Used to set Slow Max Vel - not available for "fast" - fast is jog like
		eeprom.Config.DeviceConfig[0].SlowRadius = 0;							// mm
		eeprom.Config.DeviceConfig[0].FastVelocityMax = 1333;					// steps/s - stepper 40 rpm = 200*40 * 10 / 60 
		eeprom.Config.DeviceConfig[0].FastAcceleration = 500;					// steps/s/s
		eeprom.Config.DeviceConfig[0].SlowSpeedMode = true;						// true = slow, false = fast
		eeprom.Config.DeviceConfig[0].DivisionCount = 0;						// number of divisions in division mode (up to MAX_DIVISIONS)
		//eeprom.Config.DeviceConfig[0].Divisions[MAX_DIVISIONS];		
		eeprom.Config.DeviceConfig[0].DivisionsUnits = Units::Steps;			// step or degrees
		eeprom.Config.DeviceConfig[0].LastUnits = Units::Steps;					// degrees, steps
		eeprom.Config.DeviceConfig[0].ContinuousSpeed = 500;					// last continuous speed
		
		eeprom.Config.DeviceConfig[1].MotorStepsPerRevolution = 200*10;			// steps per motor revolution (200*10 for gecko)
		eeprom.Config.DeviceConfig[1].DeviceGearRatio = 5;						// device gear ratio (5:1 ER32 axis, 90:1 rotary table) Dividend/Divisor?
		eeprom.Config.DeviceConfig[1].Backlash = 5;								// steps
		eeprom.Config.DeviceConfig[1].ReverseDirection = false;					// false=normal, true=reverse
		eeprom.Config.DeviceConfig[1].SlowVelocityMax = 333;					// steps/s - stepper 10 rpm = 200*10 * 10 / 60 
		eeprom.Config.DeviceConfig[1].SlowAcceleration = 333;					// steps/s/s
		eeprom.Config.DeviceConfig[1].SlowSFM = 0;								// Used to set Slow Max Vel - not available for "fast" - fast is jog like
		eeprom.Config.DeviceConfig[1].SlowRadius = 0;							// mm
		eeprom.Config.DeviceConfig[1].FastVelocityMax = 1333;					// steps/s - stepper 40 rpm = 200*40 * 10 / 60 
		eeprom.Config.DeviceConfig[1].FastAcceleration = 500;					// steps/s/s
		eeprom.Config.DeviceConfig[1].SlowSpeedMode = true;						// true = slow, false = fast
		eeprom.Config.DeviceConfig[1].DivisionCount = 0;						// number of divisions in division mode (up to MAX_DIVISIONS)
		//eeprom.Config.DeviceConfig[1].Divisions[MAX_DIVISIONS];		
		eeprom.Config.DeviceConfig[1].DivisionsUnits = Units::Steps;			// step or degrees
		eeprom.Config.DeviceConfig[1].LastUnits = Units::Steps;					// degrees, steps
		eeprom.Config.DeviceConfig[1].ContinuousSpeed = 500;					// last continuous speed
	}
}

static void WriteEeprom()
{
	// calculate crc
	eeprom.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)&(eeprom.Config), sizeof(eeprom.Config)/4 );
	// write eeprom
}


RotaryController::RotaryController()
{
}

void RotaryController::Init()
{
	activeDevice = eeprom.Config.SelectedDevice;
		
	ReadEeprom();
	
	motion.SetMotorConfig(eeprom.Config.DeviceConfig[activeDevice].Backlash,
						  eeprom.Config.DeviceConfig[activeDevice].MotorStepsPerRevolution,
						  eeprom.Config.DeviceConfig[activeDevice].DeviceGearRatio,
						  eeprom.Config.DeviceConfig[activeDevice].SlowVelocityMax,
						  eeprom.Config.DeviceConfig[activeDevice].SlowAcceleration);
	
	display.Init();
}

void RotaryController::SetRunButton(bool on)
{
	HAL_GPIO_WritePin(GPIO_LED_RUNBUTTON_GPIO_Port, GPIO_LED_RUNBUTTON_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void RotaryController::SetBacklight(EBacklight bl)
{
	if (bl == Green)
	{
		HAL_GPIO_WritePin(GPIO_BACKLIGHT_IN1_GPIO_Port, GPIO_BACKLIGHT_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIO_BACKLIGHT_IN2_GPIO_Port, GPIO_BACKLIGHT_IN2_Pin, GPIO_PIN_SET);
	}
	else if (bl == Red)
	{
		HAL_GPIO_WritePin(GPIO_BACKLIGHT_IN2_GPIO_Port, GPIO_BACKLIGHT_IN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIO_BACKLIGHT_IN1_GPIO_Port, GPIO_BACKLIGHT_IN1_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIO_BACKLIGHT_IN1_GPIO_Port, GPIO_BACKLIGHT_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIO_BACKLIGHT_IN2_GPIO_Port, GPIO_BACKLIGHT_IN2_Pin, GPIO_PIN_RESET);
	}
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	motion.TimerInterruptHandler();
}

extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	displayDevice.TxCpltCallback();
}

void RotaryController::DoSplash()
{
	display.ClearScreen();
	
	//                12345678901234567890
	display.Text(0,0," Rotary Controller ");
	display.Text(0,1,"    Version 1.0");
	display.Update();
	
	for (;;)
	{
		uint32_t key = keys.ScanKeys();
		if (key != Keys::KEY_NONE)
			return;
	}
}

void RotaryController::DoSetup()
{
	display.ClearScreen();
	//                12345678901234567890
	display.Text(0,0,"Setup");
	display.Update();
	
	for (;;)
	{
		uint32_t key = keys.ScanKeys();
		if (key == Keys::KEY_MODE)
			break;		
	}
}

void RotaryController::DoJog()
{
	display.ClearScreen();
	//                12345678901234567890
	display.Text(0,0,"Jog");
	display.Text(19 - strlen(Name[eeprom.Config.SelectedDevice]), 0, Name[eeprom.Config.SelectedDevice]);
	display.Update();
	
	for (;;)
	{
		uint32_t key = keys.ScanKeys();
		if (key == Keys::KEY_MODE)
			break;		
	}
}


void RotaryController::Run()
{
	Mode mode = emSplash;
	
	while (true)
	{
		static uint32_t last_tick = 0;
		uint32_t tick = HAL_GetTick();
		
		switch (mode)
		{
			case emSplash:
				DoSplash();
				mode = emSetup;
				break;
			case emSetup:
				DoSetup();
				mode = emJog;
				break;
			case emJog:
				DoJog();
				mode = emDivision;
				break;
			case emDivision:
				mode = emContinuous;
				break;
			case emContinuous:
				mode = emSynchronised;
				break;
			case emSynchronised:
				mode = emSetup;
				break;
		}
//		if (tick != last_tick)
//		{
//			last_tick = tick;
//			keys.Update();
//			buzzer.Update();
//			
//			char buf[20];
//			div_t res;
//			res = div(tick,1000);
//			int ms = res.rem;
//			res = div(res.quot, 60);
//			int s = res.rem;
//			res = div(res.quot, 60);
//			int m = res.rem;
//			res = div(res.quot, 60);
//			int h = res.rem;
//			snprintf(buf, 20, "%d:%02d:%02d.%03d", h,m,s,ms);
//			
//			display.Text(0,1,buf);
//			display.Update();
//		}
//		
//		static uint32_t next_tick = 0;
//		if (tick >= next_tick)
//		{
//			HAL_GPIO_TogglePin(GPIO_LED_ONBOARD_GPIO_Port, GPIO_LED_ONBOARD_Pin);
//			next_tick = tick + 500;
//			
//			static int n = 0;
//			n++;
//			SetBacklight((EBacklight)(n % 3));
//			//buzzer.Buzz(15);
//			static bool on = false;
//			SetRunButton(on);
//			on = !on;
//		}
//		
//		if (tick == 300)
//		{
//			static bool done = false;
//			if (!done)
//			{
//				motion.MoveDegrees( 5000UL );
//				motion.UpdateMotorTravel();
//				done = true;
//			}
//		}
	}
}





/*

Keyboard - matrix 
- every ms, 
	- poll each line and return 16 states
	- also the run key
- then debounce	
- doesn't need to be interrupts				
 
Display
- Synchronous.  
- We can use dma for block transfers.  It is probably easiest to export the whole screen at once. (4x20 characters).  We keep a screen buffer.

Keypad Backlight
- Only support red/green/off

Motor
- code for motion
- code for synchronous motion with spindle encoder

Encoder inputs
- a/b/z to watch for errors/missed steps
- configure timers

Encoder outputs
- runs with the stepper - consumer of encoder needs to know the steps per revolution.






Device Selection
	- rotary table
	- 4th axis
Device specific config
	- Name (Is this in the source? Add new device for new source)
	- steps per motor revolution (200*10 for gecko)
	- device gear ratio (5:1 ER32 axis, 90:1 rotary table) Dividend/Divisor?
	- backlash
	- direction
	- Slow Max Velocity
	- Slow Acc
	- Slow SFM    } Used to set Slow Max Vel - not available for "fast" - fast is jog like
	- Slow Radius }
	- Fast Max Velocity
	- Fast Acc 
	- Speed Mode (last slow/fast speed)
	- DivisionCount - number of divisions in division mode (up to MAX_DIVISIONS)
	- Divisions - array of MAX_DIVISIONS
	- Divisions units - step or degrees
	- JogUnits - should this be general "last units" - degrees, steps?
	- Continuous Speed - last continuous speed
- Mode - 
	- Jog
	- Division
	- Degrees
	- Continuous
	- Synchronous
	- Setup
	- Test







*/