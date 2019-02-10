
#include "main.h"
#include <string.h>
#include <stdlib.h> 

#include "rotarycontroller.h"
//#include "display_vfd_M0420SD.h"
#include "display_oled_NHD_US2066.h"
#include "eeprom_spi_25lc640.h"
#include "display.h"
#include "keys.h"
#include "buzzer.h"
#include "motion.h"

static DisplayOLEDNHDUS2066 displayDevice;
static Display<4,20> display(&displayDevice);
static Keys keys;
static Buzzer buzzer;
static Motion motion(&htim3);
static EepromSPI25lc640 eepromDevice;
static uint8_t activeDevice;

#define MAX_DEVICES		2
#define MAX_SEQUENCES	20

const char * const DeviceNames[MAX_DEVICES] = { "Rotary Table", "ER32 4th Axis" };
const char * const bool_names[] = { "False", "True" };
	
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
			uint32_t Divisions;						// the number of divisions around 360 degrees
			uint8_t SequenceCount;					// number of divisions in division mode (up to MAX_DIVISIONS)
			uint32_t Sequences[MAX_SEQUENCES];		
			Units SequencesUnits;					// step or degrees
			Units LastUnits;						// "last units" - degrees, steps?
			uint32_t ContinuousSpeed;				// last continuous speed
		} DeviceConfig[MAX_DEVICES];
	
	} Config;
	
	uint32_t crc;
};

static_assert(((sizeof(Eeprom::Config) / 4) * 4) == sizeof(Eeprom::Config), "eeprom not multiple of 4 bytes");
static Eeprom eeprom;

void RotaryController::WriteEEPROM()
{
	// calculate crc
	eeprom.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)&(eeprom.Config), sizeof(eeprom.Config)/4 );

	eepromDevice.WriteEeprom(0, (uint8_t*)&(eeprom), sizeof(eeprom));
}

void RotaryController::ReadEEPROM()
{
	eepromDevice.ReadEeprom(0, (uint8_t*)&(eeprom), sizeof(eeprom));
	
	// check crc
	uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)&(eeprom.Config), sizeof(eeprom.Config)/4 );

	// if crc fails, use defaults
	if(crc != eeprom.crc)	
	{
		eepromHasBeenReset = true;
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
		eeprom.Config.DeviceConfig[0].Divisions = 1;							// number of divisions in division mode
		eeprom.Config.DeviceConfig[0].SequenceCount = 0;						// number of Sequences in sequence mode (up to MAX_SEQUENCES)
		//eeprom.Config.DeviceConfig[0].Sequences[MAX_SEQUENCES];		
		eeprom.Config.DeviceConfig[0].SequencesUnits = Units::Degrees;			// step or degrees
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
		eeprom.Config.DeviceConfig[1].Divisions = 1;							// number of divisions in division mode
		eeprom.Config.DeviceConfig[1].SequenceCount = 0;						// number of Sequences in sequence mode (up to MAX_SEQUENCES)
		//eeprom.Config.DeviceConfig[0].Sequences[MAX_SEQUENCES];		
		eeprom.Config.DeviceConfig[1].SequencesUnits = Units::Degrees;			// step or degrees
		eeprom.Config.DeviceConfig[1].LastUnits = Units::Steps;					// degrees, steps
		eeprom.Config.DeviceConfig[1].ContinuousSpeed = 500;					// last continuous speed
		
		WriteEEPROM();
	}
	
	SetParameters();
}

RotaryController::RotaryController()
{
	eepromHasBeenReset = false;
}

void RotaryController::Init()
{
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
		uint32_t key = DoKeyScanEtc();
		if (key != Keys::KEY_NONE)
		{
			// We init here, to let the power supply stabilise
			ReadEEPROM();
			
			if (eepromHasBeenReset)
			{
				display.Text(0, 2, "Warning EEPROM Reset");
				display.Update();
				eepromHasBeenReset = false;
			}
			else
			{
				return;
			}
		}
	}
}


struct MenuItem setupMenu[] =
{	// 12345678901234567890
	{ "Device", &eeprom.Config.SelectedDevice, ENUM, 0, 1, DeviceNames, &RotaryController::DoSetupChangeDevice },
	{ "Motor Steps Per Rev.", &eeprom.Config.DeviceConfig[0].MotorStepsPerRevolution, UINT32, 1, 10000, NULL, NULL },
	{ "Gear Ratio", &eeprom.Config.DeviceConfig[0].DeviceGearRatio, UINT32, 1, 10000, NULL, NULL },
	{ "Backlash", &eeprom.Config.DeviceConfig[0].Backlash, UINT32, 1, 10000, NULL, NULL },
	{ "Reverse Motor", &eeprom.Config.DeviceConfig[0].ReverseDirection, BOOL, 0, 0, NULL, NULL },
	{ "Slow Max Vel", &eeprom.Config.DeviceConfig[0].SlowVelocityMax, UINT32, 1, 10000, NULL, NULL },
	{ "Slow Acc", &eeprom.Config.DeviceConfig[0].SlowAcceleration, UINT32, 1, 10000, NULL, NULL },
	{ "Slow SFM", &eeprom.Config.DeviceConfig[0].SlowSFM, UINT32, 1, 10000, NULL, NULL },
	{ "Slow Rad.", &eeprom.Config.DeviceConfig[0].SlowRadius, UINT32, 1, 10000, NULL, NULL },
	{ "Fast Max Vel", &eeprom.Config.DeviceConfig[0].FastVelocityMax, UINT32, 1, 10000, NULL, NULL },
	{ "Fast Acc", &eeprom.Config.DeviceConfig[0].FastAcceleration, UINT32, 1, 10000, NULL, NULL },
};

void RotaryController::DoSetupChangeDevice()
{
	int i = eeprom.Config.SelectedDevice;
	setupMenu[1].memAddr = &eeprom.Config.DeviceConfig[i].MotorStepsPerRevolution;
	setupMenu[2].memAddr = &eeprom.Config.DeviceConfig[i].DeviceGearRatio;
	setupMenu[3].memAddr = &eeprom.Config.DeviceConfig[i].Backlash;
	setupMenu[4].memAddr = &eeprom.Config.DeviceConfig[i].ReverseDirection;
	setupMenu[5].memAddr = &eeprom.Config.DeviceConfig[i].SlowVelocityMax;
	setupMenu[6].memAddr = &eeprom.Config.DeviceConfig[i].SlowAcceleration;
	setupMenu[7].memAddr = &eeprom.Config.DeviceConfig[i].SlowSFM;
	setupMenu[8].memAddr = &eeprom.Config.DeviceConfig[i].SlowRadius;
	setupMenu[9].memAddr = &eeprom.Config.DeviceConfig[i].FastVelocityMax;
	setupMenu[10].memAddr = &eeprom.Config.DeviceConfig[i].FastAcceleration;
}

void RotaryController::DisplaySetupValue(uint8_t col, uint8_t row, MenuItem &item)
{
	switch (item.type)
	{
		case ENUM:
			display.Text(col, row, item.enums[*((uint32_t *)(item.memAddr))]);
			break;
		case UINT32:
		{
			char buf[20];
			snprintf(buf, sizeof(buf) - 1, "%u", *((uint32_t *)(item.memAddr)));
			buf[sizeof(buf)-1] = 0;
			display.Text(col, row, buf);
			break;
		}
		case BOOL:
			display.Text(col, row, *((bool *)(item.memAddr)) ? "True" : "False");
			break;
	}
}

bool RotaryController::GetDigit( int nKey, uint8_t &nValue )
{
	if ( nKey == Keys::KEY_NUM_0_PRESSED ) nValue = 0;
	else if ( nKey == Keys::KEY_NUM_1_PRESSED ) nValue = 1;
	else if ( nKey == Keys::KEY_NUM_2_PRESSED ) nValue = 2;
	else if ( nKey == Keys::KEY_NUM_3_PRESSED ) nValue = 3;
	else if ( nKey == Keys::KEY_NUM_4_PRESSED ) nValue = 4;
	else if ( nKey == Keys::KEY_NUM_5_PRESSED ) nValue = 5;
	else if ( nKey == Keys::KEY_NUM_6_PRESSED ) nValue = 6;
	else if ( nKey == Keys::KEY_NUM_7_PRESSED ) nValue = 7;
	else if ( nKey == Keys::KEY_NUM_8_PRESSED ) nValue = 8;
	else if ( nKey == Keys::KEY_NUM_9_PRESSED ) nValue = 9;
	else return false;
	
	return true;
}


bool RotaryController::EditInt32( int32_t *n, uint8_t col, uint8_t row, int32_t min, uint32_t max )
{
	char buf[20];
	itoa( *n, buf, 10 );
	uint8_t nDigit = strlen(buf);
	display.ClearEOL(col, row);
	display.Text( col, row, buf );
	display.Update();
	display.SetCursorPos(col+nDigit, row);
	display.ShowCursor( true );
	bool bRet = false;
	for (;;)
	{
		uint8_t nValue;
		int key = DoKeyScanEtc();
		if (key == Keys::KEY_NONE)
			continue;
		
		if ( key == Keys::KEY_OK_PRESSED )
		{
			uint16_t nRet = 0;
			for ( uint8_t i = 0; i < nDigit; i++ )
				nRet = nRet * 10 + (buf[i]-'0');
			if ( nRet < min || nRet > max )
			{
				buzzer.Beep();
				char msg[21];
				snprintf( msg, sizeof(msg), "Must be from %u-%u", min, max );
				msg[sizeof(msg) - 1] = 0;
				display.Text(0, 3, msg);
				display.Update();
				display.SetCursorPos(col+nDigit, row);
			}
			else
			{
				*n = nRet;
				bRet = true;
				break;
			}
		}
		else if ( key == Keys::KEY_GOTO_PRESSED || key == Keys::KEY_MODE_PRESSED )
		{ 
			break;
		}
		else if ( key == Keys::KEY_LEFT_PRESSED )
		{
			if ( nDigit == 0 )
			{
				buzzer.Beep();
			}
			else
			{
				nDigit--;
				display.Text(col+nDigit, row, ' ');
				display.Update();
				display.SetCursorPos(col+nDigit, row);
			}
		}
		else if ( GetDigit( key, nValue ) )
		{
			if ( nDigit == 5 )
			{
				buzzer.Beep();
			}
			else
			{
				display.Text(col+nDigit, row, '0' + nValue);
				display.Update();
				buf[nDigit++] = '0' + nValue;
				display.SetCursorPos(col+nDigit, row);
			}
		}
	}
	display.ShowCursor(false);
	return bRet;
}

// for angles from 0.000 to 360.000.  Return value is an integer*1000
bool RotaryController::EditUInt32Frac( uint8_t col, uint8_t row, uint32_t *nValue)
{
	uint32_t angle = *nValue;
	while (angle >= 360000)
		angle -= 360000;
	
	div_t d = div(angle, 1000);
	uint32_t hi = d.quot;
	uint32_t lo = d.rem;
	
	
	char buf[12];	
	utoa( hi, buf, 10 );

	if ( lo != 0 )
	{
		strcat( buf, "." );

		if ( lo < 100 )
		{
			strcat( buf, "0" );
			if ( lo < 10 )
				strcat( buf, "0" );
		}

		utoa( lo, buf + strlen(buf), 10 );
	}


	uint8_t nDigit = strlen(buf);
	display.Text(col, row, buf);
	display.Update();
	display.SetCursorPos(col + nDigit, row);
	display.ShowCursor( true);
	bool bRet = false;
	for (;;)
	{
		uint8_t keyValue;
		uint32_t key = DoKeyScanEtc();
		if ( key == Keys::KEY_OK_PRESSED )
		{
			uint32_t nRet = 0;
			uint8_t i = 0;
			for ( ; i < nDigit; )
			{
				char c = buf[i++];
				if ( c == '.' )
					break;
				nRet = nRet * 10 + (c-'0');
			}

			if ( nRet >= 360 )
			{
				// bad
				buzzer.Beep();
			}
			else
			{
				nRet *= 1000;
				
				uint8_t n = 3;
				for ( ; i < nDigit && n != 0; i++, n-- )
				{
					uint16_t c = (uint16_t)buf[i] - '0';
					if ( n == 3 )
						nRet += c * (uint16_t)100;
					else if ( n == 2 )
						nRet += c * (uint16_t)10;
					else
						nRet += c;
				}

				*nValue = nRet;
				bRet = true;
				break;
			}
		}
		else if ( key == Keys::KEY_GOTO_PRESSED )
		{ 
			break;
		}
		else if ( key == Keys::KEY_LEFT_PRESSED )
		{
			if ( nDigit == 0 )
			{
				buzzer.Beep();
			}
			else
			{
				nDigit--;
				display.Text(col + nDigit, row, ' ');
				display.Update();
				display.SetCursorPos(col + nDigit, row);
			}
		}
		else if ( GetDigit( key, keyValue ) )
		{
			if ( nDigit == 7 )
			{
				buzzer.Beep();
			}
			else
			{
				buf[nDigit] = '0' + keyValue;
				display.Text( col+nDigit,row, '0' + keyValue );
				nDigit++;
				display.Update();
				display.SetCursorPos( col+nDigit,row );
			}
		}
		else if ( key == Keys::KEY_DECIMAL_PRESSED )
		{
			if ( nDigit == 7 )
			{
				buzzer.Beep();
			}
			else
			{
				// Check if already a .
				bool bOK = true;
				if (nDigit > 0)
				{
					uint8_t i = nDigit-1;
					do
					{
						if ( buf[i] == '.' )
						{
							buzzer.Beep();
							bOK = false;
							break;
						}
					}
					while (--i);
				}

				if ( bOK )
				{
					buf[nDigit] = '.';
					display.Text( col+nDigit, row, '.' );
					display.Update();
					nDigit++;
					display.SetCursorPos(col + nDigit, row);
				}
			}
		}
	}
	display.ShowCursor( false );
	return bRet;
}


bool RotaryController::EditEnum(uint32_t *n, const int row, uint32_t min, uint32_t max, const char *const *enums)
{
	uint32_t selected = *n;
	display.ClearRow( row );
	display.Text( 0, row, enums[selected] );
	display.Update();
	display.SetCursorPos(strlen(enums[selected]), row);
	display.ShowCursor( true );
	bool bRet = false;
	
	for (;;)
	{
		uint8_t nValue;
		int key = DoKeyScanEtc();
		if (key == Keys::KEY_NONE)
			continue;
		
		if ( key == Keys::KEY_OK_PRESSED )
		{
			*n = selected;
			bRet = true;
			break;
		}
		else if ( key == Keys::KEY_GOTO_PRESSED || key == Keys::KEY_MODE_PRESSED )
		{ 
			break;
		}
		else if ( key == Keys::KEY_LEFT_PRESSED || key == Keys::KEY_RIGHT_PRESSED )
		{
			if (key == Keys::KEY_LEFT_PRESSED)
			{
				if (selected == min)
					selected = max;
				else 
					selected--;
			}
			else
			{
				if (selected == max)
					selected = min;
				else 
					selected++;
			}
			display.ClearRow( row );
			display.Text( 0, row, enums[selected] );
			display.Update();
			display.SetCursorPos(strlen(enums[selected]), row);
		}
	}
	display.ShowCursor(false);
	return bRet;
}


bool RotaryController::EditItem(MenuItem &item)
{
	const int ROW = 2;
	switch (item.type)
	{
		case ENUM:
			return EditEnum((uint32_t*)item.memAddr, ROW, item.min, item.max, item.enums);
		case UINT32:
			return EditInt32((int32_t*)item.memAddr, 0, ROW, item.min, item.max);
		case BOOL:
			return EditEnum((uint32_t*)item.memAddr, ROW, 0, 1, bool_names);
	}
	return false;
}

void RotaryController::DrawSetup(MenuItem &item)
{
	display.ClearScreen();
	//                12345678901234567890
	display.Text(0, 0, "SETUP");
	display.Text(0, 1, item.name);
	DisplaySetupValue(0, 2, item);
	display.Update();
}

void RotaryController::DoSetup()
{
	DoSetupChangeDevice();
	
	int item = 0;
	DrawSetup(setupMenu[item]);
	
	for (;;)
	{
		uint32_t key = DoKeyScanEtc();
		if (key == Keys::KEY_MODE_PRESSED)
			break;
		else if (key == Keys::KEY_NEXT_PRESSED)
		{
			item++;
			if (item >= countof(setupMenu))
				item -= countof(setupMenu);
			DrawSetup(setupMenu[item]);
		}
		else if (key == Keys::KEY_PREV_PRESSED)
		{
			item--;
			if (item < 0 )
				item += countof(setupMenu);
			DrawSetup(setupMenu[item]);
		}
		else if (key == Keys::KEY_OK_PRESSED)
		{
			if (EditItem(setupMenu[item]))
			{
				WriteEEPROM();
				if (setupMenu[item].OnChangeEvent)
					(this->*setupMenu[item].OnChangeEvent)();
				SetParameters();
			}
			DrawSetup(setupMenu[item]);
		}
	}
}

void RotaryController::MakeDegrees( char *buffer, uint32_t buflen, int32_t nValue )
{
	while ( nValue < 0 )
		nValue += nTicksPerRotation; 

	while ( nValue >= nTicksPerRotation )
		nValue -= nTicksPerRotation;

	uint32_t degreeTicks = 360 * nValue;
	div_t t = div((int)degreeTicks, (int)nTicksPerRotation);
	
	int degrees = t.quot;
	int frac = t.rem;
	frac = (frac * 1000)/nTicksPerRotation;
	
	snprintf(buffer, buflen, "%3d.%03d", degrees, frac);
	buffer[buflen - 1] = 0;
	
	return;
}

void RotaryController::DisplayCoordinates()
{
	display.Text(0,2,"Stp:" );
	display.Text(0,3,"Deg:" );

	uint32_t nLastDisplayPosition = motion.MotorPosition();
	int32_t nDestination = motion.MotorDesitination();

	display.Int32Right(5,2,nLastDisplayPosition, 7, ' ' );
	display.Int32Right(13,2,nDestination, 7, ' ' );
	
	// Degrees
	char buf[20];
	MakeDegrees(buf, sizeof(buf), nLastDisplayPosition);
	display.Text(5, 3, buf);

	MakeDegrees(buf, sizeof(buf), nDestination);
	display.Text(13, 3, buf);
}

bool RotaryController::DrawJog(bool block)
{
	display.ClearScreen();
	display.Text(0,0,"JOG");
	
	display.Text(0, 1, movementUnits == Units::Degrees ? "Degrees" : "Steps");
	
	DisplayCoordinates();
	
	return display.Update(block);
}

static bool LeftRightKeyValue( uint32_t key, int32_t &nValue )
{
	switch ( key )
	{
		case Keys::KEY_M1: nValue = -1; break;
		case Keys::KEY_P1: nValue = 1; break;
		case Keys::KEY_M10: nValue = -10; break;
		case Keys::KEY_P10: nValue = 10; break;
		case Keys::KEY_M100: nValue = -100; break;
		case Keys::KEY_P100: nValue = 100; break;
		case Keys::KEY_M1000: nValue = -1000; break;
		case Keys::KEY_P1000: nValue = 1000; break;
		default: return false;
	}
	return true;
}

void RotaryController::Jog(int32_t distance)
{
	if (movementUnits == Units::Steps)
		motion.MoveSteps(distance);
	else
		motion.MoveDegrees(distance);
	motion.UpdateMotorTravel();
}



void RotaryController::ToggleSpeed()
{
}

void RotaryController::ToggleUnits()
{
	if (movementUnits == Units::Steps)
		movementUnits = Units::Degrees;
	else
		movementUnits = Units::Steps;
	eeprom.Config.DeviceConfig[activeDevice].LastUnits = movementUnits;
	WriteEEPROM();
}


void RotaryController::DoJog()
{
	const int DISPLAY_UPDATE_PERIOD = 50;	// 20 times per second
	DrawJog();
	bool redrawDisplay = true;
	int32_t lastPosition = 0;
	uint32_t next_redraw = 0;

	for (;;)
	{
		int32_t nMoveValue;
		if ( HAL_GetTick() > next_redraw )
		{
			if (redrawDisplay)
			{
				if ( DrawJog(false) )
					redrawDisplay = false;
			}
			next_redraw = DISPLAY_UPDATE_PERIOD + HAL_GetTick();
		}
		
		uint32_t key = DoKeyScanEtc();
		if (key == (Keys::KEY_MODE_PRESSED))
			break;
		
		else if (LeftRightKeyValue(key, nMoveValue))
		{
			SetRunButton(true);
			Jog(nMoveValue);
		}
		else if (key == Keys::KEY_ZERO_PRESSED)
		{
			if (motion.eState == Motion::eStopped)
			{
				motion.ResetMotorCounters();
				redrawDisplay = true;
			}
		}
		else if (key == Keys::KEY_SPEED_PRESSED)
		{
			ToggleSpeed();
		}		
		else if (key == Keys::KEY_UNITS_PRESSED)
		{
			ToggleUnits();
			redrawDisplay = true;
		}		
		else if (key == Keys::KEY_GOTO_PRESSED)
		{
			if (motion.eState != Motion::eStopped)
			{
				buzzer.Beep();
				continue;	
			}

			display.ClearRow(1);
			display.Text(0, 1, "Go To:");
			if (movementUnits == Units::Steps)
			{
				int32_t position = motion.MotorPosition();
				if (EditInt32(&position, 7, 1, -99999, 99999))
				{
					SetRunButton(true);
					motion.MoveToTicks(position);
					motion.UpdateMotorTravel();
				}
			}
			else
			{
				uint64_t degrees64 = motion.MotorPosition();
				degrees64 *= 360;
				degrees64 *= 1000; 
				degrees64 /= nTicksPerRotation;
				uint32_t degrees = (uint32_t)degrees64;
				if (EditUInt32Frac(7, 1, &degrees))
				{
					SetRunButton(true);
					motion.MoveToDegrees(degrees);
					motion.UpdateMotorTravel();
				}
			}
			redrawDisplay = true;
		}
		else if (key == Keys::KEY_STOP_PRESSED)
		{
			if ( motion.eState != Motion::eStopped )
			{
				motion.MotorStop();
				redrawDisplay = true;
			}
		}
		
		
		if (lastPosition != motion.MotorPosition())
		{
			redrawDisplay = true;
			if (motion.MotorPosition() == motion.MotorDesitination() || motion.eState == Motion::eStopped )
				SetRunButton(false);
			lastPosition = motion.MotorPosition();
		}
	}
}

bool RotaryController::DrawDivisions(bool block)
{
	display.ClearScreen();
	display.Text(0,0,"DIVISION");
	
	char buf[21];
	snprintf(buf, sizeof(buf), "Division: %d/%d", division+1, divisions);
	display.Text(0, 1, buf);
	
	DisplayCoordinates();
	
	return display.Update(block);
}


void RotaryController::DoDivisions()
{
	const int DISPLAY_UPDATE_PERIOD = 50;	// 20 times per second
	
	bool redrawDisplay = true;
	int32_t lastPosition = 0;
	uint32_t next_redraw = 0;

	bool move = false;
	division = 0;
	divisions = eeprom.Config.DeviceConfig[activeDevice].Divisions;
	for (;;)
	{
		int32_t nMoveValue;
		if ( HAL_GetTick() > next_redraw )
		{
			if (redrawDisplay)
			{
				if ( DrawDivisions(false) )
					redrawDisplay = false;
			}
			next_redraw = DISPLAY_UPDATE_PERIOD + HAL_GetTick();
		}
		
		uint32_t key = DoKeyScanEtc();
		if (key == (Keys::KEY_MODE_PRESSED))
			break;
		
		else if (key == Keys::KEY_OK_PRESSED && motion.eState == Motion::eStopped)
		{
			display.ClearRow(1);
			display.Text(0, 1, "Divisions:");
			EditInt32(&divisions, 11, 1, 1, 9999);
			if (divisions != eeprom.Config.DeviceConfig[activeDevice].Divisions)
			{
				eeprom.Config.DeviceConfig[activeDevice].Divisions = divisions;
				WriteEEPROM();
				if (division + 1 >= divisions)
					division = divisions - 1;
			}
			redrawDisplay = true;
		}
		else if (key == Keys::KEY_GOTO_PRESSED && motion.eState == Motion::eStopped)
		{
			display.ClearRow(1);
			display.Text(0, 1, "Division:");
			int32_t newDivision = division+1;
			EditInt32(&newDivision, 10, 1, 1, divisions);
			if (newDivision != division+1)
			{
				division = newDivision-1;
				move = true;
			}
			redrawDisplay = true;
		}
		else if (key == Keys::KEY_LEFT_PRESSED && motion.eState == Motion::eStopped )
		{
			if (division > 0)
				division--;
			else 
				division = divisions - 1;
			move = true;
		}
		else if (key == Keys::KEY_RIGHT_PRESSED && motion.eState == Motion::eStopped )
		{
			if (division < divisions - 1)
				division++;
			else 
				division = 0;
			move = true;
		}
		else if (key == Keys::KEY_ZERO_PRESSED)
		{
			if (motion.eState == Motion::eStopped)
			{
				motion.ResetMotorCounters();
				redrawDisplay = true;
			}
		}
		else if (key == Keys::KEY_SPEED_PRESSED)
		{
			ToggleSpeed();
		}		
		else if (key == Keys::KEY_STOP_PRESSED)
		{
			if ( motion.eState != Motion::eStopped )
			{
				motion.MotorStop();
				redrawDisplay = true;
			}
		}
		
		if (move)
		{
			uint64_t next = division;
			next *= nTicksPerRotation;
			next /= divisions;
			
			SetRunButton(true);
			motion.MoveToTicks(next);
			motion.UpdateMotorTravel();
			redrawDisplay = true;
			move = false;
		}
		
		if (lastPosition != motion.MotorPosition())
		{
			redrawDisplay = true;
			if (motion.MotorPosition() == motion.MotorDesitination() || motion.eState == Motion::eStopped)
			{
				SetRunButton(false);
				buzzer.Beep();
			}
			lastPosition = motion.MotorPosition();
		}
	}
}


uint32_t RotaryController::DoKeyScanEtc()
{
	buzzer.Update();	
	uint32_t k = keys.ScanKeys();
	if (k & Keys::KEY_PRESSED)
		buzzer.Tick(); 
	return k;
}


void RotaryController::SetParameters( void )
{
	activeDevice = eeprom.Config.SelectedDevice;
	
	// Set the parameters based on eeprom settings.
	nStepperDivisionsPerRotation = eeprom.Config.DeviceConfig[activeDevice].MotorStepsPerRevolution;		// Steps per rotation on stepper
	nGearRatio = eeprom.Config.DeviceConfig[activeDevice].DeviceGearRatio;									// Worm to table turn ratio
	nBacklash = eeprom.Config.DeviceConfig[activeDevice].Backlash;
	bMotorReverseDirection = eeprom.Config.DeviceConfig[activeDevice].ReverseDirection;

	movementUnits = eeprom.Config.DeviceConfig[activeDevice].LastUnits;
	
	// Calculate the number of ticks per table 360' rotation
	nTicksPerRotation = nStepperDivisionsPerRotation * nGearRatio;

	//SetSpeed();
		
	motion.SetMotorConfig(eeprom.Config.DeviceConfig[activeDevice].Backlash,
						  eeprom.Config.DeviceConfig[activeDevice].MotorStepsPerRevolution,
						  eeprom.Config.DeviceConfig[activeDevice].DeviceGearRatio,
						  eeprom.Config.DeviceConfig[activeDevice].SlowVelocityMax,
						  eeprom.Config.DeviceConfig[activeDevice].SlowAcceleration);
	
	
}



void RotaryController::Run()
{
	Mode mode = emSplash;
	
	SetBacklight(EBacklight::Green);
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
				DoDivisions();
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





TODO -
- Many functions can be executed when the stepper is running.

*/