#pragma  once

#define GPIO_DISPLAY_SPI_SCK_Pin GPIO_PIN_5
#define GPIO_DISPLAY_SPI_SCK_GPIO_Port GPIOA
#define GPIO_DISPLAY_CS_Pin GPIO_PIN_6
#define GPIO_DISPLAY_CS_GPIO_Port GPIOA
#define GPIO_DISPLAY_SPI_MOSI_Pin GPIO_PIN_7
#define GPIO_DISPLAY_SPI_MOSI_GPIO_Port GPIOA


#include "main.h"
extern SPI_HandleTypeDef hspi1;

class DisplayVFDM04020SD
{
	enum CMDS: uint8_t
	{
		CMD_CLEAR_DISPLAY	= 0b00000001,
		CMD_CURSOR_HOME		= 0b00000010,
		CMD_ENTRY_MODE_SET	= 0b00000100,
		CMD_DISPLAY_CONTROL	= 0b00001000,
		CMD_FUNCTION_SET    = 0b00100000,
		CMD_SETDDRAM_ADDR	= 0b10000000,
	};
	
	enum BITS : uint8_t 
	{ 
		START		= 0b11111000,
		READ		= 0b00000100,
		WRITE		= 0b00000000,
		REGISTER	= 0b00000000,
		MEMORY		= 0b00000010,
	};

	const uint32_t TIMEOUT = 5;	// 5ms
	void SendCMD(uint8_t cmd)
	{
		uint8_t data[2];
		data[1] = START | WRITE | REGISTER;
		data[0] = cmd;
		
		HAL_GPIO_WritePin(GPIO_DISPLAY_CS_GPIO_Port, GPIO_DISPLAY_CS_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_SPI_Transmit(&hspi1, data, 1, TIMEOUT);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIO_DISPLAY_CS_GPIO_Port, GPIO_DISPLAY_CS_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
	}
	
	void SendData(uint8_t mem)
	{
		uint8_t data[2];
		data[1] = START | WRITE | MEMORY;
		data[0] = mem;
		
		HAL_GPIO_WritePin(GPIO_DISPLAY_CS_GPIO_Port, GPIO_DISPLAY_CS_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_SPI_Transmit(&hspi1, data, 1, TIMEOUT);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIO_DISPLAY_CS_GPIO_Port, GPIO_DISPLAY_CS_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
	}
	
public:
	const uint8_t ADDR_LINE1 = 0x00;
	const uint8_t ADDR_LINE2 = 0x40;
	const uint8_t ADDR_LINE3 = 0x14;
	const uint8_t ADDR_LINE4 = 0x54;
	
	DisplayVFDM04020SD()
	{
	}
	
	void Init()
	{
		HAL_GPIO_WritePin(GPIO_DISPLAY_CS_GPIO_Port, GPIO_DISPLAY_CS_Pin, GPIO_PIN_SET);
	}
	
	void ClearDisplay()
	{
		SendCMD(CMD_CLEAR_DISPLAY);
	}
	
	void CursorHome()
	{
		SendCMD(CMD_CURSOR_HOME);
	}
	
	void EntryModeSet(bool increment, bool shift)
	{
		uint8_t b = CMD_ENTRY_MODE_SET | (increment ? 2 : 0) | (shift ? 1 : 0);
		SendCMD(b);
	}
	
	void DisplayControl(bool displayOn, bool cursorOn, bool blinkCursor)
	{
		uint8_t b = CMD_DISPLAY_CONTROL | (displayOn ? 4 : 0) | (cursorOn ? 2 : 0) | (blinkCursor ? 1 : 0);
		SendCMD(b);
	}
	
	void CursorDisplayShift()
	{
	}
	
	void FunctionSet(bool eightBit, bool multipleLines, uint8_t luminence)
	{
		uint8_t b = CMD_FUNCTION_SET | (eightBit ? 0x10 : 0) | (multipleLines ? 0x8 : 0) | (luminence & 0b11);
		SendCMD(b);
	}
	
	void SetCGRamAddress()
	{
	}
	
	void SetDDRamAddress(uint8_t addr)
	{
		SendCMD(CMD_SETDDRAM_ADDR | (addr & 0b01111111));
	}
	
	void RamWrite(uint8_t data)
	{
		SendData(data);
	}
	
	
};