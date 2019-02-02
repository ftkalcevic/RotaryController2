#pragma  once

//#define GPIO_DISPLAY_SPI_SCK_Pin GPIO_PIN_5
//#define GPIO_DISPLAY_SPI_SCK_GPIO_Port GPIOA
//#define GPIO_DISPLAY_CS_Pin GPIO_PIN_6
//#define GPIO_DISPLAY_CS_GPIO_Port GPIOA
//#define GPIO_DISPLAY_SPI_MOSI_Pin GPIO_PIN_7
//#define GPIO_DISPLAY_SPI_MOSI_GPIO_Port GPIOA


#include "main.h"
#include "common.h"
#include <string.h>

class DisplayOLEDNHDUS2066
{
	enum CMDS: uint8_t
	{
		CMD_CLEAR_DISPLAY		= 0b00000001,
		CMD_CURSOR_HOME			= 0b00000010,
		CMD_ENTRY_MODE_SET		= 0b00000100,
		CMD_DISPLAY_CONTROL		= 0b00001000,
		CMD_FUNCTION_SET		= 0b00100000,
		CMD_SETDDRAM_ADDR		= 0b10000000,
		EXTENSION_REGISTER_RE	= 0b00000010,
		EXTENSION_REGISTER_IS	= 0b00000001,
		FUNCTION_SELECTION_A	= 0b01110001,
		FUNCTION_SELECTION_B	= 0b01110010,
		CMD_OLED_COMMANDSET	    = 0b01111000,
		OLED_CMD_SETCONTRAST	= 0b10000001,
	};
	
	enum BITS : uint8_t 
	{				// Send LSB first
		START		= 0b00011111,	// 0b11111000,
		READ		= 0b00100000,	// 0b00000100,
		WRITE		= 0b00000000,	// 0b00000000,
		REGISTER	= 0b00000000,	// 0b00000000,
		MEMORY		= 0b01000000,	// 0b00000010,
	};

	const uint32_t TIMEOUT = 5;	// 5ms
	uint8_t fs1,fs2;
	
	void SendCMD(uint8_t cmd)
	{
		uint8_t data[3];
		data[0] = START | WRITE | REGISTER;
		data[1] = cmd & 0x0F;
		data[2] = cmd >> 4;

		
		HAL_GPIO_WritePin(GPIO_DISPLAY_CS_GPIO_Port, GPIO_DISPLAY_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, data, countof(data), TIMEOUT);
		HAL_GPIO_WritePin(GPIO_DISPLAY_CS_GPIO_Port, GPIO_DISPLAY_CS_Pin, GPIO_PIN_SET);
	}
	
	void SendData(uint8_t mem)
	{
		uint8_t data[3];
		data[0] = START | WRITE | MEMORY;
		data[1] = mem & 0x0f;
		data[2] = mem >> 4;
		
		HAL_GPIO_WritePin(GPIO_DISPLAY_CS_GPIO_Port, GPIO_DISPLAY_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit_IT(&hspi1, data, countof(data));
		while (hspi1.State != HAL_SPI_STATE_READY)
			continue;
		HAL_GPIO_WritePin(GPIO_DISPLAY_CS_GPIO_Port, GPIO_DISPLAY_CS_Pin, GPIO_PIN_SET);
	}
	
	void SendData(uint8_t *mem, uint8_t len)
	{
		uint8_t data[1+2*len];
		data[0] = START | WRITE | MEMORY;
		for (int i = 0; i < len; i++)
		{
			data[1+i*2] = mem[i] & 0x0f;
			data[2+i*2] = mem[i] >> 4;
		}
		
		HAL_GPIO_WritePin(GPIO_DISPLAY_CS_GPIO_Port, GPIO_DISPLAY_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit_IT(&hspi1, data, countof(data));
		while (hspi1.State != HAL_SPI_STATE_READY)
			continue;
		HAL_GPIO_WritePin(GPIO_DISPLAY_CS_GPIO_Port, GPIO_DISPLAY_CS_Pin, GPIO_PIN_SET);
	}
	
public:
	const uint8_t ADDR_LINE1 = 0x00;
	const uint8_t ADDR_LINE2 = 0x20;
	const uint8_t ADDR_LINE3 = 0x40;
	const uint8_t ADDR_LINE4 = 0x60;
	
	DisplayOLEDNHDUS2066()
	{
		fs1 = 0;
		fs2 = 0;
	}
	
	void Init()
	{
		// Short delay to wait for display to wake
		HAL_Delay(15);	// 15ms
		
		HAL_GPIO_WritePin(GPIO_DISPLAY_CS_GPIO_Port, GPIO_DISPLAY_CS_Pin, GPIO_PIN_SET);

		SendCMD(0x2A);  //function set (extended command set)
		SendCMD(0x71);  //function selection A, disable internal Vdd regualtor
		SendData(0x00);
		SendCMD(0x28);  //function set (fundamental command set)
		SendCMD(0x08);  //display off, cursor off, blink off
		SendCMD(0x2A);  //function set (extended command set)
		SendCMD(0x79);  //OLED command set enabled
		SendCMD(0xD5);  //set display clock divide ratio/oscillator frequency
		SendCMD(0x70);  //set display clock divide ratio/oscillator frequency
		SendCMD(0x78);  //OLED command set disabled
		SendCMD(0x09);  //extended function set (4-lines)
		SendCMD(0x06);  //COM SEG direction
		SendCMD(0x72);  //function selection B,
		SendData(0x00);     //ROM CGRAM selection
		SendCMD(0x2A);  //function set (extended command set)
		SendCMD(0x79);  //OLED command set enabled
		SendCMD(0xDA);  //set SEG pins hardware configuration
		SendCMD(0x10);  //set SEG pins ... NOTE: When using NHD-0216AW-XB3 or NHD_0216MW_XB3 change to (0x00)
		SendCMD(0xDC);  //function selection C
		SendCMD(0x00);  //function selection C
		SendCMD(0x81);  //set contrast control
		SendCMD(0x7F);  //set contrast control
		SendCMD(0xD9);  //set phase length
		SendCMD(0xF1);  //set phase length
		SendCMD(0xDB);  //set VCOMH deselect level
		SendCMD(0x40);  //set VCOMH deselect level
		SendCMD(0x78);  //OLED command set disabled
		SendCMD(0x28);  //function set (fundamental command set)
		SendCMD(0x01);  //clear display
		SendCMD(0x80);  //set DDRAM address to 0x00
		SendCMD(0x0C);  //display ON
	  
		HAL_Delay(100);
		
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
	
	void FunctionSet(bool multipleLines, bool doubleHeight, bool blinkEnable, bool reverseDisplay)
	{
		fs1 = CMD_FUNCTION_SET | (multipleLines ? 0x8 : 0) | (doubleHeight ? 0x4 : 0);
		fs2 = CMD_FUNCTION_SET | (multipleLines ? 0x8 : 0) | (blinkEnable ? 0x4 : 0) | (reverseDisplay ? 1 : 0);
		SendCMD(fs2 | EXTENSION_REGISTER_RE);
		SendCMD(fs1);
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
	
	void RamWrite(uint8_t *data, uint8_t len)
	{
		SendData(data,len);
	}
	
	void SelectCharacterROM(uint8_t characterNumber, uint8_t characterROM )
	{
		SendCMD(fs2 | EXTENSION_REGISTER_RE);
		SendCMD(FUNCTION_SELECTION_B);
		uint8_t data = (characterNumber & 0x3) | ((characterROM & 0x3) << 2);
		SendData(data);
		SendCMD(fs1);
	}
	
	void SetContrast(uint8_t c)
	{
		SendCMD(fs2 | EXTENSION_REGISTER_RE);
		SendCMD(CMD_OLED_COMMANDSET | 0x1);
		SendCMD(OLED_CMD_SETCONTRAST);
		SendCMD(c);
		SendCMD(CMD_OLED_COMMANDSET);
		SendCMD(fs1);
	}
	
	void SendText(char const *s, uint8_t len)
	{
		SendData((uint8_t *)s, len);
	}
	
	void SendText(char const *s)
	{
		SendText(s, strlen(s));
	}

	void DisplayBuffer(char *mem, uint8_t len)
	{
		while (hspi1.State != HAL_SPI_STATE_READY)
			continue;
		
		uint8_t cmd = (CMD_SETDDRAM_ADDR | (0 & 0b01111111));
		mem[0] = START | WRITE | REGISTER;
		mem[1] = cmd & 0x0F;
		mem[2] = cmd >> 4;
		mem[3] = START | WRITE | MEMORY;
		
		HAL_GPIO_WritePin(GPIO_DISPLAY_CS_GPIO_Port, GPIO_DISPLAY_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)mem, len);
	}
	
	void TxCpltCallback()
	{
		HAL_GPIO_WritePin(GPIO_DISPLAY_CS_GPIO_Port, GPIO_DISPLAY_CS_Pin, GPIO_PIN_SET);
	}

};