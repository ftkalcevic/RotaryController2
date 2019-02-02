#pragma  once

#include "main.h"
#include "common.h"
#include "display_oled_NHD_US2066.h"
#include <string.h>

template <uint8_t ROWS, uint8_t COLS>
class Display
{
	DisplayOLEDNHDUS2066 *displayDevice;
	struct 
	{
		char header[4];
		char displayBuffer[ROWS][COLS][2];
	} data;
	
	public:
	Display(DisplayOLEDNHDUS2066 * device)
		: displayDevice(device)
	{
	}
	
	void Init()
	{
		displayDevice->Init();
		displayDevice->FunctionSet(true, false, false, false);
		displayDevice->DisplayControl(true, false, false);
		displayDevice->SelectCharacterROM(0,1);
		displayDevice->SetContrast(50);
		
		ClearScreen();
	}
	
	void Update()
	{
		//displayDevice->SetDDRamAddress(0);
		displayDevice->DisplayBuffer((char *)&data, sizeof(data));
	}
	
	void Text(uint8_t col, uint8_t row, const char *text, uint8_t len)
	{
		for (int i = 0; i < len; i++)
		{
			char c = text[i];
			data.displayBuffer[row][col+i][0] = c & 0x0f;
			data.displayBuffer[row][col+i][1] = c >> 4;
		}
	}
	
	void Text(uint8_t col, uint8_t row, const char *text)
	{
		Text(col, row, text, strlen(text));
	}
	
	void ClearScreen()
	{
		uint32_t c = ((' ' & 0x0f)  | ((' ' >> 4)<< 8)) << 16 | ((' ' & 0x0f)  | ((' ' >> 4)<< 8)); 
		uint32_t *p = (uint32_t *)data.displayBuffer;
		for (int i = 0; i < ROWS*COLS * 2 / 4; i++)
			p[i] = c;
			
//		for (int r = 0; r < ROWS; r++)
//			for (int c = 0; c < COLS; c++)
//			{
//				data.displayBuffer[r][c][0] = ' ' & 0x0f;
//				data.displayBuffer[r][c][1] = ' ' >> 4;
//			}
	}
};