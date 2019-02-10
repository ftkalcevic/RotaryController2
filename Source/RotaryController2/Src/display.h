#pragma  once

#include "main.h"
#include "common.h"
#include "display_oled_NHD_US2066.h"
#include <string.h>
#include <stdlib.h> 

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
	
	bool Update( bool block = true )
	{
		//displayDevice->SetDDRamAddress(0);
		return displayDevice->DisplayBuffer((char *)&data, sizeof(data), block);
	}
	
	void Text(uint8_t col, uint8_t row, char c)
	{
		if (row < ROWS && col < COLS)
		{
			data.displayBuffer[row][col][0] = c & 0x0f;
			data.displayBuffer[row][col][1] = c >> 4;	
		}
	}
	
	void Text(uint8_t col, uint8_t row, const char *text, uint8_t len)
	{
		for (int i = 0; i < len; i++)
			Text(col + i, row, text[i]);
	}
	
	void Text(uint8_t col, uint8_t row, const char *text)
	{
		Text(col, row, text, strlen(text));
	}
	
	void ClearScreen()
	{
		const uint32_t c = ((' ' & 0x0f)  | ((' ' >> 4)<< 8)) << 16 | ((' ' & 0x0f)  | ((' ' >> 4)<< 8)); 
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
	
	void ClearRow(uint8_t row)
	{
		const uint32_t c = ((' ' & 0x0f)  | ((' ' >> 4)<< 8)) << 16 | ((' ' & 0x0f)  | ((' ' >> 4)<< 8)); 
		uint32_t *p = (uint32_t *)(data.displayBuffer[row]);
		for (int i = 0; i < COLS * 2 / 4; i++)
			*(p++) = c;
	}
	
	void ClearEOL(uint8_t col, uint8_t row)
	{
		for ( int i = col; i < COLS; i++ )
			Text( i, row, ' ' );
	}
	
	void SetCursorPos(uint8_t col, uint8_t row)
	{
		displayDevice->SetDDRamAddress(row * 0x20 + col);
	}
	
	void ShowCursor(bool show)
	{
		if (show)
			displayDevice->DisplayControl(true, true, true);
		else
			displayDevice->DisplayControl(true, false, false);
	}
	
	void Int32Right(uint8_t col, uint8_t row, int32_t n, uint8_t width, char padChar)
	{
		char buf[15];
		itoa( n, buf, 10 );
		int8_t nPad = (int8_t)width - strlen( buf );
		while ( nPad > 0 )
		{
			Text( col, row, padChar );
			col++;
			nPad--;
		}
		Text( col, row, buf );
	}
	
};