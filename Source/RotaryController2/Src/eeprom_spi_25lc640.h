#pragma  once

#include "main.h"
#include "common.h"
#include "assert.h"

class EepromSPI25lc640
{
	enum EEPROMCmds
	{
		READ = 0b00000011,
		WRITE= 0b00000010,
		WREN = 0b00000110,
		WRDI = 0b00000100,
		RDSR = 0b00000101,
		WRSR = 0b00000001,
	};
	
	const uint32_t EEPROM_WIP = 0b00000001;
	
public:
	EepromSPI25lc640()
	{
	}

	void WriteEeprom(uint16_t address, uint8_t *data, uint16_t len)
	{
		assert_param(address == 0);
		
		SetEEPROMSPI();	// reset SPI for eeprom
		
		for (int i = 0; i < len; i += 32)
		{
			// 32 bytes at a time (here we are assuming address starts on at 0)
			uint32_t datalen = 32;
			if (datalen + i > len)
				datalen  = len - i;

			// Enable write
			uint8_t buf1[1] = { EEPROMCmds::WREN };
			HAL_GPIO_WritePin(GPIO_EEPROM_CS_GPIO_Port, GPIO_EEPROM_CS_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,buf1,sizeof(buf1),1000);
			HAL_GPIO_WritePin(GPIO_EEPROM_CS_GPIO_Port, GPIO_EEPROM_CS_Pin, GPIO_PIN_SET);
	
		
			// Write address and data
			uint8_t buf2[3] = { EEPROMCmds::WRITE, (uint8_t)(i >> 8), (uint8_t)(i & 0xFF) };
			HAL_GPIO_WritePin(GPIO_EEPROM_CS_GPIO_Port, GPIO_EEPROM_CS_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1,buf2,sizeof(buf2),1000);
			HAL_SPI_Transmit(&hspi1,data,datalen,1000);
			data += datalen;
			HAL_GPIO_WritePin(GPIO_EEPROM_CS_GPIO_Port, GPIO_EEPROM_CS_Pin, GPIO_PIN_SET);
		
			// Wait for write to complete
			bool busy = true;
			while (busy)
			{
				uint8_t buf3[1] = { EEPROMCmds::RDSR };
				HAL_GPIO_WritePin(GPIO_EEPROM_CS_GPIO_Port, GPIO_EEPROM_CS_Pin, GPIO_PIN_RESET);
				HAL_SPI_Transmit(&hspi1,buf3,sizeof(buf3),1000);
				HAL_SPI_Receive(&hspi1,buf3,sizeof(buf3),1000);
				HAL_GPIO_WritePin(GPIO_EEPROM_CS_GPIO_Port, GPIO_EEPROM_CS_Pin, GPIO_PIN_SET);
				busy = (buf3[0] & EEPROM_WIP) ? true : false;
			}
		}
	
		SetDisplaySPI();

	}


	void ReadEeprom(uint16_t address, uint8_t *buffer, uint16_t len)
	{
		assert_param(address == 0);
		SetEEPROMSPI();

		// read
		uint8_t buf[3] = { EEPROMCmds::READ, (uint8_t)(address>>8), (uint8_t)(address & 0xFF) };
		HAL_GPIO_WritePin(GPIO_EEPROM_CS_GPIO_Port, GPIO_EEPROM_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, buf, sizeof(buf), 1000);
		HAL_SPI_Receive(&hspi1, buffer, len, 1000);
		HAL_GPIO_WritePin(GPIO_EEPROM_CS_GPIO_Port, GPIO_EEPROM_CS_Pin, GPIO_PIN_SET);
		SetDisplaySPI();
	}

};