/*
  SPIflash.cpp - SPI flash chip Arduino and chipKit library
  Copyright (C)2015-2019 Rinky-Dink Electronics, Henning Karlsen. All right reserved
  
  This library provides basic support for handling SPI flash memory chips. It 
  also supports a very simple, read-only file system that can be used for 
  storing text files and text (string) resource files.
  The file system also has some support methods for reading information about
  images and audio files.

  You can find the latest version of the library at 
  http://www.RinkyDinkElectronics.com/

  This library is free software; you can redistribute it and/or
  modify it under the terms of the CC BY-NC-SA 3.0 license.
  Please see the included documents for further information.

  Commercial use of this library requires you to buy a license that
  will allow commercial use. This includes using the library,
  modified or not, as a tool to sell products.

  The license applies to all part of the library including the 
  examples and tools supplied with the library.
*/

#include "SPIflash.h"

#define WRITETYPE_WORD	0x0001
#define WRITETYPE_CONT	0x0002

#if defined(__AVR__)
	#include "hardware/avr/HW_AVR.h"
#elif defined(__PIC32MX__)
	#include "hardware/pic32/HW_PIC32.h"
#elif defined(__arm__)
	#include "hardware/arm/HW_ARM.h"
#endif

struct _s_device
{
  uint8_t	_manufacturer_id;
  uint8_t	_type_id;
  uint8_t	_device_id;
  uint16_t  _size;			// in Mbits
  uint32_t	_pages;			// highest page number available
  char     *_manufacturer_name;
  char     *_device_type;
  char     *_device_name;
  uint8_t	_cmd_write;		// command for page write
  uint8_t	_flags;			
};

const _s_device _device[]=
{
  {0xBF, 0x25, 0x8C, 0x0002, 0x000003ff, (char*)"SST (Microchip)",	(char*)"SPI Serial Flash",	(char*)"SST25VF020B",	0xAD, WRITETYPE_WORD},
  {0xBF, 0x25, 0x8D, 0x0004, 0x000007ff, (char*)"SST (Microchip)",	(char*)"SPI Serial Flash",	(char*)"SST25VF040B",	0xAD, WRITETYPE_WORD},
  {0xBF, 0x25, 0x8E, 0x0008, 0x00000fff, (char*)"SST (Microchip)",	(char*)"SPI Serial Flash",	(char*)"SST25VF080B",	0xAD, WRITETYPE_WORD},
  {0xBF, 0x25, 0x41, 0x0010, 0x00001fff, (char*)"SST (Microchip)",	(char*)"SPI Serial Flash",	(char*)"SST25VF016B",	0xAD, WRITETYPE_WORD},
  {0xBF, 0x25, 0x4A, 0x0020, 0x00003fff, (char*)"SST (Microchip)",	(char*)"SPI Serial Flash",	(char*)"SST25VF032B",	0xAD, WRITETYPE_WORD},
  {0xBF, 0x25, 0x4B, 0x0040, 0x00007fff, (char*)"SST (Microchip)",	(char*)"SPI Serial Flash",	(char*)"SST25VF064C",	0x02, WRITETYPE_CONT},
  {0xEF, 0x40, 0x14, 0x0008, 0x00000fff, (char*)"Winbond",			(char*)"SPI Serial Flash",	(char*)"W25Q08BV",		0x02, WRITETYPE_CONT},
  {0xEF, 0x40, 0x15, 0x0010, 0x00001fff, (char*)"Winbond",			(char*)"SPI Serial Flash",	(char*)"W25Q16BV",		0x02, WRITETYPE_CONT},
  {0xEF, 0x40, 0x16, 0x0020, 0x00003fff, (char*)"Winbond",			(char*)"SPI Serial Flash",	(char*)"W25Q32BV",		0x02, WRITETYPE_CONT},
  {0xEF, 0x40, 0x17, 0x0040, 0x00007fff, (char*)"Winbond",			(char*)"SPI Serial Flash",	(char*)"W25Q64FV",		0x02, WRITETYPE_CONT},
  {0xEF, 0x40, 0x18, 0x0080, 0x0000ffff, (char*)"Winbond",			(char*)"SPI Serial Flash",	(char*)"W25Q128BV/FV",	0x02, WRITETYPE_CONT},
  {0xEF, 0x40, 0x19, 0x0100, 0x0001ffff, (char*)"Winbond",			(char*)"SPI Serial Flash",	(char*)"W25Q256FV",		0x02, WRITETYPE_CONT},
  {0xC2, 0x20, 0x15, 0x0010, 0x00001fff, (char*)"MXIC",				(char*)"SPI Serial Flash",	(char*)"MX25L1605D",	0x02, WRITETYPE_CONT},
  {0xC2, 0x20, 0x16, 0x0020, 0x00003fff, (char*)"MXIC",				(char*)"SPI Serial Flash",	(char*)"MX25L3205D",	0x02, WRITETYPE_CONT},
  {0xC2, 0x20, 0x17, 0x0040, 0x00007fff, (char*)"MXIC",				(char*)"SPI Serial Flash",	(char*)"MX25L6405D",	0x02, WRITETYPE_CONT}
};

/****************************************************************************************/
/* Public                                                                               */
/****************************************************************************************/

SPIflash::SPIflash()
{
	_HW_SPI = true;
	_pinCE = SS;
}

SPIflash::SPIflash(uint8_t CE)
{
	_HW_SPI = true;
	_pinCE	= CE;
}

SPIflash::SPIflash(uint8_t SI, uint8_t SO, uint8_t SCK, uint8_t CE)
{
	_HW_SPI = false;
	_pinSI	= SI;
	_pinSO	= SO;
	_pinSCK	= SCK;
	_pinCE	= CE;
}

void SPIflash::begin()
{
	if (!_HW_SPI)
	{
		pinMode(_pinSI, OUTPUT);
		pinMode(_pinSO, INPUT);
		pinMode(_pinSCK, OUTPUT);
		pinMode(_pinCE, OUTPUT);
	}
	else
		_SPIstart();

	for (int c=0; c<256; c++)
		buffer[c]=0;

	for (int c=0; c<MAX_FILES; c++)
		_fileinfo[c].fileid=0xFFFF;;
	//Serial.print("--begin--");
	_ID_Device();
	_restrictedAccess = 0xFF;
}

uint8_t SPIflash::readStatus()
{
	uint8_t status;

	digitalWrite(_pinCE, LOW);
	_writeByte(0x05);
	status=_readByte();
	digitalWrite(_pinCE, HIGH);

	return status;
}

void SPIflash::readPage(uint32_t page)
{
	if (page<=_max_pages)
	{
		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte((page & 0x0000FFFF)>>8);
		_writeByte(page & 0x000000FF);
		_writeByte(0x00);
		_writeByte(0x00);

		for (int c=0; c<256; c++)
			buffer[c]=_readByte();
		digitalWrite(_pinCE, HIGH);
	}
}

void SPIflash::writePage(uint32_t page)
{
	if (page<=_max_pages)
	{
		_setBlockProtection(0);

		digitalWrite(_pinCE, LOW);
		_writeByte(0x06);
		digitalWrite(_pinCE, HIGH);

		digitalWrite(_pinCE, LOW);
		_writeByte(_cmd_write);
		_writeByte((page & 0x0000FFFF)>>8);
		_writeByte(page & 0x000000FF);
		_writeByte(0x00);
		if (_flags & WRITETYPE_WORD)
		{
			_writeByte(buffer[0]);
			_writeByte(buffer[1]);
			digitalWrite(_pinCE, HIGH);
			waitForReady();
			for (int c=2; c<256; c+=2)
			{
				digitalWrite(_pinCE, LOW);
				_writeByte(0xAD);
				_writeByte(buffer[c]);
				_writeByte(buffer[c+1]);
				digitalWrite(_pinCE, HIGH);
				waitForReady();
			}
		}
		else
		{
			for (int c=0; c<256; c++)
				_writeByte(buffer[c]);
			digitalWrite(_pinCE, HIGH);
			waitForReady();
		}
		digitalWrite(_pinCE, LOW);
		_writeByte(0x04);
		digitalWrite(_pinCE, HIGH);
		_setBlockProtection(0x0f);
	}
}

void SPIflash::waitForReady()
{
	while ((readStatus() & 0x01) == 0x01) {};
}

void SPIflash::eraseChip()
{
	_setBlockProtection(0);

	digitalWrite(_pinCE, LOW);
	_writeByte(0x06);
	digitalWrite(_pinCE, HIGH);

	digitalWrite(_pinCE, LOW);
	_writeByte(0x60);
	digitalWrite(_pinCE, HIGH);

	waitForReady();
	_setBlockProtection(0x0f);
}

uint16_t SPIflash::fileOpen(uint16_t fileid)
{
	uint32_t	startpage;
	uint8_t		filehandle = 0xFF;
	uint8_t		c;
	uint32_t	tmp1, tmp2, tmp3, tmp4;

	if (fileid<MAX_FILEID)
	{
		for (c = 0; c<MAX_FILES; c++)
			if (_fileinfo[c].fileid == fileid)
				return ERR_FILE_ALREADY_OPEN;

		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(0x00);
		_writeByte(((fileid * 32) & 0x0000FFFF)>>8);
		_writeByte((fileid * 32) & 0x000000FF);
		_writeByte(0x00);

		startpage = (_readByte()<<8) + _readByte();
		digitalWrite(_pinCE, HIGH);
		if ((startpage & 0xFFFF) == 0xFFFF)
			return ERR_FILE_DOES_NOT_EXIST;

		c = 0;
		while ((c < MAX_FILES) and (filehandle == 0xFF))
		{
			if (_fileinfo[c].fileid == 0xFFFF)
			{
				filehandle = c;
				_fileinfo[filehandle].fileid = fileid;

				digitalWrite(_pinCE, LOW);		// Get filetype
				_writeByte(0x0B);
				_writeByte(0x00);
				_writeByte((((fileid * 32) + 2) & 0x0000FFFF)>>8);
				_writeByte(((fileid * 32) + 2) & 0x000000FF);
				_writeByte(0x00);

				_fileinfo[filehandle].filetype = _readByte();
				digitalWrite(_pinCE, HIGH);		// End filetype
				
				if (_fileinfo[filehandle].filetype == 3)
				{
					_fileinfo[filehandle].fileid = 0xFFFF;
					return ERR_FILETYPE_INCORRECT;
				}

				digitalWrite(_pinCE, LOW);		// Get filesize
				_writeByte(0x0B);
				_writeByte(0x00);
				_writeByte((((fileid * 32) + 4) & 0x0000FFFF)>>8);
				_writeByte(((fileid * 32) + 4) & 0x000000FF);
				_writeByte(0x00);

				tmp1 = _readByte();
				tmp2 = _readByte();
				tmp3 = _readByte();
				tmp4 = _readByte();
				_fileinfo[filehandle].filesize = (tmp1<<24) + (tmp2<<16) + (tmp3<<8) + tmp4;
				digitalWrite(_pinCE, HIGH);		// End filesize

				_fileinfo[filehandle].start = startpage * PAGE_SIZE;
				_fileinfo[filehandle].position = startpage * PAGE_SIZE;
			}
			else
				c++;
		}
		if (filehandle == 0xFF)
			return ERR_NO_AVAILABLE_HANDLES;
		else
			return filehandle;
	}
	else
		return ERR_FILE_DOES_NOT_EXIST;
}

uint16_t SPIflash::fileClose(uint8_t filehandle)
{
	if ((_fileinfo[filehandle].fileid == 0xFFFF) or (filehandle >= MAX_FILES))
		return ERR_FILE_NOT_OPEN;
	else
	{
		_fileinfo[filehandle].fileid = 0xFFFF;
		if (_restrictedAccess == filehandle)
			_restrictedAccess = 0xFF;
		return ERR_NO_ERROR;
	}
}

uint16_t SPIflash::restrictAccess(uint8_t filehandle)
{
	if (_fileinfo[filehandle].fileid == 0xFFFF)
	{
		_restrictedAccess = 0xFF;
		return ERR_FILE_NOT_OPEN;
	}
	else
	{
		_restrictedAccess = filehandle;
		return ERR_NO_ERROR;
	}
}

void SPIflash::unrestrictAccess()
{
	_restrictedAccess = 0xFF;
}

uint8_t	SPIflash::isRestricted()
{
	return _restrictedAccess;
}

uint16_t SPIflash::fileSeek(uint8_t filehandle, int32_t offset)
{
	if ((_restrictedAccess != 0xFF) and (_restrictedAccess != filehandle))
		return ERR_ACCESS_IS_RESTRICTED;

	if (_fileinfo[filehandle].fileid == 0xFFFF)
		return ERR_FILE_NOT_OPEN;
	else
	{
		if (offset == 0)
		{
			_fileinfo[filehandle].position = _fileinfo[filehandle].start;
			return ERR_NO_ERROR;
		}
		if (offset < 0)
		{
			if ((_fileinfo[filehandle].position + offset) < _fileinfo[filehandle].start)
				return ERR_SEEK_PAST_FILE_START;
		}
		else
		{
			if ((_fileinfo[filehandle].position + offset) > (_fileinfo[filehandle].position + _fileinfo[filehandle].filesize))
				return ERR_SEEK_PAST_FILE_END;
		}

		_fileinfo[filehandle].position += offset;
		return ERR_NO_ERROR;
	}
}

uint16_t SPIflash::fileRead(uint8_t filehandle)
{
	uint8_t data;

	if ((_restrictedAccess != 0xFF) and (_restrictedAccess != filehandle))
		return ERR_ACCESS_IS_RESTRICTED;

	if (_fileinfo[filehandle].fileid == 0xFFFF)
		return ERR_FILE_NOT_OPEN;
	else
	{
		if (_fileinfo[filehandle].position == uint32_t(_fileinfo[filehandle].start + _fileinfo[filehandle].filesize))
			return ERR_AT_EOF;

		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(((_fileinfo[filehandle].position) & 0x00FF0000)>>16);
		_writeByte(((_fileinfo[filehandle].position) & 0x0000FF00)>>8);
		_writeByte((_fileinfo[filehandle].position) & 0x000000FF);
		_writeByte(0x00);

		data=_readByte();
		digitalWrite(_pinCE, HIGH);

		_fileinfo[filehandle].position++;
		return data;
	}
}

uint16_t SPIflash::fileRead(uint8_t filehandle, char *buffer, uint16_t buflen)
{
	uint32_t bytesToRead;

	if ((_restrictedAccess != 0xFF) and (_restrictedAccess != filehandle))
		return ERR_ACCESS_IS_RESTRICTED;

	if (_fileinfo[filehandle].fileid == 0xFFFF)
		return ERR_FILE_NOT_OPEN;
	else
	{
		if (_fileinfo[filehandle].position == uint32_t(_fileinfo[filehandle].start + _fileinfo[filehandle].filesize))
		{
			buffer[0] = 0;
			return ERR_AT_EOF;
		}

		if (_fileinfo[filehandle].filetype == 2)
		{
			// Text file
			for (uint32_t c=0; c<buflen; c++)
				buffer[c] = 0;
			if ((_fileinfo[filehandle].position + (buflen - 1)) > (_fileinfo[filehandle].start + _fileinfo[filehandle].filesize))
				bytesToRead = (_fileinfo[filehandle].start + _fileinfo[filehandle].filesize) - _fileinfo[filehandle].position;
			else
				bytesToRead = buflen - 1;

			digitalWrite(_pinCE, LOW);
			_writeByte(0x0B);
			_writeByte(((_fileinfo[filehandle].position) & 0x00FF0000)>>16);
			_writeByte(((_fileinfo[filehandle].position) & 0x0000FF00)>>8);
			_writeByte((_fileinfo[filehandle].position) & 0x000000FF);
			_writeByte(0x00);

			for (uint32_t c=0; c<bytesToRead; c++)
				buffer[c]=_readByte();
			digitalWrite(_pinCE, HIGH);

			_fileinfo[filehandle].position += bytesToRead;
			return bytesToRead;
		}
		else
		{
			// Other files
			if ((_fileinfo[filehandle].position + buflen) > (_fileinfo[filehandle].start + _fileinfo[filehandle].filesize))
				bytesToRead = (_fileinfo[filehandle].start + _fileinfo[filehandle].filesize) - _fileinfo[filehandle].position;
			else
				bytesToRead = buflen;

			digitalWrite(_pinCE, LOW);
			_writeByte(0x0B);
			_writeByte(((_fileinfo[filehandle].position) & 0x00FF0000)>>16);
			_writeByte(((_fileinfo[filehandle].position) & 0x0000FF00)>>8);
			_writeByte((_fileinfo[filehandle].position) & 0x000000FF);
			_writeByte(0x00);

			for (uint32_t c=0; c<bytesToRead; c++)
				buffer[c]=_readByte();
			digitalWrite(_pinCE, HIGH);

			_fileinfo[filehandle].position += bytesToRead;
			return bytesToRead;
		}
	}
}

uint16_t SPIflash::fileReadLn(uint8_t filehandle, char *buffer, uint16_t buflen)
{
	uint16_t	bytesToRead;
	boolean		foundEOL = false;

	if ((_restrictedAccess != 0xFF) and (_restrictedAccess != filehandle))
		return ERR_ACCESS_IS_RESTRICTED;

	if (_fileinfo[filehandle].fileid == 0xFFFF)
		return ERR_FILE_NOT_OPEN;
	else
	{
		if (_fileinfo[filehandle].filetype == 2)	// Text files only
		{
			if (_fileinfo[filehandle].position == (_fileinfo[filehandle].start + _fileinfo[filehandle].filesize))
			{
				buffer[0] = 0;
				return ERR_AT_EOF;
			}

			for (uint32_t c=0; c<buflen; c++)
				buffer[c] = 0;
			if ((_fileinfo[filehandle].position + (buflen - 1)) > (_fileinfo[filehandle].start + _fileinfo[filehandle].filesize))
				bytesToRead = (_fileinfo[filehandle].start + _fileinfo[filehandle].filesize) - _fileinfo[filehandle].position;
			else
				bytesToRead = buflen - 1;

			digitalWrite(_pinCE, LOW);
			_writeByte(0x0B);
			_writeByte(((_fileinfo[filehandle].position) & 0x00FF0000)>>16);
			_writeByte(((_fileinfo[filehandle].position) & 0x0000FF00)>>8);
			_writeByte((_fileinfo[filehandle].position) & 0x000000FF);
			_writeByte(0x00);

			uint32_t c = 0;
			while ((c < bytesToRead) and (foundEOL == false))
			{
				buffer[c]=_readByte();
				if (buffer[c] == 0x0A)
				{
					foundEOL = true;
					buffer[c] = 0;
				}
				if (buffer[c] == 0x0D)
				{
					foundEOL = true;
					buffer[c] = 0;
					char temp = _readByte();
					if (temp == 0x0A)
						c++;
				}
				c++;
			}
			digitalWrite(_pinCE, HIGH);
			
			_fileinfo[filehandle].position += c;
			if (c == bytesToRead)
				return ERR_BUFFER_OVERFLOW;
			else
				return ERR_NO_ERROR;
		}
		else
			return ERR_FILETYPE_INCORRECT;
	}
}

uint16_t SPIflash::getFileType(uint16_t fileid)
{
	uint16_t	startpage = 0xFFFF;
	uint8_t		tmp;

	if (_restrictedAccess != 0xFF)
		return ERR_ACCESS_IS_RESTRICTED;

	if (fileid < MAX_FILEID)
	{
		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(0x00);
		_writeByte(((fileid * 32) & 0x0000FFFF)>>8);
		_writeByte((fileid * 32) & 0x000000FF);
		_writeByte(0x00);

		startpage = (_readByte()<<8) + _readByte();
		digitalWrite(_pinCE, HIGH);
	}
	else
		return ERR_FILE_DOES_NOT_EXIST;

	if (startpage == 0xFFFF)
		return ERR_FILE_DOES_NOT_EXIST;

	digitalWrite(_pinCE, LOW);
	_writeByte(0x0B);
	_writeByte(0x00);
	_writeByte((((fileid * 32) + 2) & 0x0000FFFF)>>8);
	_writeByte(((fileid * 32) + 2) & 0x000000FF);
	_writeByte(0x00);

	tmp = _readByte();
	digitalWrite(_pinCE, HIGH);

	return tmp;
}

uint32_t SPIflash::getFileSize(uint16_t fileid)
{
	uint16_t	startpage = 0xFFFF;
	uint32_t	tmp;
	uint32_t	tmp1, tmp2, tmp3, tmp4;

	if (_restrictedAccess != 0xFF)
		return ERROR_ACCESS_IS_RESTRICTED;

	if (fileid < MAX_FILEID)
	{
		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(0x00);
		_writeByte(((fileid * 32) & 0x0000FFFF)>>8);
		_writeByte((fileid * 32) & 0x000000FF);
		_writeByte(0x00);

		startpage = (_readByte()<<8) + _readByte();
		digitalWrite(_pinCE, HIGH);
	}
	else
		return ERROR_FILE_DOES_NOT_EXIST;

	if (startpage == 0xFFFF)
		return ERROR_FILE_DOES_NOT_EXIST;

	digitalWrite(_pinCE, LOW);
	_writeByte(0x0B);
	_writeByte(0x00);
	_writeByte((((fileid * 32) + 4) & 0x0000FFFF)>>8);
	_writeByte(((fileid * 32) + 4) & 0x000000FF);
	_writeByte(0x00);

	tmp1 = _readByte();
	tmp2 = _readByte();
	tmp3 = _readByte();
	tmp4 = _readByte();
	tmp = (tmp1<<24) + (tmp2<<16) + (tmp3<<8) + tmp4;
	digitalWrite(_pinCE, HIGH);

	return tmp;
}

uint16_t SPIflash::readFileNote(uint16_t fileid, char *buffer)
{
	uint16_t	startpage;
	uint8_t		tmp;

	if (_restrictedAccess != 0xFF)
		return ERR_ACCESS_IS_RESTRICTED;

	if (fileid < MAX_FILEID)
	{
		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(0x00);
		_writeByte(((fileid * 32) & 0x0000FFFF)>>8);
		_writeByte((fileid * 32) & 0x000000FF);
		_writeByte(0x00);

		startpage = (_readByte()<<8) + _readByte();
		digitalWrite(_pinCE, HIGH);
	}
	else
		return ERR_FILE_DOES_NOT_EXIST;

	if (startpage == 0xFFFF)
		return ERR_FILE_DOES_NOT_EXIST;

	digitalWrite(_pinCE, LOW);
	_writeByte(0x0B);
	_writeByte(0x00);
	_writeByte((((fileid * 32) + 16) & 0x0000FFFF)>>8);
	_writeByte(((fileid * 32) + 16) & 0x000000FF);
	_writeByte(0x00);

	for (int c=0; c<16; c++)
	{
		tmp = _readByte();
		if (tmp != 0xFF)
			buffer[c] = tmp;
		else
			buffer[c] = 0;
	}
	buffer[16] = 0;
	digitalWrite(_pinCE, HIGH);

	return ERR_NO_ERROR;
}

uint16_t SPIflash::getImageXSize(uint16_t fileid)
{
	uint16_t	startpage = 0xFFFF;
	uint8_t		tmp;
	uint16_t	size = 0;

	if (_restrictedAccess != 0xFF)
		return ERR_ACCESS_IS_RESTRICTED;

	if (fileid < MAX_FILEID)
	{
		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(0x00);
		_writeByte(((fileid * 32) & 0x0000FFFF)>>8);
		_writeByte((fileid * 32) & 0x000000FF);
		_writeByte(0x00);

		startpage = (_readByte()<<8) + _readByte();
		digitalWrite(_pinCE, HIGH);
	}
	else
		return ERR_FILE_DOES_NOT_EXIST;

	if (startpage == 0xFFFF)
		return ERR_FILE_DOES_NOT_EXIST;

	digitalWrite(_pinCE, LOW);
	_writeByte(0x0B);
	_writeByte(0x00);
	_writeByte((((fileid * 32) + 2) & 0x0000FFFF)>>8);
	_writeByte(((fileid * 32) + 2) & 0x000000FF);
	_writeByte(0x00);

	tmp = _readByte();
	digitalWrite(_pinCE, HIGH);

	if ((tmp >= 4) and (tmp <=6))
	{
		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(0x00);
		_writeByte((((fileid * 32) + 8) & 0x0000FFFF)>>8);
		_writeByte(((fileid * 32) + 8) & 0x000000FF);
		_writeByte(0x00);

		size = (_readByte()<<8) + _readByte();
		digitalWrite(_pinCE, HIGH);
		return size;
	}
	else
		return ERR_FILETYPE_INCORRECT;
}

uint16_t SPIflash::getImageYSize(uint16_t fileid)
{
	uint16_t	startpage = 0xFFFF;
	uint8_t		tmp;
	uint16_t	size = 0;

	if (_restrictedAccess != 0xFF)
		return ERR_ACCESS_IS_RESTRICTED;

	if (fileid < MAX_FILEID)
	{
		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(0x00);
		_writeByte(((fileid * 32) & 0x0000FFFF)>>8);
		_writeByte((fileid * 32) & 0x000000FF);
		_writeByte(0x00);

		startpage = (_readByte()<<8) + _readByte();
		digitalWrite(_pinCE, HIGH);
	}
	else
		return ERR_FILE_DOES_NOT_EXIST;

	if (startpage == 0xFFFF)
		return ERR_FILE_DOES_NOT_EXIST;

	digitalWrite(_pinCE, LOW);
	_writeByte(0x0B);
	_writeByte(0x00);
	_writeByte((((fileid * 32) + 2) & 0x0000FFFF)>>8);
	_writeByte(((fileid * 32) + 2) & 0x000000FF);
	_writeByte(0x00);

	tmp = _readByte();
	digitalWrite(_pinCE, HIGH);

	if ((tmp >= 4) and (tmp <=6))
	{
		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(0x00);
		_writeByte((((fileid * 32) + 10) & 0x0000FFFF)>>8);
		_writeByte(((fileid * 32) + 10) & 0x000000FF);
		_writeByte(0x00);

		size = (_readByte()<<8) + _readByte();
		digitalWrite(_pinCE, HIGH);
		return size;
	}
	else
		return ERR_FILETYPE_INCORRECT;
}

uint16_t	SPIflash::getAudioSamplerate(uint16_t fileid)
{
	uint16_t	startpage = 0xFFFF;
	uint8_t		tmp;
	uint16_t	rate = 0;

	if (_restrictedAccess != 0xFF)
		return ERR_ACCESS_IS_RESTRICTED;

	if (fileid < MAX_FILEID)
	{
		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(0x00);
		_writeByte(((fileid * 32) & 0x0000FFFF)>>8);
		_writeByte((fileid * 32) & 0x000000FF);
		_writeByte(0x00);

		startpage = (_readByte()<<8) + _readByte();
		digitalWrite(_pinCE, HIGH);
	}
	else
		return ERR_FILE_DOES_NOT_EXIST;

	if (startpage == 0xFFFF)
		return ERR_FILE_DOES_NOT_EXIST;

	digitalWrite(_pinCE, LOW);
	_writeByte(0x0B);
	_writeByte(0x00);
	_writeByte((((fileid * 32) + 2) & 0x0000FFFF)>>8);
	_writeByte(((fileid * 32) + 2) & 0x000000FF);
	_writeByte(0x00);

	tmp = _readByte();
	digitalWrite(_pinCE, HIGH);

	if (tmp == 7)
	{
		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(0x00);
		_writeByte((((fileid * 32) + 8) & 0x0000FFFF)>>8);
		_writeByte(((fileid * 32) + 8) & 0x000000FF);
		_writeByte(0x00);

		rate = (_readByte()<<8) + _readByte();
		digitalWrite(_pinCE, HIGH);
		return rate;
	}
	else
		return ERR_FILETYPE_INCORRECT;
}

uint16_t	SPIflash::getAudioBPS(uint16_t fileid)
{
	uint16_t	startpage = 0xFFFF;
	uint8_t		tmp;
	uint16_t	bps = 0;

	if (_restrictedAccess != 0xFF)
		return ERR_ACCESS_IS_RESTRICTED;

	if (fileid < MAX_FILEID)
	{
		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(0x00);
		_writeByte(((fileid * 32) & 0x0000FFFF)>>8);
		_writeByte((fileid * 32) & 0x000000FF);
		_writeByte(0x00);

		startpage = (_readByte()<<8) + _readByte();
		digitalWrite(_pinCE, HIGH);
	}
	else
		return ERR_FILE_DOES_NOT_EXIST;

	if (startpage == 0xFFFF)
		return ERR_FILE_DOES_NOT_EXIST;

	digitalWrite(_pinCE, LOW);
	_writeByte(0x0B);
	_writeByte(0x00);
	_writeByte((((fileid * 32) + 2) & 0x0000FFFF)>>8);
	_writeByte(((fileid * 32) + 2) & 0x000000FF);
	_writeByte(0x00);

	tmp = _readByte();
	digitalWrite(_pinCE, HIGH);

	if (tmp == 7)
	{
		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(0x00);
		_writeByte((((fileid * 32) + 10) & 0x0000FFFF)>>8);
		_writeByte(((fileid * 32) + 10) & 0x000000FF);
		_writeByte(0x00);

		bps = _readByte();
		digitalWrite(_pinCE, HIGH);
		return bps;
	}
	else
		return ERR_FILETYPE_INCORRECT;
}

uint16_t	SPIflash::getAudioChannels(uint16_t fileid)
{
	uint16_t	startpage = 0xFFFF;
	uint8_t		tmp;
	uint16_t	chan = 0;

	if (_restrictedAccess != 0xFF)
		return ERR_ACCESS_IS_RESTRICTED;

	if (fileid < MAX_FILEID)
	{
		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(0x00);
		_writeByte(((fileid * 32) & 0x0000FFFF)>>8);
		_writeByte((fileid * 32) & 0x000000FF);
		_writeByte(0x00);

		startpage = (_readByte()<<8) + _readByte();
		digitalWrite(_pinCE, HIGH);
	}
	else
		return ERR_FILE_DOES_NOT_EXIST;

	if (startpage == 0xFFFF)
		return ERR_FILE_DOES_NOT_EXIST;

	digitalWrite(_pinCE, LOW);
	_writeByte(0x0B);
	_writeByte(0x00);
	_writeByte((((fileid * 32) + 2) & 0x0000FFFF)>>8);
	_writeByte(((fileid * 32) + 2) & 0x000000FF);
	_writeByte(0x00);

	tmp = _readByte();
	digitalWrite(_pinCE, HIGH);

	if (tmp == 7)
	{
		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(0x00);
		_writeByte((((fileid * 32) + 11) & 0x0000FFFF)>>8);
		_writeByte(((fileid * 32) + 11) & 0x000000FF);
		_writeByte(0x00);

		chan = _readByte();
		digitalWrite(_pinCE, HIGH);
		return chan;
	}
	else
		return ERR_FILETYPE_INCORRECT;
}

uint16_t SPIflash::readTextResource(uint16_t fileid, uint16_t resid, char *buffer, uint16_t buflen)
{
	uint16_t	startpage;
	uint8_t		filetype;
	uint16_t	linecount;
	uint32_t	offset;
	uint16_t	length;
	boolean		overflow = false;

	if (_restrictedAccess != 0xFF)
		return ERR_ACCESS_IS_RESTRICTED;

	if (fileid < MAX_FILEID)
	{
		digitalWrite(_pinCE, LOW);
		_writeByte(0x0B);
		_writeByte(0x00);
		_writeByte(((fileid * 32) & 0x0000FFFF)>>8);
		_writeByte((fileid * 32) & 0x000000FF);
		_writeByte(0x00);

		startpage = (_readByte()<<8) + _readByte();
		digitalWrite(_pinCE, HIGH);
	}
	else
		return ERR_FILE_DOES_NOT_EXIST;

	if (startpage == 0xFFFF)
		return ERR_FILE_DOES_NOT_EXIST;

	digitalWrite(_pinCE, LOW);
	_writeByte(0x0B);
	_writeByte(0x00);
	_writeByte((((fileid * 32) + 2) & 0x0000FFFF)>>8);
	_writeByte(((fileid * 32) + 2) & 0x000000FF);
	_writeByte(0x00);
	filetype = _readByte();
	digitalWrite(_pinCE, HIGH);

	if (filetype != 3)
		return ERR_FILETYPE_INCORRECT;
	
	digitalWrite(_pinCE, LOW);
	_writeByte(0x0B);
	_writeByte((startpage & 0x0000FFFF)>>8);
	_writeByte(startpage & 0x000000FF);
	_writeByte(0x00);
	_writeByte(0x00);

	linecount = (_readByte()<<8) + _readByte();
	digitalWrite(_pinCE, HIGH);

	if (resid > linecount-1)
		return ERR_OUT_OF_RANGE;

	digitalWrite(_pinCE, LOW);
	_writeByte(0x0B);
	_writeByte((((startpage * PAGE_SIZE) + (resid * 4) + 2) & 0x00FF0000)>>16);
	_writeByte((((startpage * PAGE_SIZE) + (resid * 4) + 2) & 0x0000FF00)>>8);
	_writeByte(((startpage * PAGE_SIZE) + (resid * 4) + 2) & 0x000000FF);
	_writeByte(0x00);

	offset = (_readByte()<<8) + _readByte();
	length = (_readByte()<<8) + _readByte();
	digitalWrite(_pinCE, HIGH);

	for (uint32_t c=0; c<buflen; c++)
		buffer[c]=0;

	if (buflen<(length+1))
	{
		length = buflen-1;
		overflow = true;
	}

	digitalWrite(_pinCE, LOW);
	_writeByte(0x0B);
	_writeByte((((startpage * PAGE_SIZE) + offset) & 0x00FF0000)>>16);
	_writeByte((((startpage * PAGE_SIZE) + offset) & 0x0000FF00)>>8);
	_writeByte(((startpage * PAGE_SIZE) + offset) & 0x000000FF);
	_writeByte(0x00);

	for (uint32_t c=0; c<length; c++)
		buffer[c]=_readByte();
	digitalWrite(_pinCE, HIGH);

	if (overflow == true)
		return ERR_BUFFER_OVERFLOW;
	else
		return ERR_NO_ERROR;
}

/****************************************************************************************/
/* Proctected                                                                           */
/****************************************************************************************/

uint8_t SPIflash::_readByte()
{
	if (_HW_SPI)
	{
		return _SPIread();
	}
	else
	{
		pinMode(_pinSO, INPUT);
		digitalWrite(_pinSCK, LOW);

		uint8_t value = 0;
		uint8_t currentBit = 0;

		for (int i = 0; i < 8; ++i)
		{
			currentBit = digitalRead(_pinSO);
			value |= (currentBit << (7-i));
			digitalWrite(_pinSCK, HIGH);
			delayMicroseconds(1);
			digitalWrite(_pinSCK, LOW);
		}
		return value;
	}
}

void SPIflash::_writeByte(uint8_t value)
{
	if (_HW_SPI)
	{
		_SPIwrite(value);
	}
	else
	{
		pinMode(_pinSI, OUTPUT);
		digitalWrite(_pinSCK, LOW);
		shiftOut(_pinSI, _pinSCK, MSBFIRST, value);
	}
}

void SPIflash::_setBlockProtection(uint8_t prot)
{
	digitalWrite(_pinCE, LOW);
	_writeByte(0x50);
	digitalWrite(_pinCE, HIGH);

	digitalWrite(_pinCE, LOW);
	_writeByte(0x01);
	_writeByte((prot & 0x0f)<<2);
	digitalWrite(_pinCE, HIGH);
}

void SPIflash::_ID_Device()
{
	uint8_t res1, res2, res3;

	digitalWrite(_pinCE, LOW);
	_writeByte(0x9F);
	res1=_readByte();
	res2=_readByte();
	res3=_readByte();
	digitalWrite(_pinCE, HIGH);
	ID_manufacturer=res1;
	ID_type=res2;
	ID_device=res3;
	Text_manufacturer=(char*)"xxx";
	for(uint32_t i = 0; i<(sizeof(_device)/sizeof(struct _s_device)); i++)
	{
		if ((_device[i]._manufacturer_id==res1) & (_device[i]._type_id==res2) & (_device[i]._device_id==res3))
		{
			Text_manufacturer	= _device[i]._manufacturer_name;
			Text_type			= _device[i]._device_type;
			Text_device			= _device[i]._device_name;
			Capacity			= _device[i]._size;
			Pages				= _device[i]._pages+1;
			_max_pages			= _device[i]._pages;
			_cmd_write			= _device[i]._cmd_write;
			_flags				= _device[i]._flags;
		}
	}
#if defined(DEBUG)	
	Serial.println("------");
	Serial.print("Manufacturer :");
	Serial.println(Text_manufacturer);
	Serial.print("type 0x");
	Serial.println(Text_type);
	Serial.print("Device 0x");
	Serial.println(Text_device);
	Serial.print("Capacity 0x");
	Serial.println(Capacity);
	Serial.print("Pages 0x");
	Serial.println(Pages);
	Serial.println("------");
#endif
	if (Text_manufacturer==(char*)"xxx")
	{
		Text_manufacturer	= (char*)"(Unknown manufacturer)";
		Text_type			= (char*)"(Unknown type)";
		Text_device			= (char*)"(Unknown device)";
		Capacity			= 0;
		ID_manufacturer		= 0;
		ID_type				= 0;
		ID_device			= 0;
		_max_pages			= 0;
		_cmd_write			= 0;
		_flags				= 0;
	}
	
}
