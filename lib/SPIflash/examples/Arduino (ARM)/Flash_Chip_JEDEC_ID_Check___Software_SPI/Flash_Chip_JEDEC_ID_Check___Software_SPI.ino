// Flash_Chip_JEDEC_ID_Check___Software_SPI 
// Copyright (C)2015 Rinky-Dink Electronics, Henning Karlsen. All right reserved
// web: http://www.RinkyDinkElectronics.com/
//
// This program will attempt to read the JEDEC ID from
// the flash memory chip. If successful it will check
// if the chip is compatible with the SPIflash library.
//

// Set the signal pins here:
int _CE = 52;
int _SCK = 76;
int _SI = 75;
int _SO = 74;

struct _s_manufacturer
{
  uint8_t  _id;
  char     *_name;
};

struct _s_memtype
{
  uint8_t  _id;
  uint8_t  _m_id;
  char     *_name;
};

struct _s_device
{
  uint8_t  _id;
  uint8_t  _t_id;
  uint8_t  _m_id;
  uint16_t _size;
  char     *_name;
};

_s_manufacturer _manufacturer[]=
{
  {0xBF, "SST/Microchip"},
  {0xEF, "Winbond"},
  {0xC2, "MXIC"}
};

_s_memtype _memtype[]=
{
  {0x25, 0xBF, "SPI Serial Flash"},
  {0x40, 0xEF, "SPI Serial Flash"},
  {0x20, 0xC2, "SPI Serial Flash"}
};

_s_device _device[]=
{
  {0x8C, 0x25, 0xBF, 0x0002, "SST25VF020B"},
  {0x8D, 0x25, 0xBF, 0x0004, "SST25VF040B"},
  {0x8E, 0x25, 0xBF, 0x0008, "SST25VF080B"},
  {0x41, 0x25, 0xBF, 0x0010, "SST25VF016B"},
  {0x4A, 0x25, 0xBF, 0x0020, "SST25VF032B"},
  {0x4B, 0x25, 0xBF, 0x0040, "SST25VF064C"},
  {0x14, 0x40, 0xEF, 0x0008, "W25Q08BV"},
  {0x15, 0x40, 0xEF, 0x0010, "W25Q16BV"},
  {0x16, 0x40, 0xEF, 0x0020, "W25Q32BV"},
  {0x17, 0x40, 0xEF, 0x0040, "W25Q64FV"},
  {0x18, 0x40, 0xEF, 0x0080, "W25Q128BV/FV"},
  {0x19, 0x40, 0xEF, 0x0100, "W25Q256FV"},
  {0x15, 0x20, 0xC2, 0x0010, "MX25L1605D"},
  {0x16, 0x20, 0xC2, 0x0020, "MX25L3205D"},
  {0x17, 0x20, 0xC2, 0x0040, "MX25L6405D"},
};

byte bt, b1, b2, b3;

boolean _printManu(uint8_t id, bool par=false)
{
  boolean found = false;
  
  if (par)
    Serial.print(" (");
  for(int i = 0; i<(sizeof(_manufacturer)/sizeof(struct _s_manufacturer)); i++)
  {
    if (_manufacturer[i]._id==id)
    {
      found = true;
      Serial.print(_manufacturer[i]._name);
      if (par)
        Serial.println(")");
      else
        Serial.println();
    }
  }
  if (!found)
  {
      Serial.print("Unknown/Unsupported");
      if (par)
        Serial.println(")");
      else
        Serial.println();
  }
  
  return found;
}

boolean _printMemType(uint8_t id, uint8_t mid, bool par=false)
{
  boolean found = false;
  
  if (par)
    Serial.print(" (");
  for(int i = 0; i<(sizeof(_memtype)/sizeof(struct _s_memtype)); i++)
  {
    if ((_memtype[i]._id==id) & (_memtype[i]._m_id==mid))
    {
      found = true;
      Serial.print(_memtype[i]._name);
      if (par)
        Serial.println(")");
      else
        Serial.println();
    }
  }
  if (!found)
  {
      Serial.print("Unknown/Unsupported");
      if (par)
        Serial.println(")");
      else
        Serial.println();
  }
  
  return found;
}

boolean _printDevice(uint8_t id, uint8_t tid, uint8_t mid, bool par=false)
{
  boolean found = false;
  
  if (par)
    Serial.print(" (");
  for(int i = 0; i<(sizeof(_device)/sizeof(struct _s_device)); i++)
  {
    if ((_device[i]._id==id) & (_device[i]._m_id==mid) & (_device[i]._t_id==tid))
    {
      found = true;
      Serial.print(_device[i]._name);
      Serial.print(", ");
      Serial.print(_device[i]._size, DEC);
      Serial.print("MBit");
      if (par)
        Serial.println(")");
      else
        Serial.println();
    }
  }
  if (!found)
  {
      Serial.print("Unknown/Unsupported");
      if (par)
        Serial.println(")");
      else
        Serial.println();
  }
  
  return found;
}

void _readByte()
{
  pinMode(_SO, INPUT);

  uint8_t value = 0;
  uint8_t currentBit = 0;

  for (int i = 0; i < 8; ++i)
  {
    currentBit = digitalRead(_SO);
    value |= (currentBit << (7-i));
    digitalWrite(_SCK, HIGH);
    delayMicroseconds(1);
    digitalWrite(_SCK, LOW);
  }
  bt=value;
}

void _writeByte(uint8_t value)
{
  Serial.print("Sending data    :> 0x");
  Serial.println(value, HEX);
  pinMode(_SI, OUTPUT);
  shiftOut(_SI, _SCK, MSBFIRST, value);
}

void setup()
{
  pinMode(_CE, OUTPUT);
  pinMode(_SCK, OUTPUT);
  pinMode(_SI, OUTPUT);
  pinMode(_SO, INPUT);
  
  Serial.begin(115200);
}

void loop()
{
  delay(1000);
  Serial.println("*****************************");
  Serial.println("* Flash Chip JEDEC ID Check *");
  Serial.println("*****************************");
  Serial.println();
  digitalWrite(_SCK, LOW);
  digitalWrite(_CE, LOW);
  _writeByte(0x9F);      // JEDEC Read-ID

  _readByte();
  b1=bt;  
  _readByte();
  b2=bt;  
  _readByte();
  b3=bt;  
  digitalWrite(_CE, HIGH);
  
  Serial.print("Manufacturers ID:< 0x");
  Serial.print(b1, HEX);
  if (_printManu(b1, true))
  {
    Serial.print("Memory type     :< 0x");
    Serial.print(b2, HEX);
    if (_printMemType(b2, b1, true))
    {
      Serial.print("Device ID       :< 0x");
      Serial.print(b3, HEX);
      _printDevice(b3, b2, b1, true);
    }
  }
  Serial.println();
  Serial.println("*****************************");
  Serial.println();
  
  while (1) {};
}


