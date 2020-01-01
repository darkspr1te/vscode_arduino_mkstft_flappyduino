// ACHTUNG!!!! Immer zuerst den zu schreibenden Bereich vorher löschen, sonst Mist!!!!!
// löschen nur in Sektoren (4KB = 4096) möglich. d.h. immer 4096 Schritte einplanen oder zwischenpuffern
// Workaround: zuerst den Bereich, den man füllen möchte, mit 0xFF überschreiben? nicht geklappt


#include <SPI.h>
//#include <Metro.h>
#include "winbondflash.h"

winbondFlashSPI mem;

void setup()
{
  pinMode(10,OUTPUT);
    SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  Serial.begin(9600);
  Serial.println("InitWaitingChip...");
  delay(2000);
  Serial.println("Init Chip...");
  if(mem.begin(_W25Q64,SPI,10))
    Serial.println("OK");
  else
  {
    Serial.println("FAILED");
    while(1);
  }
}

void loop()
{
  if(Serial.available() > 0)
  {
    switch(Serial.read())
    {
    case 'R':
      {
        //R0,100
        Serial.print(F("Read "));
        int addr = Serial.parseInt();
        int len = Serial.parseInt();
        Serial.print(F("addr=0x"));
        Serial.print(addr>>8,HEX);
        Serial.print(addr,HEX);
        Serial.print(F(",len=0x"));
        Serial.print(len>>8,HEX);
        Serial.print(len,HEX);
        Serial.println(F(":"));
        uint8_t *buf = new uint8_t[len];
        while(mem.busy());
        mem.read(addr,buf,len);
        for(int i = 0;i < len; i++)
        {
          Serial.print((char)buf[i]);
        }
        Serial.println();
        Serial.println(F("OK"));
        delete [] buf;
      }
      break;
    case 'W':
      {
        //W0,test string[LF]
        //
        Serial.print(F("Write "));
        int addr = Serial.parseInt();
        Serial.print(F("0x"));
        Serial.print(addr>>8,HEX);
        Serial.print(addr,HEX);
        Serial.read();
        uint8_t buf[256];
       memset(buf,0,256);
        uint8_t len = Serial.readBytesUntil('\n',(char*)buf,256);
        Serial.print(F(",0x"));
        Serial.print(len>>8,HEX);
        Serial.print(len,HEX);
        Serial.print(F(": "));
        while(mem.busy());
      
        mem.WE();
        mem.writePage(addr,buf);
        Serial.println(F("OK"));
      }
      break;
    case 'E':
      while(Serial.available() < 1);
      char x = Serial.read();
      //Serial.print(x);
      switch(x)
      {
        int addr;
      case 'S':
        Serial.print(F("Erase Sector "));
        addr = Serial.parseInt();
        Serial.print(F("addr=0x"));
        Serial.print(addr>>8,HEX);
        Serial.print(addr,HEX);
        Serial.print(F(": "));
        mem.WE();
        mem.eraseSector(addr);
        Serial.println("OK");
        break;
      case 'b':
        Serial.print(F("Erase 32k Block "));
        addr = Serial.parseInt();
        Serial.print(F("addr=0x"));
        Serial.print(addr>>8,HEX);
        Serial.print(addr,HEX);
        Serial.print(F(": "));
        mem.WE();
        mem.erase32kBlock(addr);
        Serial.println("OK");
        break;
      case 'B':
        Serial.print(F("Erase 64k Block "));
        addr = Serial.parseInt();
        Serial.print(F("addr=0x"));
        Serial.print(addr>>8,HEX);
        Serial.print(addr,HEX);
        Serial.print(F(": "));
        mem.WE();
        mem.erase64kBlock(addr);
        Serial.println("OK");
        break;
      case 'A':
        while(mem.busy());
        mem.WE();
        mem.eraseAll();
        long ss = millis();
        while(mem.busy())
        {
          Serial.print(millis()-ss);
          delay(1000);
        }
        break;
      }

      /*
      if(Serial.read() == 'S')
       {
       //ES0
       Serial.print(F("Erase Sector "));
       int addr = Serial.parseInt();
       Serial.print(F("addr=0x"));
       Serial.print(addr>>8,HEX);
       Serial.print(addr,HEX);
       Serial.print(F(": "));
       mem.WE();
       mem.eraseSector(addr);
       Serial.println("OK");
       }
       break;
       */
    } 
  }
}



