// SPIflash_ChipTest 
// Copyright (C)2015 Rinky-Dink Electronics, Henning Karlsen. All right reserved
// web: http://www.RinkyDinkElectronics.com/
//
// This program will test the flash chip.
//
// ****** WARNING ******
// Running this sketch will erase the entire chip.
//

#include <SPIflash.h>

// Software SPI
// Remember to set the correct pins for your development board
// Parameter order: SI/MOSI, SO/MISO, SCK/SCLK, CE/SS
//SPIflash myFlash(5, 6, 7, 4);

// Hardware SPI
// Remember to set the correct pin for your development board
// Parameter order: CE/SS
SPIflash myFlash(10);

char strbuf[100];
int  ok = 0;
int  mainok = 0;
long maxpage;

void setup()
{
  Serial.begin(115200);
  myFlash.begin();
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}

void loop()
{
  if (myFlash.ID_device!=0)
  {
    Serial.print("The connected device is a ");
    Serial.print(myFlash.Text_device);
    Serial.print(", which is a ");
    Serial.print(myFlash.Capacity, DEC);
    Serial.print("Mbit ");
    Serial.print(myFlash.Text_type);
    Serial.print(" from ");
    Serial.print(myFlash.Text_manufacturer);
    Serial.println(".");
    Serial.println();
    Serial.println("****************************");
    Serial.print("Manufacturer ID : 0x");
    Serial.println(myFlash.ID_manufacturer, HEX);
    Serial.print("Memory type     : 0x");
    Serial.println(myFlash.ID_type, HEX);
    Serial.print("Device ID       : 0x");
    Serial.println(myFlash.ID_device, HEX);
    Serial.println("****************************");
    Serial.print("Number of pages : 0x");
    Serial.println(myFlash.Pages, HEX);
    Serial.println("****************************");
    Serial.println();
    maxpage = myFlash.Pages;
    Serial.print("Waiting for Flash chip to be ready... ");
    myFlash.waitForReady();
    Serial.println("Chip is ready.");

    Serial.println();
    Serial.println("IMPORTANT: The flash chip will be erased if you continue!");
    Serial.println("           **********************************************");
    Serial.println();
    Serial.println("Send any character to start the test...");
    while (!Serial.available()) {};
    while (Serial.available()) {Serial.read();};

    Serial.println();
    Serial.println("Erasing chip");
    digitalWrite(13, HIGH);
    myFlash.eraseChip();
    digitalWrite(13, LOW);
    Serial.println();
    Serial.println("Writing pages");
    for (uint32_t page=0; page<maxpage; page++)
    {
      if ((page % 128) == 0)
      {
        if (page != 0)
          Serial.println();
        sprintf (strbuf, "0x%08lX: ", page);
        Serial.print(strbuf);
      }
      for (int i=0; i<256; i++)
        myFlash.buffer[i] = (page % 256);
      digitalWrite(13, HIGH);
      myFlash.writePage(page);
      Serial.print(".");
      digitalWrite(13, LOW);
    }
    Serial.println();
    Serial.println();
    Serial.println("Clearing buffer");
    for (int i=0; i<256; i++)
      myFlash.buffer[i]=0;
    Serial.println("Reading page (* = OK, - = Fault)");
    for (uint32_t page=0; page<maxpage; page++)
    {
      if ((page % 128) == 0)
      {
        if (page != 0)
          Serial.println();
        sprintf (strbuf, "0x%08lX: ", page);
        Serial.print(strbuf);
      }
      ok = 0;
      digitalWrite(13, HIGH);
      myFlash.readPage(page);
      digitalWrite(13, LOW);
      for (int i=0; i<256; i++)
        if (myFlash.buffer[i] != (page % 256))
        {
          ok = 1;
          mainok = 1;
        }
      if (ok==0)
        Serial.print("*");
      else
        Serial.print("-");
    }
    Serial.println();
    Serial.println();
    if (mainok==0)
      Serial.print("Chip is OK...");
    else
      Serial.print("Chip failed...");
    Serial.println();
    Serial.println();
    Serial.println("Erasing chip");
    digitalWrite(13, HIGH);
    myFlash.eraseChip();
    digitalWrite(13, LOW);
    Serial.println("Done...");
  }
  else
    Serial.println("The connected device is not supported!");
 
  while(1) {};
}


