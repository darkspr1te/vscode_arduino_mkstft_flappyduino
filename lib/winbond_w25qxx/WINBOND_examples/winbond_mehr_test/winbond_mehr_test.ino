// winbond W25Q64xxx test code, 
// original library: http://tinyurl.com/ptm98pw
// by WarMonkey (luoshumymail@gmail.com)
// stripped down for STM32-Arduino and translated from Japanese by Matthias Diro (29.03.2015)
// Pin 7 (HOLD) must be set HIGH (3.3V)
// ******** from Datasheet **********
/*
The W25Q64FV array is organized into 32,768 programmable pages of 256-bytes each. 
Up to 256 bytes can be programmed at a time. Pages can be erased in groups of 16 
(4KB sector erase), groups of 128 (32KB block erase), groups of 256 (64KB block erase) 
or the entire chip (chip erase). The W25Q64FV has 2,048 erasable sectors and 128 
erasable blocks respectively. The small 4KB sectors allow for greater flexibility 
in applications that require data and parameter storage.
*/

// rough translation (original: Japanese)
// SPI serial flash memory W25Q64 operation verification program
// W25Q64 memory area structure of
// Total number of bytes 8388608
// Memory space 24-bit addressing 0x00000 - 0x7FFFFF
// Block number 128 (64KB / block)
// Sector number 2048 (4KB / sector)
// Total number of sector 2048

#include <SPI.h>
//#include <string.h>
//#define ledpin 33
#define SPI_SLAVE_SEL_PIN    10     // chip select pin number
#define MAX_BLOCKSIZE        128    // total number of blocks
#define MAX_SECTORSIZE       2048   // total number of sectors

#define CMD_WRIRE_ENABLE      0x06
#define CMD_WRITE_DISABLE     0x04
#define CMD_READ_STATUS_R1    0x05
#define CMD_READ_STATUS_R2    0x35
#define CMD_WRITE_STATUS_R    0x01 // not implemented
#define CMD_PAGE_PROGRAM      0x02
#define CMD_QUAD_PAGE_PROGRAM 0x32 // not implemented
#define CMD_BLOCK_ERASE64KB   0xd8
#define CMD_BLOCK_ERASE32KB   0x52
#define CMD_SECTOR_ERASE      0x20
#define CMD_CHIP_ERASE        0xC7 // not implemented
#define CMD_ERASE_SUPPEND     0x75 // not implemented
#define CMD_ERASE_RESUME      0x7A // not implemented
#define CMD_POWER_DOWN        0xB9
#define CMD_HIGH_PERFORM_MODE 0xA3 // not implemented
#define CMD_CNT_READ_MODE_RST 0xFF // not implemented
#define CMD_RELEASE_PDOWN_ID  0xAB // not implemented
#define CMD_MANUFACURER_ID    0x90
#define CMD_READ_UNIQUE_ID    0x4B
#define CMD_JEDEC_ID          0x9f

#define CMD_READ_DATA         0x03
#define CMD_FAST_READ         0x0B
#define CMD_READ_DUAL_OUTPUT  0x3B // not implemented
#define CMD_READ_DUAL_IO      0xBB // not implemented
#define CMD_READ_QUAD_OUTPUT  0x6B // not implemented
#define CMD_READ_QUAD_IO      0xEB // not implemented
#define CMD_WORD_READ         0xE3 // not implemented

#define SR1_BUSY_MASK	0x01
#define SR1_WEN_MASK	0x02

void setup() {
    byte buf[256];        // get data
    byte wdata[256];       // write data
    
    uint16_t n;           // number of acquired data
        Serial.begin(9600);
        delay(2000);
        Serial.println("Test in 2 seconds");
        delay(2000);
        Serial.println("Test starts");
    W25Q64_begin();        // Flash memory utilization start

    
    // Acquisition test of // JEDEC ID
    readManufacturer(buf);
    Serial.print("JEDEC ID : ");
    for (byte i=0; i< 3; i++) {
      Serial.print(buf[i],HEX);
      Serial.print(" ");
    }
    Serial.println("");
    
    // Acquisition test of Unique ID
    readUniqieID(buf);
    Serial.print("Unique ID : ");
    for (byte i=0; i< 7; i++) {
      Serial.print(buf[i],HEX);
      Serial.print(" ");
    }
    Serial.println("");
    
    // Reading of data (from address 0 to 256 bytes acquisition)
    memset(buf,0,256);
    n =  read(0,buf, 256);
    Serial.print("Read Data: n=");
    Serial.println(n,DEC);
    dump(buf,256);

    // Read of the high-speed data (from address 0 to 256 bytes acquisition)
    memset(buf,0,256);
    n =  fastread(0,buf, 256);
    Serial.print("Fast Read Data: n=");
    Serial.println(n,DEC);
    dump(buf,256);

    // Delete the sector unit
    n = eraseSector(0,true);
    Serial.print("Erase Sector(0): n=");
    Serial.println(n,DEC);
    memset(buf,0,256);
    n =  read (0,buf, 256);
    dump(buf,256);
 
    // Data writing test
    for (byte y=0; y < 255;y++) {
    for (byte i=0; i < 255;i++) {
      wdata[i]= y;
    }  
    n =  page_write(y, 0, wdata, 255);
    Serial.print("page_write(0,0,d,16): n=");
    Serial.println(n,DEC);
    memset(buf,y,256);
    n =  read (y,buf, 256);
    dump(buf,256);
    }
    // Acquisition of status register 1
    buf[0] = readStatusReg1();
    Serial.print("Status Register-1: ");
    Serial.print(buf[0],BIN);
    Serial.println("");

    // Acquisition of status register 2
    buf[0] = readStatusReg2();
    Serial.print("Status Register-2: ");
    Serial.print(buf[0],BIN);
    Serial.println("");
}

//
// Use the start of the flash memory W25Q64
// 
void W25Q64_begin() {
    pinMode (SPI_SLAVE_SEL_PIN, OUTPUT);
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV2);
   SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
    deselect();
}

//
// Use the end of the flash memory W25Q64
// 
void W25Q64_end() {
  powerDown();
   SPI.end();
}

//
// Chip select
// I want to select a flash memory operation
//
void select() {
   digitalWrite(SPI_SLAVE_SEL_PIN, LOW); 
}

//
// Chip deselected
// I make a flash memory operation to Yuhi selection
//
void deselect() {
   digitalWrite(SPI_SLAVE_SEL_PIN, HIGH); 
}

//
// Values obtained in the status register 1
// Returns: status register value of 1
//
byte readStatusReg1() {
  byte rc;
  select();
  SPI.transfer(CMD_READ_STATUS_R1);
  rc = SPI.transfer(0xFF);
  deselect();
  return rc;
}

//
//Values obtained in the status register 2
//  Returns: status register 2 of value
//
byte readStatusReg2() {
  byte rc;
  select();
  SPI.transfer(CMD_READ_STATUS_R2);
  rc = SPI.transfer(0xFF);
  deselect();
  return rc;
}

//
// Acquisition of JEDEC ID(Manufacture, Memory Type,Capacity)
// d (out): Manufacture, Memory Type, store the 3 bytes of Capacity
//
void readManufacturer(byte* d) {
  select();
  SPI.transfer(CMD_JEDEC_ID);
  for (byte i =0; i <3; i++) {
    d[i] = SPI.transfer(0x00);
  } 
  deselect();
}

//
// Acquisition of Unique ID
// d (out):  returns the Unique ID 7 bytes
//
void readUniqieID(byte* d) {
  select();
  SPI.transfer(CMD_READ_UNIQUE_ID);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  for (byte i =0; i <7; i++) {
    d[i] = SPI.transfer(0x00);
  }
 deselect(); 
}

//
// During the processing of writing such as check
// Return Value: true: work, false: in idle
//
boolean IsBusy() {
  uint8_t r1;
  select();
  SPI.transfer(CMD_READ_STATUS_R1);
  r1 = SPI.transfer(0xff);
  deselect();
  if(r1 & SR1_BUSY_MASK)
    return true;
  return false;
}

//
//　Power down the specified 
//
void powerDown() {
  select();
  SPI.transfer(CMD_POWER_DOWN);
  deselect();
}

//
// Write permission settings
//
void WriteEnable() {
  select();
  SPI.transfer(CMD_WRIRE_ENABLE);
  deselect();
}

//
// Write-protect setting
//
void WriteDisable() {
  select();
  SPI.transfer(CMD_WRITE_DISABLE);
  deselect();
}

//
// Reading data
// addr (in): read start address (24 bits 0x00000 - 0xFFFFF)
// n (in): the number of read data
//
uint16_t read(uint32_t addr,uint8_t *buf,uint16_t n){ 
  select();
  SPI.transfer(CMD_READ_DATA);
  SPI.transfer(addr>>16);          // A23-A16
  SPI.transfer((addr>>8) & 0xFF);  // A15-A08
  SPI.transfer(addr & 0xFF);       // A07-A00
 
  uint16_t i;
  for(i = 0; i<n; i++ ) {
    buf[i] = SPI.transfer(0x00);
  }
  
  deselect();
  return i;
}

//
// Read of the high-speed data
// addr (in): read start address (24 bits 0x00000 - 0xFFFFF)
// n (in): the number of read data
//
uint16_t fastread(uint32_t addr,uint8_t *buf,uint16_t n) {
  select();
  SPI.transfer(CMD_FAST_READ);
  SPI.transfer(addr>>16);          // A23-A16
  SPI.transfer((addr>>8) & 0xFF);  // A15-A08
  SPI.transfer(addr & 0xFF);       // A07-A00
  SPI.transfer(0x00);              // dummy
  
  uint16_t i;
  for(i = 0; i<n; i++) {
    buf[i] = SPI.transfer(0x00);
  }
  
  deselect();  
  return i;
}

//
// Erase sector units (erases the data in 4kb space units)
// sect_no (in) sector number (0 - 2048)
// flgwait(in) true:waits to be processed false:No wait
// Return value: true:Successful completion false:Failure
//  Note: in the data sheet is described as normally takes 30ms, 400ms maximum erasure
//         Significant 11 bits of the address 23-bit corresponds sector number. Lower 12 bits is the sector in the address.
//
boolean eraseSector(uint16_t sect_no, boolean flgwait) {
 uint32_t addr = sect_no;
 addr<<=12;

 WriteEnable();
 select(); 
 SPI.transfer(CMD_SECTOR_ERASE);
 SPI.transfer((addr>>16) & 0xff);
 SPI.transfer((addr>>8) & 0xff);
 SPI.transfer(addr & 0xff);
 deselect();
 
 // 処理待ち
 while(IsBusy() & flgwait) {
    delay(10);
 }
 
 return true;
}

//
// 64KB block erase (erasing the data in the 64kb space units)
// blk_no (in) block number (0 - 127)
// flgwait(in) true:waiting to be processed false: no wait
// Return value: true:Successful completion false:Failure
//   Note: in the data sheet is described as normally takes 150ms, 1000ms maximum erasure
//         Upper 7 bits of the address 23-bit is equivalent of block. Lower 16 bits will be block address.
//
boolean erase64Block(uint16_t blk_no, boolean flgwait) {
 uint32_t addr = blk_no;
 addr<<=16;

 WriteEnable();
 select(); 
 SPI.transfer(CMD_BLOCK_ERASE64KB);
 SPI.transfer((addr>>16) & 0xff);
 SPI.transfer((addr>>8) & 0xff);
 SPI.transfer(addr & 0xff);
 deselect();
 
 // 処理待ち
 while(IsBusy() & flgwait) {
    delay(50);
 }
 
 return true;
}


//
// (erasing the data in the 32kb space units) 32KB block erase
// Blk_no (in) block number (0 - 255)
// Flgwait (in) true: false perform a process waiting: waiting None
// Return value: true: successful completion false: failure
// Note: in the data sheet it has been described as usually takes 120ms, 800ms maximum to erase
// Upper 8 bits of the address 23-bit corresponds to block. Lower 15 bits is the block address.
boolean erase32Block(uint16_t blk_no, boolean flgwait) {

 uint32_t addr = blk_no;
 addr<<=15;

 WriteEnable();  
 select(); 
 SPI.transfer(CMD_BLOCK_ERASE32KB);
 SPI.transfer((addr>>16) & 0xff);
 SPI.transfer((addr>>8) & 0xff);
 SPI.transfer(addr & 0xff);
 deselect();
 
 // Waiting to be processed
 while(IsBusy() & flgwait) {
    delay(50);
 }
 
 return true;
}

//
// 全領域の消去
// flgwait(in) true:処理待ちを行う false:待ち無し
// 戻り値: true:正常終了 false:失敗
//   補足: データシートでは消去に通常 15s 、最大30sかかると記載されている

// Erase of the entire region
// Flgwait (in) true: false perform a process waiting: waiting None
// Return value: true: successful completion false: failure
// Note: in the data sheets are described as normally takes 15s, 30s up to the clear
//
boolean eraseAll(boolean flgwait) {
 WriteEnable();  
 select(); 
 SPI.transfer(CMD_CHIP_ERASE);
 deselect();

 // Waiting to be processed
 while(IsBusy() & flgwait) {
    delay(500);
 }
 
 deselect();
 return true;
}

//
// Writing of data
// Sect_no (in): sector number (0x00 - 0x7FF)
// Inaddr (in): sector in the address (0x00-0xFFF)
// Data (in): write data storage address
// N (in): the number of bytes written (0-256)
//
uint16_t page_write(uint16_t sect_no, uint16_t inaddr, byte* data, byte n) {

  uint32_t addr = sect_no;
  int i;
  addr<<=12;
  addr += inaddr;

  WriteEnable();  
  
  if (IsBusy()) {
    return 0;  
  }

  select();
  SPI.transfer(CMD_PAGE_PROGRAM);
  SPI.transfer((addr>>16) & 0xff);
  SPI.transfer((addr>>8) & 0xff);
  SPI.transfer(addr & 0xff);

  for (i=0; i < n; i++) {
    SPI.transfer(data[i]);
  }  
  deselect();
  while(IsBusy()) ;
  
  return i;
}

//
// Dump list of write data
// Dt (in): data storage start address
// N (in): the number of display data
void dump(byte *dt, int n) {
  unsigned long sz;
  char buf[64];
  int clm = 0;
  byte data;
  byte sum;
  byte vsum[16];
  byte total =0;
  int saddr =0;
  int eaddr =n-1;
  sz = eaddr -saddr;
  
  Serial.println("----------------------------------------------------------");
  
  for (int i=0;i<16;i++) vsum[i]=0;  
  for (unsigned long addr = saddr; addr <= eaddr; addr++) {
    data = dt[addr];
    if (clm == 0) {
      sum =0;
      sprintf(buf,"%05lx: ",addr);
      Serial.print(buf);
    }

    sum+=data;
    vsum[addr % 16]+=data;
    
    sprintf(buf,"%02x ",data);
    Serial.print(buf);
    clm++;
    if (clm == 16) {
      sprintf(buf,"|%02x ",sum);
      Serial.print(buf);      
      Serial.println("");
      clm = 0;
    }    
  }
  Serial.println("----------------------------------------------------------");
  Serial.print("       ");
  for (int i=0; i<16;i++) {
    total+=vsum[i];
    sprintf(buf,"%02x ",vsum[i]);
    Serial.print(buf);
  }
  sprintf(buf,"|%02x ",total);
  Serial.print(buf);      
  Serial.println("");
  Serial.println("");
}

void loop() {

}
