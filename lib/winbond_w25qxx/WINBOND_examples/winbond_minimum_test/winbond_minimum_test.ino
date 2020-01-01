// Pin 7 am Winbond (HOLD) unbedingt auf HIGH setzen!!!!!


#include <SPI.h>
#define SPI_SLAVE_SEL_PIN    10   
//***commands defines
#define SPI_SLAVE_SEL_PIN    10     // チップセレクトピン番号
#define MAX_BLOCKSIZE        128    // ブロック総数
#define MAX_SECTORSIZE       2048   // 総セクタ数

#define CMD_WRIRE_ENABLE      0x06
#define CMD_WRITE_DISABLE     0x04
#define CMD_READ_STATUS_R1    0x05
#define CMD_READ_STATUS_R2    0x35
#define CMD_WRITE_STATUS_R    0x01 // 未実装
#define CMD_PAGE_PROGRAM      0x02
#define CMD_QUAD_PAGE_PROGRAM 0x32 // 未実装
#define CMD_BLOCK_ERASE64KB   0xd8
#define CMD_BLOCK_ERASE32KB   0x52
#define CMD_SECTOR_ERASE      0x20
#define CMD_CHIP_ERASE        0xC7 // 未実装
#define CMD_ERASE_SUPPEND     0x75 // 未実装
#define CMD_ERASE_RESUME      0x7A // 未実装
#define CMD_POWER_DOWN        0xB9
#define CMD_HIGH_PERFORM_MODE 0xA3 // 未実装
#define CMD_CNT_READ_MODE_RST 0xFF // 未実装
#define CMD_RELEASE_PDOWN_ID  0xAB // 未実装
#define CMD_MANUFACURER_ID    0x90
#define CMD_READ_UNIQUE_ID    0x4B
#define CMD_JEDEC_ID          0x9f

#define CMD_READ_DATA         0x03
#define CMD_FAST_READ         0x0B
#define CMD_READ_DUAL_OUTPUT  0x3B // 未実装
#define CMD_READ_DUAL_IO      0xBB // 未実装
#define CMD_READ_QUAD_OUTPUT  0x6B // 未実装
#define CMD_READ_QUAD_IO      0xEB // 未実装
#define CMD_WORD_READ         0xE3 // 未実装

#define SR1_BUSY_MASK	0x01
#define SR1_WEN_MASK	0x02
// end commands



void setup() {
  // Serial on
        Serial.begin(9600);
        delay(2000);
        Serial.println("Test in 2 seconds");
        delay(2000);
        Serial.println("Test starts");
        
// init spi
    pinMode (SPI_SLAVE_SEL_PIN, OUTPUT);
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV4);
   SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
    deselect();
}

void loop() {
  static byte d[5];
  // test: read ID
  Serial.println("Read ID: ");
    select();
  SPI.transfer(CMD_JEDEC_ID);
  for (byte i =0; i <3; i++) {
    d[i] = SPI.transfer(0x00);
  } 
  deselect();
   for (byte i =0; i <3; i++) {
     Serial.print(i);
     Serial.print(": ");
     Serial.print(d[i]);
     Serial.print(" ");
   }
delay (1000);
}

//*********voids
void select() {
   digitalWrite(SPI_SLAVE_SEL_PIN, LOW); 
}

void deselect() {
   digitalWrite(SPI_SLAVE_SEL_PIN, HIGH); 
}
