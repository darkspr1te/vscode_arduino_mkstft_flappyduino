// Pin 7 am Winbond (HOLD) unbedingt auf HIGH setzen!!!!!

// SPI シリアルフラッシュメモリ W25Q64 操作検証プログラム
// W25Q64のメモリ領域構造
//   総バイト数 8388608
//   メモリ空間 24ビットアドレス指定 0x00000 - 0x7FFFFF 
//   ブロック数 128 (64KB/ブロック)
//   セクタ数 2048  ( 4KB/セクタ)
//   総セクタ数 2048

#include <SPI.h>
#include <string.h>
#define ledpin 33
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

void setup() {
    byte buf[256];        // 取得データ
    byte wdata[256];       // 書込みデータ
    
    uint16_t n;           // 取得データ数
        Serial.begin(9600);
        delay(2000);
        Serial.println("Test in 2 seconds");
        delay(2000);
        Serial.println("Test starts");
    W25Q64_begin();        // フラッシュメモリ利用開始

    
    // JEDEC IDの取得テスト
    readManufacturer(buf);
    Serial.print("JEDEC ID : ");
    for (byte i=0; i< 3; i++) {
      Serial.print(buf[i],HEX);
      Serial.print(" ");
    }
    Serial.println("");
    
    // Unique IDの取得テスト
    readUniqieID(buf);
    Serial.print("Unique ID : ");
    for (byte i=0; i< 7; i++) {
      Serial.print(buf[i],HEX);
      Serial.print(" ");
    }
    Serial.println("");
    
    // データの読み込み(アドレス0から256バイト取得)
    memset(buf,0,256);
    n =  read(0,buf, 256);
    Serial.print("Read Data: n=");
    Serial.println(n,DEC);
    dump(buf,256);

    // 高速データの読み込み(アドレス0から256バイト取得)
    memset(buf,0,256);
    n =  fastread(0,buf, 256);
    Serial.print("Fast Read Data: n=");
    Serial.println(n,DEC);
    dump(buf,256);

    // セクタ単位の削除
    n = eraseSector(0,true);
    Serial.print("Erase Sector(0): n=");
    Serial.println(n,DEC);
    memset(buf,0,256);
    n =  read (0,buf, 256);
    dump(buf,256);
 
    // データ書き込みテスト
    for (byte i=0; i < 255;i++) {
      wdata[i]= i;
    }  
    n =  page_write(0, 0, wdata, 255);
    Serial.print("page_write(0,0,d,16): n=");
    Serial.println(n,DEC);
    memset(buf,0,256);
    n =  read (0,buf, 256);
    dump(buf,256);

    // ステータスレジスタ1の取得
    buf[0] = readStatusReg1();
    Serial.print("Status Register-1: ");
    Serial.print(buf[0],BIN);
    Serial.println("");

    // ステータスレジスタ2の取得
    buf[0] = readStatusReg2();
    Serial.print("Status Register-2: ");
    Serial.print(buf[0],BIN);
    Serial.println("");
}

//
// フラッシュメモリ W25Q64の利用開始
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
// フラッシュメモリ W25Q64の利用終了
// 
void W25Q64_end() {
  powerDown();
   SPI.end();
}

//
// チップセレクト
// フラッシュメモリ操作を選択にする
//
void select() {
   digitalWrite(SPI_SLAVE_SEL_PIN, LOW); 
}

//
// チップディセレクト
// フラッシュメモリ操作を有非選択にする
//
void deselect() {
   digitalWrite(SPI_SLAVE_SEL_PIN, HIGH); 
}

//
// ステータスレジスタ1の値取得
// 戻り値: ステータスレジスタ1の値
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
// ステータスレジスタ2の値取得
// 戻り値: ステータスレジスタ2の値
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
// JEDEC ID(Manufacture, Memory Type,Capacity)の取得
// d(out) :Manufacture, Memory Type,Capacityの３バイトを格納する
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
// Unique IDの取得
// d(out): Unique ID 7バイトを返す  
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
// 書込み等の処理中チェック
// 戻り値: true:作業 、false:アイドル中
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
//　パワーダウン指定 
//
void powerDown() {
  select();
  SPI.transfer(CMD_POWER_DOWN);
  deselect();
}

//
// 書込み許可設定
//
void WriteEnable() {
  select();
  SPI.transfer(CMD_WRIRE_ENABLE);
  deselect();
}

//
// 書込み禁止設定
//
void WriteDisable() {
  select();
  SPI.transfer(CMD_WRITE_DISABLE);
  deselect();
}

//
// データの読み込み
// addr(in): 読込開始アドレス (24ビット 0x00000 - 0xFFFFF)
// n(in):読込データ数
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
// 高速データの読み込み
// addr(in): 読込開始アドレス (24ビット 0x00000 - 0xFFFFF)
// n(in):読込データ数
//
uint16_t fastread(uint32_t addr,uint8_t *buf,uint16_t n) {
  select();
  SPI.transfer(CMD_FAST_READ);
  SPI.transfer(addr>>16);          // A23-A16
  SPI.transfer((addr>>8) & 0xFF);  // A15-A08
  SPI.transfer(addr & 0xFF);       // A07-A00
  SPI.transfer(0x00);              // ダミー
  
  uint16_t i;
  for(i = 0; i<n; i++) {
    buf[i] = SPI.transfer(0x00);
  }
  
  deselect();  
  return i;
}

//
// セクタ単位消去(4kb空間単位でデータの消去を行う)
// sect_no(in) セクタ番号(0 - 2048)
// flgwait(in) true:処理待ちを行う false:待ち無し
// 戻り値: true:正常終了 false:失敗
//  補足： データシートでは消去に通常 30ms 、最大400msかかると記載されている
//         アドレス23ビットのうち上位 11ビットがセクタ番号の相当する。下位12ビットはセクタ内アドレスとなる。
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
// 64KBブロック単位消去(64kb空間単位でデータの消去を行う)
// blk_no(in) ブロック番号(0 - 127)
// flgwait(in) true:処理待ちを行う false:待ち無し
// 戻り値: true:正常終了 false:失敗
//   補足: データシートでは消去に通常 150ms 、最大1000msかかると記載されている
//         アドレス23ビットのうち上位 7ビットがブロックの相当する。下位16ビットはブロック内アドレスとなる。
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
// 32KBブロック単位消去(32kb空間単位でデータの消去を行う)
// blk_no(in) ブロック番号(0 - 255)
// flgwait(in) true:処理待ちを行う false:待ち無し
// 戻り値: true:正常終了 false:失敗
//   補足: データシートでは消去に通常 120ms 、最大800msかかると記載されている
//         アドレス23ビットのうち上位 8ビットがブロックの相当する。下位15ビットはブロック内アドレスとなる。
//
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
 
 // 処理待ち
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
//
boolean eraseAll(boolean flgwait) {
 WriteEnable();  
 select(); 
 SPI.transfer(CMD_CHIP_ERASE);
 deselect();

 // 処理待ち
 while(IsBusy() & flgwait) {
    delay(500);
 }
 
 deselect();
 return true;
}

//
// データの書き込み
// sect_no(in) : セクタ番号(0x00 - 0x7FF) 
// inaddr(in)  : セクタ内アドレス(0x00-0xFFF)
// data(in)    : 書込みデータ格納アドレス
// n(in)       : 書込みバイト数(0～256)
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
// 書込みデータのダンプリスト
// dt(in) : データ格納先頭アドレス
// n(in)  : 表示データ数
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
