## Flappy birds demo program 
ported by darkspr1te to test graphics routines, touch, sound ,sdcard and eeprom. 
video https://www.youtube.com/watch?v=LzjbU5s8YZI 

This source is compatible with 
MKS TFT 2.8/3.2" touch screens, see my port of marlin touch fw here https://github.com/darkspr1te/MKSTFT_Marlin_Touch
or original function here https://github.com/makerbase-mks/MKS-TFT




bugs :-
yes, next question bwana, 

##Flashing guide
any platformio support device (dont forget to adjust this folders platformio.ini for your upload protocol etc)
You can flash using most JTAG adapters or via UART using FTDI adaper or buspirate 
https://z4ziggy.wordpress.com/2018/09/29/programming-stm32-with-buspirate-v3/
 A9 = PA9 or Serial 1 on Wifi (TXD,RXD)

 A10 = PA10 or Serial 1 on Wifi (TXD,RXD)

 for best results use a jtag/stlink/swd device for realtime memory debugging and proper segfault handling 

Original functions and firmware can be restored by flashing firmware from 
https://github.com/darkspr1te/MKS-TFT-V4-orig-fw
guide and PC software 
https://github.com/darkspr1te/MKSTFT_Marlin_Touch/tree/master/TFT35_V2%20Bootloader%20fix
Note, Do not use bootloader from MarlinTFT github, Only from MKSTFTV4 github

## MKS TFT basic bring up for platformio/vscode
    Just build and flash to device via stlink or uart method 
	(first build will be slow as platformio downloads required libs in background) 
	Does not require use of bootloader  
	inital script work+bugfixes  @delwinbest (https://github.com/delwinbest) 
	
	 Whats working

	.Calibrate TouchTFT(with calibration example code, simple)
	.TFT Via UTFT
	.SD Card via STM libs or arduino 
	.USART1/2/3 
	.SPIFlash
	.Image loading from SPI Flash, BMP's , C array's  etc
	.settings storage via i2c eeprom
	.speaker  (now with working PWM+DMA audio) 
	.ext_sdcard 
	.IRQ's for PB0/1/4/5
	.PWM for outputs inc backlight !! adjust brightness with gamma !!
	.USB Host fat32 via tinyfat/fat_fs
	.bitmap buttons via UTFT_Buttons !!For menus with flair and colour, 320x240 background images supported too !!
	.graphs/scales/Moving coil meter via UTFT_Geometry 
	.hardfault handling (reports & mem dump via UART1) !!This is super handy for debugging !!
	.real time debugging /breakpoints
	

## Connecting ST-LINK v2 to the MKS TFT: 

    ST-LINK    MKS-TFT32: 
    5v         AUX-1 5v 
    GND        AUX-1 GND 
    SWDIO      SWDIO pin 4 
    SWCLK      SWCLK pin 5 

## Board JTAG connector (left-to-right):

    3.3v   GND   GND 
    SWDIO  SWCLK RESET

Disconnect MKS TFT from printer before connecting ST-LINK. Do not connect ST-LINK 3.3v pin.


## Board Configuration

  

    SPI1: SERIAL FLASH MEMORY: Winbond 8MB (64Mb) Serial Flash Memory on SPI1
   
    I2C: Two-wire Serial EEPROM AT24C16B ????
    UART1: TXD,RXD on Wifi connector
    UART2: to AUX1 ramps board 
    UART3: TX,RX  on wifi adapter 
    
	
    PA2: Buzzer


