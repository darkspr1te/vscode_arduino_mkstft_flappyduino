//Flappy bird game demo for MKS TFT, comes with usual health warning :-)
//Original game code from
//https://howtomechatronics.com/projects/arduino-game-project-replica-of-flappy-bird-for-arduino-on-a-tft-touch-screen
//
//Note, read and writes to i2c eeprom, backup if you wish to restore original function, crazy values stored in the eeprom when running
//original MKS fw will cause bugs and features like you've never seen
//**You have been officially notified**
//
//Ported by darkspr1te as part of the MKS_TFT_VSCODE project build
// initial UTFT,touch,sd-card,variants ports by darkspr1te
//
//build scripts/bug fixes/etc also  by https://github.com/delwinbest
//
//



#include <Arduino.h>
#include <UTFT.h>
#include <memorysaver.h>
#include <URTouch.h>
#include <EEPROM.h>
#include <tones_p.h>
#include <SPIflash.h>
#include <SPI.h>
#include <SD.h>
//We using SPIFlash lib right now, but we can swap between , SPI flash is smaller and does not relay on UTFT,UTFT_Buttons
//#include <UTFT_SPIflash.h>
#include <UTFT_Buttons.h> 

const int SD_chipSelect = SDCARD_CS ;//Requied to use Arduino SD Libs
const int FLASH_chipSelect = FLASH_CS;//8Mbyte SPi Flash W25q64

//SPIflash myFlash(FLASH_chipSelect); 


#define X_RES 320-1
#define Y_RES 240-1
#define LED LED_BUILTIN


extern uint8_t SmallFont[];
extern uint8_t Dingbats1_XL[];
extern uint8_t BigFont[];
extern uint8_t SevenSegNumFont[];

// I said it could not be done then proved myself wrong by writing routine that worked
//with PWM , actual sound not just a pc beep, original routine from Arduino.com
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, NOTE_A3, NOTE_B3, NOTE_C4
};
// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

//Define LCD 
//UTFT myGLCD(HX8558,LCD_RS,LCD_WR,LCD_CS,LCD_RD,0);
//LCD_RD is implmented ,but here we dont use it here, maybe some speed can be achieved with GRAM reads so i've left original pointer above 
 UTFT myGLCD(HX8558,LCD_RS,LCD_WR,LCD_CS,NOT_IN_USE,0);
//Define Touch Screen
 URTouch  myTouch(PIN_SPI1_SCK , TOUCH_CS, PIN_SPI1_MOSI,PIN_SPI1_MISO, TOUCH_DI);
 //Not used in this demo but here for completeness
 SPIflash      myFlash(PIN_SPI_MOSI,PIN_SPI_MISO,PIN_SPI_SCK,FLASH_CS);

File SDCARD_Device;

//Comment out when not including header
//UTFT_SPIflash myFiles(&myGLCD, &myFlash);

//SDCard not used in this demo
//File SDCARD_Device;

//Define lcd gfx lib and touch for UTFT_Buttons 
UTFT_Buttons  myButtons(&myGLCD, &myTouch);


//Setup PWM on speaker
 TIM_TypeDef *Instance_two = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(SPEAKER), PinMap_PWM);
 uint32_t channel_two = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(SPEAKER), PinMap_PWM));


  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
  HardwareTimer *MyTim_two = new HardwareTimer(Instance_two);


void start_pwm_backlight()
{
   TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(LCD_LED), PinMap_PWM);
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(LCD_LED), PinMap_PWM));


  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
  HardwareTimer *MyTim = new HardwareTimer(Instance);

  // Configure and start PWM
  MyTim->setPWM(channel, LCD_LED, 160, 50, NULL, NULL); // No callback required, we can simplify the function call
 
}
void pwm_piezo(int status)
{
 
  
  // Configure and start PWM
  if (status==1)
  {
   // No callback required, we can simplify the function call
    MyTim_two->setPWM(channel_two, SPEAKER, 800, 10, NULL, NULL);
    delay(500);
    MyTim_two->setPWM(channel_two, SPEAKER, 1800, 1, NULL, NULL); // No callback required, we can simplify the function call
    delay(500);
  
  }
else 
{
 
  MyTim_two->timerHandleDeinit();
 
 } 
 
}


void do_melody()
{
//Yes Amuse me with your anicdotes about why this routine is so bad
// 
MyTim_two->setMode( channel_two, TIMER_OUTPUT_COMPARE_TOGGLE, SPEAKER);
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    MyTim_two->setPWM(channel_two, SPEAKER,melody[thisNote], 10, NULL, NULL);
   
    
    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
  

  }

 MyTim_two->setPWM(channel_two, SPEAKER, 0, 0, NULL, NULL);

}

//flappy bird stuff
int x, y; // Variables for the coordinates where the display has been pressed
extern unsigned short alt_bird[0x41A]; // Bird Bitmap


// Flappy Bird
int xP = 319;
int yP = 100;
int yB = 50;
int movingRate = 3;
int fallRateInt = 0;
float fallRate = 0;
int score = 0;
int lastSpeedUpScore = 0;
int highestScore;
boolean screenPressed = false;
boolean gameStarted = false;

int s=1;
//====== drawBird() - Custom Function
void drawBird(int y) {
  // Draws the bird - bitmap

  myGLCD.drawBitmap (50, y, 35, 30,alt_bird);
 
  // Draws blue rectangles above and below the bird in order to clear its previus state
  myGLCD.setColor(114, 198, 206);
  myGLCD.fillRoundRect(50,y,85,y-6);
  myGLCD.fillRoundRect(50,y+30,85,y+36);
}
// ===== initiateGame - Custom Function
void initiateGame() {
  myGLCD.clrScr();
  // Blue background
  myGLCD.setColor(114, 198, 206);
  myGLCD.fillRect(0,0,319,239);
  // Ground
  myGLCD.setColor(221,216,148);
  myGLCD.fillRect(0, 215, 319, 239);
  myGLCD.setColor(47,175,68);
  myGLCD.fillRect(0, 205, 319, 214);
  // Text
  myGLCD.setColor(0, 0, 0);
  myGLCD.setBackColor(221, 216, 148);
  myGLCD.setFont(BigFont);
  myGLCD.print("Score:",5,220);
  myGLCD.setFont(SmallFont);
  //I'll mention you up top boss, 
  //myGLCD.print("HowToMechatronics.com", 140, 220); 
  myGLCD.print("ported by darkspr1te", 140, 220); 
  myGLCD.setColor(0, 0, 0);
  myGLCD.setBackColor(114, 198, 206);
  myGLCD.print("Highest Score: ",5,5);
  myGLCD.printNumI(highestScore, 120, 6);
  myGLCD.print(">RESET<",255,5);
  myGLCD.drawLine(0,23,319,23);
  myGLCD.print("TAP TO START",CENTER,100);
  
  drawBird(yB); // Draws the bird
  
  // Wait until we tap the sreen
  while (!gameStarted) {
    if (myTouch.dataAvailable()) {
    myTouch.read();
    x=myTouch.getX();
    y=myTouch.getY();        
    // Reset higest score
    if ((x>=250) && (x<=319) &&(y>=0) && (y<=28)) {
    highestScore = 0;
    myGLCD.setColor(114, 198, 206);
    myGLCD.fillRect(120, 0, 150, 22);
    myGLCD.setColor(0, 0, 0);
    myGLCD.printNumI(highestScore, 120, 5);
    } 
    if ((x>=0) && (x<=319) &&(y>=30) && (y<=239)) {
    gameStarted = true;
    myGLCD.setColor(114, 198, 206);
    myGLCD.fillRect(0, 0, 319, 32);
    }   
  }
  }
  // Clears the text "TAP TO START" before the game start
  myGLCD.setColor(114, 198, 206);
  myGLCD.fillRect(85, 100, 235, 116);
  
}
// ===== drawPlillars - Custom Function
void drawPilars(int x, int y) {
    if (x>=270){
      myGLCD.setColor(0, 200, 20);
      myGLCD.fillRect(318, 0, x, y-1);
      myGLCD.setColor(0, 0, 0);
      myGLCD.drawRect(319, 0, x-1, y);

      myGLCD.setColor(0, 200, 20);
      myGLCD.fillRect(318, y+81, x, 203);
      myGLCD.setColor(0, 0, 0);
      myGLCD.drawRect(319, y+80, x-1, 204); 
    }
    else if( x<=268) {
      // Draws blue rectangle right of the pillar
      myGLCD.setColor(114, 198, 206);
      myGLCD.fillRect(x+51, 0, x+60, y);
      // Draws the pillar
      myGLCD.setColor(0, 200, 20);
      myGLCD.fillRect(x+49, 1, x+1, y-1);
      // Draws the black frame of the pillar
      myGLCD.setColor(0, 0, 0);
      myGLCD.drawRect(x+50, 0, x, y);
      // Draws the blue rectangle left of the pillar
      myGLCD.setColor(114, 198, 206);
      myGLCD.fillRect(x-1, 0, x-3, y);

      // The bottom pillar
      myGLCD.setColor(114, 198, 206);
      myGLCD.fillRect(x+51, y+80, x+60, 204);
      myGLCD.setColor(0, 200, 20);
      myGLCD.fillRect(x+49, y+81, x+1, 203);
      myGLCD.setColor(0, 0, 0);
      myGLCD.drawRect(x+50, y+80, x, 204);
      myGLCD.setColor(114, 198, 206);
      myGLCD.fillRect(x-1, y+80, x-3, 204);
  }
  // Draws the score
  myGLCD.setColor(0, 0, 0);
  myGLCD.setBackColor(221, 216, 148);
  myGLCD.setFont(BigFont);
  myGLCD.printNumI(score, 100, 220);
}

//======== gameOver() - Custom Function
void gameOver() {
 do_melody();
  // Clears the screen and prints the text
  myGLCD.clrScr();
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(0, 0, 0);
  myGLCD.setFont(BigFont);
  myGLCD.print("GAME OVER", CENTER, 40);
  myGLCD.print("Score:", 100, 80);
  myGLCD.printNumI(score,200, 80);
  myGLCD.print("Restarting...", CENTER, 120);
  myGLCD.setFont(SevenSegNumFont);
  myGLCD.printNumI(2,CENTER, 150);
  delay(1000);
  myGLCD.printNumI(1,CENTER, 150);
  delay(1000);
  
  // Writes the highest score in the EEPROM
  if (score > highestScore) {
    highestScore = score;
    EEPROM.write(0,highestScore);
  }
  // Resets the variables to start position values
  xP=319;
  yB=50;
  fallRate=0;
  score = 0;
  lastSpeedUpScore = 0;
  movingRate = 3;  
  gameStarted = false;
  // Restart game
  initiateGame();
}

void setup()
{
  Serial.begin(115200);
  Serial.println("_setup"); //UART1 on Wifi plug, TXD,RXD @3.3v

  pinMode(SS, OUTPUT);
  pinMode(SDCARD_CS,OUTPUT);
  start_pwm_backlight();
  myGLCD.InitLCD(PORTRAIT);
  myGLCD.clrScr();
  myGLCD.setColor(VGA_LIME);
  myGLCD.setFont(SmallFont);
  myGLCD.setBackColor(0,0,0);
  myGLCD.fillScr(VGA_BLACK);
  myTouch.InitTouch();
  myTouch.setPrecision(PREC_MEDIUM);
  
  highestScore = EEPROM.read(0); // Read the highest score from the EEPROM
  Serial.println("end _setup");
  initiateGame(); // Initiate the game


}


void Game_loop()
{
  xP=xP-movingRate; // xP - x coordinate of the pilars; range: 319 - (-51)   
    drawPilars(xP, yP); // Draws the pillars 
    
    // yB - y coordinate of the bird which depends on value of the fallingRate variable
    yB+=fallRateInt; 
    fallRate=fallRate+0.4; // Each inetration the fall rate increase so that we can the effect of acceleration/ gravity
    fallRateInt= int(fallRate);
    
    // Checks for collision
    if(yB>=180 || yB<=0){ // top and bottom
      gameOver();
    }
    if((xP<=85) && (xP>=5) && (yB<=yP-2)){ // upper pillar
      gameOver();
    }
    if((xP<=85) && (xP>=5) && (yB>=yP+60)){ // lower pillar
      gameOver();
    }
    
    // Draws the bird
    drawBird(yB);

    // After the pillar has passed through the screen
    if (xP<=-51){
      xP=319; // Resets xP to 319
      yP = rand() % 100+20; // Random number for the pillars height
      score++; // Increase score by one
    }
    //==== Controlling the bird
    if (myTouch.dataAvailable()&& !screenPressed) {
       fallRate=-6; // Setting the fallRate negative will make the bird jump
       screenPressed = true;
    }
    // Doesn't allow holding the screen / you must tap it
    else if ( !myTouch.dataAvailable() && screenPressed){
      screenPressed = false;
    }
    
    // After each five points, increases the moving rate of the pillars
    if ((score - lastSpeedUpScore) == 5) {
      lastSpeedUpScore = score;
      movingRate++;
    }

}


void loop()
{
   Game_loop();
}
