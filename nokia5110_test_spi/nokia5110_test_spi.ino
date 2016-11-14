#define NRS 2
#define DNC 0

#define LCDWIDTH 84
#define LCDHEIGHT 48

#define PCD8544_POWERDOWN 0x04
#define PCD8544_ENTRYMODE 0x02
#define PCD8544_EXTENDEDINSTRUCTION 0x01

#define PCD8544_DISPLAYBLANK 0x0
#define PCD8544_DISPLAYNORMAL 0x4
#define PCD8544_DISPLAYALLON 0x1
#define PCD8544_DISPLAYINVERTED 0x5

#define PCD8544_FUNCTIONSET 0x20
#define PCD8544_DISPLAYCONTROL 0x08
#define PCD8544_SETYADDR 0x40
#define PCD8544_SETXADDR 0x80

#define PCD8544_SETTEMP 0x04
#define PCD8544_SETBIAS 0x10
#define PCD8544_SETVOP 0x80

#include <SPI.h>

void setup() {
  SPI.begin();
  
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  
  pinMode(NRS, OUTPUT);
  pinMode(DNC, OUTPUT);
  pinMode(13, OUTPUT);

  digitalWrite(NRS, HIGH);
  delay(5);
  digitalWrite(NRS, LOW);
  delay(500);
  digitalWrite(NRS, HIGH);
  
  digitalWrite(DNC, LOW);

  SPI.transfer(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION);
  SPI.transfer(PCD8544_SETBIAS | 4);
  SPI.transfer(PCD8544_SETVOP  | 64);
  SPI.transfer(PCD8544_FUNCTIONSET);
  SPI.transfer(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
}

int runs = 0;
int led = false;

void loop() {
  digitalWrite(DNC, HIGH);
  
  SPI.transfer(0xAA);
  SPI.transfer(0xAA);
  SPI.transfer(0xAA);
  SPI.transfer(0xAA);
  SPI.transfer(0x55);
  runs+=5;
  if(runs >= (84*6*2)) {
    runs = 0;
    led = (led + 1)%2;
    digitalWrite(13, led);
    //delay(1000);
  }
}
