#define SCK 3
#define NRS 2
#define DIN 1
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

void setup() {
  pinMode(SCK, OUTPUT);
  pinMode(NRS, OUTPUT);
  pinMode(DIN, OUTPUT);
  pinMode(DNC, OUTPUT);
  pinMode(13, OUTPUT);

  digitalWrite(NRS, HIGH);
  delay(5);
  digitalWrite(NRS, LOW);
  delay(500);
  digitalWrite(NRS, HIGH);
  
  digitalWrite(DNC, LOW);

  shiftOut(DIN, SCK, MSBFIRST, PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION);
  shiftOut(DIN, SCK, MSBFIRST, PCD8544_SETBIAS | 4);
  shiftOut(DIN, SCK, MSBFIRST, PCD8544_SETVOP  | 56);
  shiftOut(DIN, SCK, MSBFIRST, PCD8544_FUNCTIONSET);
  shiftOut(DIN, SCK, MSBFIRST, PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);
}

int runs = 0;
int led = false;

void loop() {
  digitalWrite(DNC, HIGH);
  
  shiftOut(DIN, SCK, MSBFIRST, 0xAA);
  shiftOut(DIN, SCK, MSBFIRST, 0xAA);
  shiftOut(DIN, SCK, MSBFIRST, 0xAA);
  shiftOut(DIN, SCK, MSBFIRST, 0xAA);
  shiftOut(DIN, SCK, MSBFIRST, 0x55);
  runs+=5;
  if(runs >= (84*6)) {
    runs = 0;
    led = (led + 1)%2;
    digitalWrite(13, led);
    delay(250);
  }
}
