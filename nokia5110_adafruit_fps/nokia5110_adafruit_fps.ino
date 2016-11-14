/*********************************************************************
This is an example sketch for our Monochrome Nokia 5110 LCD Displays

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/338

These displays use SPI to communicate, 4 or 5 pins are required to
interface

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// Software SPI (slower updates, more flexible pin options):
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
//Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 4, 3);

// Hardware SPI (faster, but must use certain hardware pins):
// SCK is LCD serial clock (SCLK) - this is pin 13 on Arduino Uno
// MOSI is LCD DIN - this is pin 11 on an Arduino Uno
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(2, 5, 4);
// Note with hardware SPI MISO and SS pins aren't used but will still be read
// and written to during SPI transfer.  Be careful sharing these pins!

void setup() {
  display.begin();
  display.setContrast(40);

  display.display();
  delay(150);
  display.clearDisplay();

  display.drawRect(0, 0, 84 ,48, BLACK);
  display.setTextSize(2);
  display.setCursor(12,16);
  display.print("08:30");
  display.drawLine(0, 5, 5, 0, BLACK);
  display.drawLine(83-5, 47, 83, 47-5,  BLACK);
  display.display();
}

void loop(){
  display.fillTriangle(1,1 , 3,1 , 1,3 , BLACK);
  display.fillTriangle(82,46 , 82-2,46 , 82,46-2 , BLACK);
  display.display();
  delay(245);
  display.fillTriangle(1,1 , 3,1 , 1,3 , WHITE);
  display.fillTriangle(82,46 , 82-2,46 , 82,46-2 , WHITE);
  display.display();
  delay(245);
}

