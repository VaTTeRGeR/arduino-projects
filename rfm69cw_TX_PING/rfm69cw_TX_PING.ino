#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>

#define LED 13

RFM69 radio;

typedef struct {
  int number;
} Packet;

Packet data;


void setup() {
  pinMode(LED, OUTPUT);
  
  Blink(1000);
  Blink(1000);
  Blink(1000);
  
  if(radio.initialize(RF69_433MHZ, 1, 100)) {
    Blink(100);
    Blink(100);
    Blink(100);
    Blink(100);
    Blink(100);
    Blink(100);
    Blink(100);
    Blink(100);
    Blink(100);
    Blink(100);
  }

  
  radio.setPowerLevel(15);
  
  Blink(1000);
  Blink(1000);
  Blink(1000);
  
  data.number = 133;//7;
}

void loop() {
    radio.send(2, (const void*)(&data), sizeof(data));
    Blink(100);
}

void Blink(byte DELAY_MS) {
    digitalWrite(LED, HIGH);
    delay(DELAY_MS);
    digitalWrite(LED, LOW);
    delay(DELAY_MS);
}
