#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>

#define NETWORKID     100  //the same on all nodes that talk to each other
#define RECEIVER      1    //unique ID of the gateway/receiver
#define SENDER        2
#define NODEID        RECEIVER  //change to "SENDER" if this is the sender node (the one with the button)
#define FREQUENCY     RF69_433MHZ
#define SERIAL_BAUD   9600

RFM69 radio;

void setup() {
  Serial.begin(SERIAL_BAUD);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  Serial.flush();
}

void loop() {
    Serial.print('[');Serial.print(radio.SENDERID);Serial.print("] ");
    Serial.print((char*)radio.DATA);
    Serial.print("   [FREQUENCY:");Serial.print(radio.getFrequency());Serial.print("]");
    Serial.println();

    Serial.print('[');Serial.print(radio.SENDERID);Serial.print("] ");
    Serial.print((char*)radio.DATA);
    Serial.print("   [TEMPERATURE:");Serial.print(radio.readTemperature(0));Serial.print("]");
    Serial.println();
    
    Serial.flush();
    delay(1000);
}

void Blink(byte PIN, byte DELAY_MS)
{
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
}
