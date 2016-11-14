#include <SPI.h>

#include <RFM69.h>
#include <RFM69registers.h>

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#define LED_R 5
#define LED_G 6
#define LED_DEFAULT 13

#define getXLeft() ((long)analogRead(A1))
#define getYLeft() ((long)analogRead(A0))

#define getXRight() (1023L - ((long)analogRead(A2)))
#define getYRight() ((long)analogRead(A3))

#define BITN 0x00
#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

#define TX_RFM69 1
#define RX_RFM69 2

#define NETWORK_RFM69 100

#define TX_PER_SECOND 30

#define SIGNAL_WARN_LB -85
#define SIGNAL_WARN_UB -115
#define SIGNAL_WARN_TIME 500

typedef struct {
  uint16_t x_left;
  uint16_t y_left;
  uint16_t x_right;
  uint16_t y_right;
  uint8_t flags;
} Packet;

typedef struct {
  signed int rssi;
} PacketRSSI;

RFM69 radio;
Adafruit_PCD8544 pcd = Adafruit_PCD8544(7, 8, 9);

Packet packet;
PacketRSSI packetRSSI;

boolean radio_ready = false;

unsigned long t_last_rssi = 0;


void setup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);

  digitalWrite(LED_G, LOW);
  digitalWrite(LED_R, HIGH);
  
  pcd.begin();
  
  pcd.setContrast(50);

  pcd.display();
  
  delay(500);
  
  pcd.clearDisplay();

  if(false/* && radio.initialize(RF69_433MHZ, TX_RFM69, NETWORK_RFM69)*/) {
    if(radio.getFrequency() == 433000000L) {
      radio_ready = true;
    
      radio.setHighPower();
      radio.setPowerLevel(18);
  
      radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_9600);
      radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_9600);
    }
  }
}


void loop() {
  if(radio_ready){
    digitalWrite(LED_G, HIGH);
    
    unsigned long t_send = millis();
    
    packet.x_left = getXLeft();
    packet.y_left = getYLeft();
    packet.x_right = getXRight();
    packet.y_right = getYRight();
    packet.flags = 0xAA;
    
    radio.send(RX_RFM69, (const void*)(&packet), sizeof(packet));

    digitalWrite(LED_G, LOW);

    radio.receiveDone();

    wait((1000/TX_PER_SECOND) - max(0, millis() - t_send));

    if(radio.receiveDone()) {
      if(radio.DATALEN == sizeof(packetRSSI)) {
        packetRSSI = *(PacketRSSI*)radio.DATA;
        t_last_rssi = millis();
        if(packetRSSI.rssi <= SIGNAL_WARN_LB) {
          signed int rssi = packetRSSI.rssi;
          rssi = max(SIGNAL_WARN_UB, rssi);
          rssi = min(SIGNAL_WARN_LB, rssi);
          rssi = map(rssi, SIGNAL_WARN_LB, SIGNAL_WARN_UB, 0, 255);
          analogWrite(LED_R, rssi);
        } else {
          analogWrite(LED_R, 0);
        }
      }
    }
    
    if(millis() - t_last_rssi >= SIGNAL_WARN_TIME) {
      t_last_rssi = millis() - SIGNAL_WARN_TIME;
      digitalWrite(LED_R, HIGH);
      wait(10);
      digitalWrite(LED_R, LOW);
      wait(10);
      digitalWrite(LED_R, HIGH);
      wait(10);
      digitalWrite(LED_R, LOW);
      wait(10);
    }
    
  } else {
    blinkLed(20, LED_R);
  }
  
  pcd.clearDisplay();
  
  pcd.drawRect(41, 0, 2 ,48, BLACK);
  pcd.drawRect(0, 0, 84 ,48, BLACK);

  pcd.setTextSize(1);
  pcd.setCursor(2,2);
  pcd.print(radio_ready ? "ROK" : "ERR");

  int pxl = ((getXLeft()*41L)/1023L);
  int pyl = (((1023L - getYLeft())*47L)/1023L);

  int pxr = ((getXRight()*41L)/1023L);
  int pyr = (((1023L - getYRight())*47L)/1023L);

  pcd.drawFastVLine(pxl, 0, 48, BLACK);
  pcd.drawFastHLine(0, pyl, 42, BLACK);
  
  pcd.drawFastVLine(pxr + 42, 0, 48, BLACK);
  pcd.drawFastHLine(42, pyr, 42, BLACK);

  pcd.drawRect(pxl-2, pyl-2, 5, 5, BLACK);
  pcd.drawRect(pxr-2 + 42, pyr-2, 5, 5, BLACK);
  
  pcd.display();
}

// UTIL METHOD SECTION

void blinkLed(unsigned int DELAY_MS, unsigned int LED_PIN) {
    digitalWrite(LED_PIN, HIGH);

    wait((2*DELAY_MS)/10);
    
    digitalWrite(LED_PIN, LOW);

    wait((8*DELAY_MS)/10);
}

void wait(long t_wait) {
  long t_now = millis();
  while(millis() < t_now + t_wait){}
}
