//** README **//
// This turns an Arduino Micro or other
// 32u4 device into an (only!) ATK500V2 (Arduino Serial Bootloader Protocol)
// compatible serial Adapter with auto reset on program.

// Connections
// Micro  --    Target Board
// VCC    ->    VCC  (Not needed if already powered)
// GND    ->    GND
// RX     ->    TX
// TX     ->    RX
// Pin-2  ->    DTR of target

uint32_t cooldown;
boolean cooledDown = true;
boolean allowMessages = false;

void setup() {
  pinMode(13, OUTPUT);
  pinMode(2, OUTPUT);

  digitalWrite(13, LOW);
  digitalWrite(13, HIGH);
  
  Serial1.begin(57600);
  Serial.begin(57600);
  while(!Serial);
  
  digitalWrite(13, LOW);
}

void resetCooldown() {
  cooldown = millis();
}

boolean updateCooldown() {
  return (cooledDown = (millis()-cooldown) > 500);
}

void loop() {
  if(Serial1.available()) {
    if(allowMessages)
      Serial.write(Serial1.read());
    else
      Serial1.read();
  }

  boolean readSomething = false;
  while(Serial.available()) {
    readSomething = true;
    byte b = Serial.read();
    if(cooledDown && b == 0x30) {

      Serial1.end();
      digitalWrite(2, LOW);
      delay(1);
      digitalWrite(2, HIGH);
      Serial1.begin(57600);
      

      allowMessages = true;
      resetCooldown();
    } else if(cooledDown && b == '.') {
      allowMessages =! allowMessages;
    }
    Serial1.write(b);
  }
  
  if(readSomething)
    resetCooldown();
  else
    updateCooldown();
}

