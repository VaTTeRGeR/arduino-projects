void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  Serial.begin(115200);
  while (!Serial) {}
  
  Serial1.begin(115200);
}

void loop() { // run over and over
  if (Serial1.available()) {
    digitalWrite(13, HIGH);
    Serial.write(Serial1.read());
    digitalWrite(13, LOW);
  }
  
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }
}
