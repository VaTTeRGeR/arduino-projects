void setup() {
  pinMode(A0, INPUT);
  Serial.begin(9600);
  while (!Serial) {}
  
}

void loop() {
  int value = analogRead(A0);
  
  Serial.print("Value: ");
  Serial.print(value);
  Serial.print("\n");

  Serial.print("Voltage: ");
  Serial.print(((float)value)/1023.0f*5.0f*1.0307);
  Serial.print("\n");

  delay(250);
}
