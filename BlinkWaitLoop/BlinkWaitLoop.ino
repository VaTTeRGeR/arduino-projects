unsigned long t_wait0 = 1900;
unsigned long t_wait1 = 100;
unsigned long t_now;

void setup() {
  pinMode(13, OUTPUT);
}

void loop() {
  t_now = millis();

  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)

  while(millis() < t_now + t_wait0){}

  t_now = millis();

  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW

  while(millis() < t_now + t_wait1){}
}
