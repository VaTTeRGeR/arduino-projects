#import <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int value = 78;
int minV = 54;
int maxV = 102;

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(value);              // tell servo to go to position in variable 'pos'
  pinMode(13, OUTPUT);
}

void loop() {
    while(value > minV) {
      myservo.write(value);
      value -= 1;
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
    }
    
    while(value < maxV){
      myservo.write(value);
      value += 1;
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
    }
}
