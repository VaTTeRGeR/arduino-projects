#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

int main (void) {
  CCP = 0xD8; // lift write protection for 4 clock cycles
  CLKMSR = 0b0000; // internal 8MHz oscillator
  CLKPSR = 0b0000; // 1 prescaler => 8 MHz clk_io
  
  PUEB  = 0b0000; // no pullup
  DDRB  = 0b0110; // PB1 and PB2 as OUTPUT

  TCCR0A = 0b00100000 | 0b00000001; // set on bottom, clear on OC0B match | FAST 8bit PWM
  TCCR0B = 0b00000001 | 0b00001000; // 1 prescaler => 8MHz timer-clk | FAST 8bit PWM
  TCCR0C = 0b00000000; // all default
  // => 31.25kHz FAST PWM
  
  while(1){
    loop();
  };
}

uint8_t pwm = 0; //current count
uint8_t updown = 1; // count up or down

void loop(){
  if(updown) {
    pwm += 1;
    if(pwm == 255) {
      updown = 0;
      PORTB = 0b0000;
    }
  } else {
    pwm -= 1;
    if(pwm == 0) {
      updown = 1;
      PORTB = 0b0100;
    }
  }
  
  OCR0B = pwm;
  
  delayMillis(2); //=> 2ms*512=> ~1Hz
}

void delayMillis(const uint16_t milliseconds){
  for(volatile uint16_t i = 0; i < 34*milliseconds; i++);
}
