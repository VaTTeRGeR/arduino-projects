#ifndef _RC_RADIO_JOYSTICKS_H
#define _RC_RADIO_JOYSTICKS_H

#define getXLeft() ((uint16_t)analogRead(A1))
#define getYLeft() ((uint16_t)analogRead(A0))

#define getXRight() (1023 - ((uint16_t)analogRead(A2)))
#define getYRight() ((uint32_t)analogRead(A3))

#define getJB() (!digitalRead(4))
#define getJX() (1023 - (uint16_t)analogRead(A7))
#define getJY() ((uint16_t)analogRead(A6))

#endif
