#ifndef _RC_RADIO_JOYSTICKS_H
#define _RC_RADIO_JOYSTICKS_H

#define getXLeft() ((unsigned int)analogRead(A1))
#define getYLeft() ((unsigned int)analogRead(A0))

#define getXRight() (1023L - ((unsigned int)analogRead(A2)))
#define getYRight() ((unsigned int)analogRead(A3))

#define getJB() (!digitalRead(4))
#define getJX() (1023L - (unsigned int)analogRead(A7))
#define getJY() ((unsigned int)analogRead(A6))

#endif
