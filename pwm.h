#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>

void initPWM();
void leftForward(uint16_t duty);
void leftBackwards(uint16_t duty);
void rightForward(uint16_t duty);
void rightBackwards(uint16_t duty);
void leftStop(void);
void rightStop(void);

#endif
