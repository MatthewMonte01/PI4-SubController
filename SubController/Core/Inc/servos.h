

#ifndef INC_SERVOS_H_
#define INC_SERVOS_H_

#include <stdint.h>
#include "stm32f4xx.h"

#define SERVO_MAX_PWM 1175
#define SERVO_MIN_PWM 425
#define SERVO_CENTER_PWM 800
#define SERVO_ANGLE_VARIATION 60  // this paramater sets servo rotation speed



void turnLeft();
void turnRight();
void surface();
void dive();
void neutralRudders();




#endif /* INC_SERVOS_H_ */
