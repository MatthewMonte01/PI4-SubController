

#ifndef INC_SERVOS_H_
#define INC_SERVOS_H_

#include <stdint.h>
#include "stm32f4xx.h"


#define SERVO_MAX_PWM 1000
#define SERVO_MIN_PWM 500
#define SERVO_CENTER_PWM 750
#define SERVO_ANGLE_VARIATION 250  // Ce paramètre contrôle la vitesse de rotation du servo



void turnLeft();
void turnRight();
void surface();
void dive();
void neutralRudders();




#endif /* INC_SERVOS_H_ */
