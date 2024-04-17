

#ifndef INC_SERVOS_H_
#define INC_SERVOS_H_

#include <stdint.h>
#include "stm32f4xx.h"


// ANNIMO SERVOS: 270 degrees rotation. 500-2500 us PWM, center at 1500. 270/2000=0.135deg/us.
// For 17 degrees rotation both directions: 17/0.135 = 126
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
