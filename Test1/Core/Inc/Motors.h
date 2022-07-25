/*
 * Motors.h
 *
 *  Created on: Dec 29, 2021
 *      Author: stefan
 */

#ifndef MOTORS_H_
#define MOTORS_H_
#include "main.h"
void moveForward(uint8_t speed);
void moveBackward(uint8_t speed);
void moveRight(uint8_t speed);
void moveLeft(uint8_t speed);
void Stop();
void setSpeed(uint8_t* currentSpeed, uint8_t speedToSet);


#endif /* MOTORS_H_ */
