/*
 * Motors.h
 *
 *  Created on: Dec 29, 2021
 *      Author: stefan
 */

#ifndef MOTORS_H_
#define MOTORS_H_
#include "main.h"
#include "Config.h"
#define ACC_VAL (65535U)
#define MAX_SPEED (10u)
void MoveForward(uint8_t Speed);
void MoveBackward(uint8_t Speed);
void MoveRight(uint8_t Speed);
void MoveLeft(uint8_t Speed);
void Stop(void);
void setSpeed(uint8_t* CurrentSpeed, uint8_t SpeedToSet);


#endif /* MOTORS_H_ */
