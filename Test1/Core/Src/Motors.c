/*
 * Motors.c
 *
 *  Created on: Dec 29, 2021
 *      Author: stefan
 */
#include "Motors.h" //pa12 - front left(1), pb0 - front left (2),  pb7- front right (1), pb6 - front right(2);
void moveForward(uint8_t speed){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	TIM2->CCR1 = speed*65535/10;
}
void moveBackward(uint8_t speed){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
	TIM2->CCR1 = speed*65535/10;
}
void moveLeft(uint8_t speed){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
	TIM2->CCR1 = speed*65535/10;
}
void moveRight(uint8_t speed){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
	TIM2->CCR1 = speed*65535/10;
}
void Stop(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
}
void setSpeed(uint8_t* currentSpeed, uint8_t speedToSet){
	if(speedToSet>0 && speedToSet<=10)
		*currentSpeed = speedToSet;
	else *currentSpeed = 0;
}
