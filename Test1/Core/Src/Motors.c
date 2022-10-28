/*
 * Motors.c
 *
 *  Created on: Dec 29, 2021
 *      Author: stefan
 */
#include "Motors.h" /* pa12 - front left(1), pb0 - front left (2),  pb7- front right (1), pb6 - front right(2); */
void MoveForward(uint8_t Speed){
	HAL_GPIO_WritePin(IN_1_REGISTER, IN1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN_2_TO_4_REGISTER, IN2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_2_TO_4_REGISTER, IN3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN_2_TO_4_REGISTER, IN4, GPIO_PIN_RESET);
	TIM2->CCR1 = ((uint32_t)Speed)*ACC_VAL/MAX_SPEED;
}
void MoveBackward(uint8_t Speed){
	HAL_GPIO_WritePin(IN_1_REGISTER, IN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_2_TO_4_REGISTER, IN2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN_2_TO_4_REGISTER, IN3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_2_TO_4_REGISTER, IN4, GPIO_PIN_SET);
	TIM2->CCR1 = ((uint32_t)Speed)*ACC_VAL/MAX_SPEED;
}
void MoveLeft(uint8_t Speed){
	HAL_GPIO_WritePin(IN_1_REGISTER, IN1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN_2_TO_4_REGISTER, IN2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_2_TO_4_REGISTER, IN3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_2_TO_4_REGISTER, IN4, GPIO_PIN_SET);
	TIM2->CCR1 = ((uint32_t)Speed)*ACC_VAL/MAX_SPEED;
}
void MoveRight(uint8_t Speed){
	HAL_GPIO_WritePin(IN_1_REGISTER, IN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_2_TO_4_REGISTER, IN2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN_2_TO_4_REGISTER, IN3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN_2_TO_4_REGISTER, IN4, GPIO_PIN_RESET);
	TIM2->CCR1 = ((uint32_t)Speed)*ACC_VAL/MAX_SPEED;
}
void Stop(void){
	HAL_GPIO_WritePin(GPIOA, IN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_2_TO_4_REGISTER, IN2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_2_TO_4_REGISTER, IN3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_2_TO_4_REGISTER, IN4, GPIO_PIN_RESET);
}
void setSpeed(uint8_t* CurrentSpeed, uint8_t SpeedToSet){
	if((SpeedToSet>0u) && (SpeedToSet<=MAX_SPEED))
	{
		*CurrentSpeed = SpeedToSet;
	}
	else
	{
		*CurrentSpeed = 0u;
	}
}
