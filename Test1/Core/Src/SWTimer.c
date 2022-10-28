/*
 * SWTimer.c
 *
 *  Created on: Jul 27, 2022
 *      Author: stefan
 */
#include "SWTimer.h"

uint64_t Interval;

HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
		Interval++;
}
void TimerSWStartup(uint64_t* Timer, uint64_t ValMs){
    *Timer = Interval + ValMs;
}
bool TimerSWCheckExpire(uint64_t* Timer){
    if(Interval>=*Timer) return true;
    return false;
}
uint64_t GetInterval(){
	return Interval;
}
