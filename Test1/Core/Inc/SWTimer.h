/*
 * SWTimer.h
 *
 *  Created on: Jul 27, 2022
 *      Author: stefan
 */

#ifndef INC_SWTIMER_H_
#define INC_SWTIMER_H_
#include "main.h"
#include <stdbool.h>

void TimerSWStartup(uint64_t* timer, uint64_t val_ms);
bool TimerSWCheckExpire(uint64_t* timer);
uint64_t GetInterval();

#endif /* INC_SWTIMER_H_ */
