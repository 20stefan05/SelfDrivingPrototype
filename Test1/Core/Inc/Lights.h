/*
 * Lights.h
 *
 *  Created on: Jul 25, 2022
 *      Author: stefan
 */

#ifndef INC_LIGHTS_H_
#define INC_LIGHTS_H_
#include "Config.h"
#include "stm32l4xx_hal.h"
#define PWM_MAX 65535
void LightsAdjustByIntensity(void);
void LightsOff(void);
#endif /* INC_LIGHTS_H_ */
