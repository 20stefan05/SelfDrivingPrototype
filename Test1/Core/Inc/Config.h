/*
 * Config.h
 *
 *  Created on: Jul 25, 2022
 *      Author: stefan
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_
/* Light-Related defines */
#define DARKEST_ADC_VAL (2800u)
#define BRIGHTEST_ADC_VAL (0u)
/* Motors and GPIO defines */
#define BUTTON_L (GPIO_PIN_4) /* PB_4 <=> D12 */
#define BUTTON_R (GPIO_PIN_5) /* PB_5 <=> D11 */
#define IN1 (GPIO_PIN_12) /* PA_12 <=> D2 */
#define IN2 (GPIO_PIN_0) /* PB_0 <=> D3 */
#define IN3 (GPIO_PIN_7) /* PB_7 <=> D4 */
#define IN4 (GPIO_PIN_6) /* PB7 <=> D5 */
#define EN (GPIO_PIN_0) /* PA0 <=> A0 */
#define UART_BT_TX (GPIO_PIN_2) /* PA_2 <=> A7 */
#define UART_BT_RX (GPIO_PIN_3) /* PA_3 <=> A2 */
#define IN_2_TO_4_REGISTER (GPIOB)
#define IN_1_REGISTER (GPIOA)
#define BUTTON_REGISTER (GPIOB)

#endif /* INC_CONFIG_H_ */
