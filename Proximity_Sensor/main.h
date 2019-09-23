#ifndef MAIN_H_
#define MAIN_H_

#include "driverlib/MSP430FR2xx_4xx/driverlib.h"

#define TIMER_A_PERIOD  100000 //T = 1/f = (TIMER_A_PERIOD * 1 us)
#define HIGH_COUNT      10  //Number of cycles signal is high (Duty Cycle = HIGH_COUNT / TIMER_A_PERIOD)

//Output pin to buzzer
#define BZ1_PORT        GPIO_PORT_P1
#define BZ1_PIN         GPIO_PIN7
//LaunchPad LED1 - note unavailable if UART is used
#define LED1_PORT       GPIO_PORT_P1
#define LED1_PIN        GPIO_PIN0
//LaunchPad LED2
#define LED2_PORT       GPIO_PORT_P4
#define LED2_PIN        GPIO_PIN0
//LaunchPad Pushbutton Switch 1
#define SW1_PORT        GPIO_PORT_P1
#define SW1_PIN         GPIO_PIN2
//LaunchPad Pushbutton Switch 2
#define SW2_PORT        GPIO_PORT_P2
#define SW2_PIN         GPIO_PIN6
//Input to ADC - in this case input A9 maps to pin P8.1
#define ADC_IN_PORT     GPIO_PORT_P8
#define ADC_IN_PIN      GPIO_PIN1
#define ADC_IN_CHANNEL  ADC_INPUT_A9

void Init_GPIO(void);
void Init_Clock(void);
void Init_UART(void);
void Init_Buzzer(void);
void Init_ADC(void);
void Init_Timer(void);
void Init_Keypad(void);

Timer_A_outputPWMParam param; //Timer configuration data structure for PWM
Timer_A_initContinuousModeParam param2; //Timer configuration data structure for Timer A1

#endif /* MAIN_H_ */
