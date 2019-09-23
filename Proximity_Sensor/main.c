#include "main.h"
#include "driverlib/MSP430FR2xx_4xx/driverlib.h"
#include "hal_LCD.h"
#include "stdlib.h"
#include <string.h>
#include <stdio.h>

/*
 * This project contains some code samples that may be useful.
 *
 * UART: It configures P1.0 and P1.1 to be connected internally to the
 * eSCSI module, which is a serial communications module, and places it
 * in UART mode. This let's you communicate with the PC via a software
 * COM port over the USB cable. You can use a console program, like PuTTY,
 * to type to your LaunchPad. The code in this sample just echos back
 * whatever character was received.
 *
 * ADC:
 *
 * PWM:
 */

char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result
volatile double timerVal = 0;
int flag = 0;
char KeypadCol1 = 0;
char KeypadCol2 = 0;
char KeypadCol3 = 0;
volatile int inputFlag = 1;
volatile int buttonFlag = 0;

void main(void)
{
    //char buttonState = 0; //Current button press state (to allow edge detection)

    /*
     * Functions with two underscores in front are called compiler intrinsics.
     * They are documented in the compiler user guide, not the IDE or MCU guides.
     * They are a shortcut to insert some assembly code that is not really
     * expressible in plain C/C++. Google "MSP430 Optimizing C/C++ Compiler
     * v18.12.0.LTS" and search for the word "intrinsic" if you want to know
     * more.
     * */


    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    Init_Buzzer();  //Sets up a PWM output
    Init_ADC();     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display
    Init_Timer();   //Sets up Echo Timer
    Init_Keypad();  // sets up keypad


     /*
     * The MSP430 MCUs have a variety of low power modes. They can be almost
     * completely off and turn back on only when an interrupt occurs. You can
     * look up the power modes in the Family User Guide under the Power Management
     * Module (PMM) section. You can see the available API calls in the DriverLib
     * user guide, or see "pmm.h" in the driverlib directory. Unless you
     * purposefully want to play with the power modes, just leave this command in.
     */

    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.
    __enable_interrupt();

    displayScrollText("ECE 298");

    uint8_t col, row;
    static uint8_t colPort[3] = { GPIO_PORT_P8, GPIO_PORT_P2, GPIO_PORT_P1 };
    static uint16_t colPin[3] = { GPIO_PIN0, GPIO_PIN5, GPIO_PIN0 };
    static uint8_t rowPort[4] = { GPIO_PORT_P5, GPIO_PORT_P8, GPIO_PORT_P1, GPIO_PORT_P2 };
    static uint16_t rowPin[4] = { GPIO_PIN1, GPIO_PIN2, GPIO_PIN5, GPIO_PIN7 };
    static char keypad[3][4] = { // [col][row]
           {'1', '4', '7', '*'},
           {'2', '5', '8', '0'},
           {'3', '6', '9', '#'}
    };
    volatile char buffer[20] = {0};
    volatile int pos = 0;
    volatile int close = 0;
    volatile int mid = 0;
    volatile int far = 0;
    volatile int led = 0;

    // configure button interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN6, GPIO_LOW_TO_HIGH_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);


    volatile double distance = 0;
    while(1) //Do this when you want an infinite loop of code
    {
        //getting and displaying distance
        if (flag > 0){
            distance = timerVal*340*1000 / (2*32768);
            int dist = (int)distance;
            clearLCD();
            showDistance(dist);
            flag = 0;
        }

        if (distance >= 150) { //green
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);
        }
        else if (distance < 150 && distance >= 100) { //yellow
            GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);
        }
        else if (distance < 100) { //red
            GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
        }

        // programming the leds
        if(inputFlag) {
            for(col = 0; col < 3; col++) {
                 GPIO_setAsOutputPin(colPort[col], colPin[col]);
                 GPIO_setOutputHighOnPin(colPort[col], colPin[col]);
                 for(row = 0; row < 4; row++) {
                     if(GPIO_getInputPinValue(rowPort[row], rowPin[row]) > 0) {
                         showChar(keypad[col][row], 5);
                         if(keypad[col][row] == '#') {
                             switch(led) {
                                 case 0:

                                     //turn on red
                                     GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);
                                     GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
                                     GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);

                                     close = atoi(buffer); // red led
                                     memset(buffer, 0, pos);
                                     led++; // set led color for the next one
                                     pos = 0;

                                     break;
                                 case 1:
                                     //turn on yellow led
                                     GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
                                     GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
                                     GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);

                                     mid = atoi(buffer); // yellow led
                                     memset(buffer, 0, pos);
                                     led++; // set led color for the next one
                                     pos = 0;
                                     break;
                                 case 2:
                                     //turn on greed led
                                     GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);
                                     GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
                                     GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);

                                     far = atoi(buffer); // green led
                                     memset(buffer, 0, pos);
                                     led = 0; // set back to the first led
                                     pos = 0;
                                     inputFlag = 0;
                                     break;
                                 default:
                                     break;
                             }
                         }
                         else {
                             buttonFlag = 1;
                             buffer[pos] = keypad[col][row];
                             showDistance(atoi(buffer));
                             pos++;
                             while(GPIO_getInputPinValue(rowPort[row], rowPin[row]));
                             buttonFlag = 0;
                         }
                     }
                 }
                 GPIO_setAsInputPinWithPullDownResistor(colPort[col], colPin[col]);
            }
        }
        else {
            // if value is 0, use old value
            // if mid > far or close > mid > far, then do something
        }

        //Buttons SW1 and SW2 are active low (1 until pressed, then 0)
//        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 1) & (buttonState == 0)) //Look for rising edge
//        {
//            Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal
//            buttonState = 1;                //Capture new button state
//        }
//        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0) & (buttonState == 1)) //Look for falling edge
//        {
//            Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
//            buttonState = 0;                            //Capture new button state
//        }

    }

    /*
     * You can use the following code if you really do plan on only using interrupts
     * to handle all your system events since you don't need any infinite loop of code.
     *
     * //Enter LPM0 - interrupts only
     * __bis_SR_register(LPM0_bits);
     * //For debugger to let it know that you meant for there to be no more code
     * __no_operation();
    */

}

void Init_Keypad(void)
{
    // COL1: p8.0
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P8, GPIO_PIN0);

    // COL2: p2.5
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN5);

    // COL3: p1.0
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN0);

    // ROW1: p5.1 works!
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P5, GPIO_PIN1);

    // ROW2: p8.1 wont go high (broken), use p8.2
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P8, GPIO_PIN2);

    // ROW3: p1. 1- shorted to 3.3v (broken), use p1.5
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN5);

    // ROW4: p2.7
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN7);
}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);

    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN6);

    //set LED's to high because active low
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2|GPIO_PIN3);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }
}

/* Buzzer PWM Initialization */
void Init_Buzzer(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //BZ1 (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(BZ1_PORT, BZ1_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    Timer_A_outputPWM(TIMER_A0_BASE, &param);
}

/* Timer Initialization */
void Init_Timer(void)
{
    //Set Parameters
    param2.clockSource                  = TIMER_A_CLOCKSOURCE_ACLK; // clk frequency = 32768Hz
    param2.clockSourceDivider           = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param2.timerInterruptEnable_TAIE    = TIMER_A_TAIE_INTERRUPT_DISABLE;
    param2.timerClear                   = TIMER_A_DO_CLEAR; //Defined in main.h
    param2.startTimer                   = 0; //Defined in main.h

    //Init Timer A1 with the given param
    Timer_A_initContinuousMode(TIMER_A1_BASE, &param2);
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}

#pragma vector = PORT1_VECTOR       // Using PORT1_VECTOR interrupt because P1.4 and P1.5 are in port 1
__interrupt
void PORT1_ISR(void)
{
    GPIO_disableInterrupt(GPIO_PORT_P1, GPIO_PIN6);
    uint8_t ultraVal = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN6);
    uint8_t echoStatus = GPIO_getInterruptStatus(GPIO_PORT_P1, GPIO_PIN6);

    if (ultraVal == GPIO_INPUT_PIN_HIGH) {
        //start timer
        //showChar('1', 5);
        Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);

        // set the interrupt to trigger on low
        GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
    }
    else {
        //Stop timer
        Timer_A_stop(TIMER_A1_BASE);
        // get the value from the timer
        timerVal = Timer_A_getCounterValue(TIMER_A1_BASE);

        //flag to say that we have received data
        flag = 1;

        //clear the timer
        Timer_A_clear(TIMER_A1_BASE);

        // change the interrupt edge to trigger on high
        GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN6, GPIO_LOW_TO_HIGH_TRANSITION);

    }

    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN6);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN6);
}

//Button interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt
void PORT2_ISR(void)
{
    uint8_t buttonStatus = GPIO_getInterruptStatus(GPIO_PORT_P2, GPIO_PIN6);
    if (buttonStatus)
    {
        inputFlag = 1;
    }
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);
}
