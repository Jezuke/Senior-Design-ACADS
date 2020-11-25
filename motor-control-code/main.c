#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include <driverlib/sysctl.h>
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"

#define PWM_FREQ 50e3 //PWM Frequency 50KHz
#define FCY 40e6 //Clock Frequency 40MHz

//Output PWM bits (PWM_OUT_X_BIT)
#define PWM0_AH 1
#define PWM1_AL 2
#define PWM2_BH 4
#define PWM3_BL 8
#define PWM4_CH 16
#define PWM5_CL 32

#define PWM_PERIOD (FCY/(2*PWM_FREQ)) //~= 800

// QEI Position
volatile int qeiPosition;

void initHW()
{
    //Set the clock
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //40 Mhz
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOA); //Enable Port A (Hall sensor input)


    // Configure PA2 Input pins
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);

    // Configure PA5 as an Output pin
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);
}

void initPWM()
{
    //Configure PWM Clock to match system

    SysCtlPWMClockSet (SYSCTL_PWMDIV_1); //Use same frequency as Sys_Clk (40Mhz)

    // Enable the peripherals used by this program.
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable (SYSCTL_PERIPH_PWM0); //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pinsboar

    GPIOPinConfigure (GPIO_PB6_M0PWM0);
    GPIOPinConfigure (GPIO_PB7_M0PWM1);
    GPIOPinConfigure (GPIO_PB4_M0PWM2);
    GPIOPinConfigure (GPIO_PB5_M0PWM3);
    GPIOPinConfigure (GPIO_PE4_M0PWM4);
    GPIOPinConfigure (GPIO_PE5_M0PWM5);


    //Configure PB6,PB7,PB4, PE4, PE5 Pins as PWM
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_4 | GPIO_PIN_5); //PWM 0,1,2,3 (In order)
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5); //PWM 4 and 5

    //Configure PWM Options
    //PWM_GEN_2 Covers M1PWM4 and M1PWM5
    //PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //Setting
    //Set the Period (expressed in clock ticks)
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWM_PERIOD);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, PWM_PERIOD);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, PWM_PERIOD);

    //Set PWM duty cycle initially to 0
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 0);

    // Enable the PWM generator
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    //Invert the output
//    PWMOutputInvert(PWM0_BASE, PWM0_AH | PWM1_AL | PWM2_BH | PWM3_BL | PWM4_CH | PWM5_CL , false);
}

//Sets PWM Duty Cycle
 void setPWM_Duty(uint8_t Duty)
 {
     PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (PWM_PERIOD*Duty)/100);
     PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, (PWM_PERIOD*Duty)/100);
     PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (PWM_PERIOD*Duty)/100);
     PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, (PWM_PERIOD*Duty)/100);
     PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, (PWM_PERIOD*Duty)/100);
     PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (PWM_PERIOD*Duty)/100);
 }

 // Code from: https://forum.43oh.com/topic/7170-using-harware-qei-on-tiva-launchpad/
 void initQEI()
 {
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); // Enable Port D (QEI)
     SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0); // Enable QEI Module 0

     //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
      HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
      HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
      HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

      //Set Pins to be PHA0 and PHB0
      GPIOPinConfigure(GPIO_PD6_PHA0);
      GPIOPinConfigure(GPIO_PD7_PHB0);

      //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7.
      GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);

      //Disable peripheral and interrupt before configuration
      QEIDisable(QEI0_BASE);
      QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

      // Configure quadrature encoder, use an arbitrary top limit of 1000
      QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 4096);

      // Enable the quadrature encoder.
      QEIEnable(QEI0_BASE);

      //Set position to a middle value so we can see if things are working
      QEIPositionSet(QEI0_BASE, 500);
 }

uint8_t HallA = 0, HallB = 0, HallC = 0; // For debugging purposes
void main(void)
{
    initHW();
    initPWM();

    //Set Duty Cycle (0 - 100)
    setPWM_Duty(90);
    // Turn on the Output pins
    PWMOutputState(PWM0_BASE, PWM0_AH | PWM1_AL | PWM2_BH | PWM3_BL | PWM4_CH | PWM5_CL, true);

    //Do nothing
    while (1)
    {
        HallA = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2);
        HallB = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3);
        HallC = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4);

        if (HallA && HallC && !(HallB))
        {
            PWMOutputState(PWM0_BASE, PWM0_AH | PWM3_BL, true);
            PWMOutputState(PWM0_BASE, PWM1_AL | PWM2_BH | PWM4_CH | PWM5_CL, false);
        }
        else if (HallA && !(HallB && HallC))
        {
            PWMOutputState(PWM0_BASE, PWM0_AH | PWM5_CL, true);
            PWMOutputState(PWM0_BASE, PWM1_AL | PWM2_BH | PWM3_BL | PWM4_CH, false);
        }
        else if (HallA && HallB && !(HallC))
        {
            PWMOutputState(PWM0_BASE, PWM2_BH | PWM5_CL, true);
            PWMOutputState(PWM0_BASE, PWM0_AH | PWM1_AL | PWM3_BL | PWM4_CH, false);
        }
        else if (HallB && !(HallA && HallC))
        {
            PWMOutputState(PWM0_BASE, PWM1_AL | PWM2_BH, true);
            PWMOutputState(PWM0_BASE, PWM0_AH | PWM3_BL | PWM4_CH | PWM5_CL, false);
        }
        else if (HallB && HallC && !(HallA))
        {
            PWMOutputState(PWM0_BASE, PWM1_AL | PWM4_CH, true);
            PWMOutputState(PWM0_BASE, PWM0_AH | PWM2_BH | PWM3_BL | PWM5_CL, false);
        }
        else if (HallC && !(HallA && HallB))
        {
            PWMOutputState(PWM0_BASE, PWM3_BL | PWM4_CH, true);
            PWMOutputState(PWM0_BASE, PWM0_AH | PWM1_AL | PWM2_BH | PWM5_CL, false);
        }
    }
}
