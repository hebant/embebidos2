//Librerias

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "utils/uartstdio.c"


//variables
#define SYS_CLOCK_FREQ 120000000




int width;
volatile int duty_cycle=1;

void PWM(int Duty);
void Delay(uint32_t);

int Val_PWM = 0;

int main(void)
{
  SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, SYS_CLOCK_FREQ);


    //Enable PWM0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE,0x02);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    //Pulsos=120,000,000/Freq_deseada
    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_0,1700);
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,duty_cycle);
    PWMGenEnable(PWM0_BASE,PWM_GEN_0);
    PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT,true);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, 0x11);

    
    // Check if the peripheral access is enabled.

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
    {
    }

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, 0x11);


     while(1)
    {
        PWM(100);
        Delay(1000);
        PWM(50);
        Delay(1000);
        PWM(0);
        Delay(1000);
    } 
}
//
void PWM (int Duty){
    if (Duty >= 100) Duty = 100;
    if (Duty <= 0) Duty = 0;
    duty_cycle = (int)( Duty*(1700/100));
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,duty_cycle+1);
  }

  void Delay(uint32_t ms){
      SysCtlDelay((SYS_CLOCK_FREQ/3000)*ms);
  }
  