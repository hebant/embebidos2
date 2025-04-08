//PWM PF1 1 solo sentido

// Librerías
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
#include "utils/uartstdio.c"

#define SYS_CLOCK_FREQ 120000000

// Pines del puente H
#define IN1 GPIO_PIN_6  // PA6
#define IN2 GPIO_PIN_7  // PA7

volatile int duty_cycle=1;

void PWM(int Duty);
void Delay(uint32_t);
void Motor_Adelante(void);
void Motor_Apagado(void);

int main(void)
{
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, SYS_CLOCK_FREQ);

    // PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 1700); // Frecuencia PWM
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, duty_cycle);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

    // Pines de control del motor
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, IN1 | IN2);

    Motor_Adelante();  // Solo una vez al inicio

    while(1)
    {
        PWM(20); // Velocidad baja
        Delay(3000);

        PWM(50); // Velocidad media
        Delay(3000);

        PWM(100); // Velocidad máxima
        Delay(3000);

        PWM(0); // Motor parado
        Delay(3000);
    }
}

void PWM(int Duty){
    if (Duty >= 100) Duty = 100;
    if (Duty <= 0) Duty = 0;
    duty_cycle = (int)( Duty*(1700/100));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, duty_cycle+1);
}

void Delay(uint32_t ms){
    SysCtlDelay((SYS_CLOCK_FREQ/3000)*ms);
}

void Motor_Adelante(void){
    GPIOPinWrite(GPIO_PORTA_BASE, IN1 | IN2, IN1); // IN1=1, IN2=0
}

void Motor_Apagado(void){
    GPIOPinWrite(GPIO_PORTA_BASE, IN1 | IN2, 0); // IN1=0, IN2=0
}