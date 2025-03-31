//Librerias
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.c"
#include "stdio.h"
#include "stdlib.h"
#include <string.h>
//#include <stdio.h>
//#include <stdlib.h>
#include "driverlib/adc.h"


#define SYS_CLOCK_FREQ 120000000
#define BUFFER_SIZE 128
char data[BUFFER_SIZE];
int doble;
char data_out[BUFFER_SIZE];
uint32_t FS1=SYS_CLOCK_FREQ*0.5;
uint32_t lectura=0;
//Funciones
void Delay(uint32_t);
// add the timer in startupp 
void timer0A_handler(void);
char * utoa(long value, char * str, int base);
//Main

int main(void)
{
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, SYS_CLOCK_FREQ);
    //UART
    //Enable the UART Periph
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //Enable module 8 receive
    GPIOPinConfigure(GPIO_PA0_U0RX);
    //Enable module 8 transmit
    GPIOPinConfigure(GPIO_PA1_U0TX);
    //Define UART Pins
    GPIOPinTypeUART(GPIO_PORTA_BASE,0x03);
    //Configures the UART communication (UART periph, clock freq, baud rate)
    UARTStdioConfig(0,9600,SYS_CLOCK_FREQ);
    
    // Enable Timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, FS1);
    IntMasterEnable();
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
    
    //Enable ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    
    // Enable the GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    
    // Check if the peripheral access is enabled.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }
    
    ADCSequenceConfigure(ADC0_BASE,3,ADC_TRIGGER_PROCESSOR,0);
    ADCSequenceStepConfigure(ADC0_BASE,3,0,ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH19);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE,3);
    
    //Declaracion de pines
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, 0x03);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, 0x11);
    GPIOPinTypeADC(GPIO_PORTK_BASE, 0x08); //ADC pin
    //Limpiar pantalla
    UARTprintf("\033[2J");
    while(1)
    {
    	ADCProcessorTrigger(ADC0_BASE,3);
	while(!ADCIntStatus(ADC0_BASE,3,false))
	{
	}
	ADCIntClear(ADC0_BASE,3);
	ADCSequenceDataGet(ADC0_BASE,3,&lectura);
	if(lectura > 2000)
	{
		GPIOPinWrite(GPIO_PORTN_BASE, 0x03, 0x03);
	}
	else
	{
		GPIOPinWrite(GPIO_PORTN_BASE, 0x03, 0x00);
	}
    }
}
void Delay(uint32_t ms){
	SysCtlDelay((SYS_CLOCK_FREQ/3000)*ms);
}

void timer0A_handler(void){
    TimerIntClear(TIMER0_BASE, TIMER_A);
    utoa(lectura, data_out,10); //Base numerica (decimal)
    UARTprintf("Recibido: %s\n",data_out);
}
