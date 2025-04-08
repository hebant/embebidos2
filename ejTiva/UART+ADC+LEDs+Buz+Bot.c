/*
===============================================================================
Nombre: Proyecto Final - UART + ADC + LEDs + Buzzer + Botones
Autor: Heber
Fecha: Abril 2025
Placa: EK-TM4C1294XL
Descripción:
    Este código realiza la integración de varios módulos de la Tiva C:

    - Lee un potenciómetro conectado al pin PK3 mediante el ADC (Canal AIN19).
    - Envía el valor leído por UART a 9600 baudios.
    - Enciende los LEDs (Puerto N y Puerto F) y Activa un buzzer si el valor ADC es mayor a 2047.
    - Tiene dos botones conectados al Puerto J:
        * Si se presiona el botón PJ0, envía el mensaje "motor1" por UART.
        * Si se presiona el botón PJ1, envía el mensaje "motor2" por UART.
    - Además, si se recibe un mensaje por UART, se concatena " desde Tiva" al mensaje recibido y se reenvía.

Configuraciones importantes:
    - Frecuencia del reloj del sistema: 120 MHz.
    - Comunicación UART por pines PA0 (RX) y PA1 (TX).
    - Lectura ADC en PK3 (AIN19).
    - LEDs controlado por puertos N y F.
    - Buzzer conectado a PN4 
    - Botones conectados en PJ0 y PJ1 con resistencias pull-up internas activadas.

===============================================================================
*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

#include "utils/uartstdio.c"

char data[100]; // Buffer para UART
uint32_t ui32ADCValue; // Valor ADC

int main(void)
{
    // Configuración de reloj 120 MHz
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    // Habilitar Periféricos
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);  // UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);  // UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);  // Botones
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);   // ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);  // ADC Entrada
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);  // LEDs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  // LEDs

    // Configuración UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 9600, 120000000);

    // Botones PJ0 y PJ1
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // ADC en PK3 (AIN19)
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH19 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);

    // LEDs
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    // Habilitar y configurar buzzer en PN4
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);  // Ya estaba habilitado por los LEDs
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_4);


    while(1)
    {
        // UART Recepción
        if(UARTCharsAvail(UART0_BASE))
        {
            UARTgets(data, sizeof(data));
            strcat(data, " desde Tiva\n");
            UARTprintf(data);
        }

        // Botón PJ0
        if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == 0)
        {
            UARTprintf("motor1\n");
            SysCtlDelay(12000000);
        }

        // Botón PJ1
        if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) == 0)
        {
            UARTprintf("motor2\n");
            SysCtlDelay(12000000);
        }

        // Lectura ADC
        ADCProcessorTrigger(ADC0_BASE, 3);
        while(!ADCIntStatus(ADC0_BASE, 3, false));
        ADCIntClear(ADC0_BASE, 3);
        ADCSequenceDataGet(ADC0_BASE, 3, &ui32ADCValue);

        // Enviar por UART el valor ADC
        UARTprintf("ADC Value: %d\n", ui32ADCValue);

        // Control de LEDs
        if(ui32ADCValue > 2047)
        {
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0 | GPIO_PIN_1);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_PIN_0 | GPIO_PIN_4);
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, GPIO_PIN_4);  // Encender Buzzer
        }
        else
        {
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0);
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0);  // Apagar Buzzer
        }

        SysCtlDelay(4000000); // Pequeña pausa
    }
}