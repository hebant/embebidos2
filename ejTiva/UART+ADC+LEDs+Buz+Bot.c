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

char data[100]; // Buffer para almacenar datos recibidos por UART
uint32_t ui32ADCValue; // Variable para almacenar el valor leído por el ADC

int main(void)
{
    // Configuración del reloj del sistema a 120 MHz usando cristal de 25 MHz y PLL
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    // Habilitar periféricos necesarios:
    // GPIOA para UART0 (pines PA0 y PA1)
    // UART0 para comunicación serial
    // GPIOJ para los botones
    // ADC0 para lectura analógica
    // GPIOK para el pin ADC de entrada
    // GPION y GPIOF para LEDs y buzzer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);  // UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);  // UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);  // Botones
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);   // ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);  // ADC Entrada
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);  // LEDs y buzzer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  // LEDs

    // Configuración UART:
    // Configurar pines PA0 como RX y PA1 como TX para UART0
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Inicializar UART0 a 9600 baudios con reloj a 120 MHz
    UARTStdioConfig(0, 9600, 120000000);

    // Configuración de botones:
    // PJ0 y PJ1 como entradas digitales con resistencia pull-up interna activada
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Configuración ADC:
    // Configurar pin PK3 como entrada ADC (AIN19)
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);
    // Configurar el secuenciador 3 del ADC0 para muestrear el canal 19 con trigger por software
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    // Configurar el paso 0 del secuenciador para leer canal 19 y generar interrupción al terminar la conversión
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH19 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);

    // Configuración LEDs:
    // PN0 y PN1 como salida para LEDs
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // PF0 y PF4 como salida para LEDs
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    // Configuración buzzer:
    // PN4 como salida digital para buzzer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);  // Ya estaba habilitado antes
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_4);

    while(1)
    {
        // Verificar si hay datos disponibles en UART0 (recepción)
        if(UARTCharsAvail(UART0_BASE))
        {
            // Leer cadena recibida y guardarla en data
            UARTgets(data, sizeof(data));
            // Concatenar mensaje para identificar que viene desde la Tiva
            strcat(data, " desde Tiva\n");
            // Enviar la cadena modificada de vuelta por UART
            UARTprintf(data);
        }

        // Leer estado del botón en PJ0 (activo en nivel bajo)
        if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == 0)
        {
            // Enviar mensaje "motor1" por UART si se presiona botón PJ0
            UARTprintf("motor1\n");
            // Pequeña demora para evitar rebotes o lecturas múltiples
            SysCtlDelay(12000000);
        }

        // Leer estado del botón en PJ1 (activo en nivel bajo)
        if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) == 0)
        {
            // Enviar mensaje "motor2" por UART si se presiona botón PJ1
            UARTprintf("motor2\n");
            // Pequeña demora para evitar rebotes o lecturas múltiples
            SysCtlDelay(12000000);
        }

        // Inicio de conversión ADC en secuenciador 3 (software trigger)
        ADCProcessorTrigger(ADC0_BASE, 3);
        // Esperar hasta que la conversión termine
        while(!ADCIntStatus(ADC0_BASE, 3, false));
        // Limpiar la interrupción para nueva conversión
        ADCIntClear(ADC0_BASE, 3);
        // Obtener valor convertido y almacenarlo en ui32ADCValue
        ADCSequenceDataGet(ADC0_BASE, 3, &ui32ADCValue);

        // Enviar valor ADC leído por UART para monitorización
        UARTprintf("ADC Value: %d\n", ui32ADCValue);

        // Control de LEDs y buzzer según valor ADC:
        if(ui32ADCValue > 2047)
        {
            // Encender LEDs PN0, PN1 y PF0, PF4
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0 | GPIO_PIN_1);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_PIN_0 | GPIO_PIN_4);
            // Encender buzzer en PN4
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, GPIO_PIN_4);
        }
        else
        {
            // Apagar LEDs PN0, PN1 y PF0, PF4
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0);
            // Apagar buzzer en PN4
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0);
        }

        // Pausa corta para estabilizar lecturas y evitar saturación UART
        SysCtlDelay(4000000);
    }
}
