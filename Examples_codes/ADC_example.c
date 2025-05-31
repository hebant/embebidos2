/*
 * ===========================================================================================
 * Código para EK-TM4C1294XL que realiza lectura ADC y control de LEDs basado en el valor leído.
 * Además configura UART0 para comunicación serial y un timer para enviar datos periódicamente.
 * 
 * Funcionalidad:
 * - Configura el sistema a 120 MHz.
 * - Inicializa UART0 a 9600 baudios usando los pines PA0 (RX) y PA1 (TX).
 * - Configura el Timer0 para generar interrupciones periódicas.
 * - Configura el ADC0 canal 19 (PK3) para lectura analógica.
 * - En el ciclo principal, se ejecuta la lectura ADC y dependiendo del valor:
 *      - Si es mayor a 2000, enciende los LEDs conectados a PN0 y PN1.
 *      - Si es menor o igual a 2000, apaga esos LEDs.
 * - En la interrupción del timer se convierte el valor ADC a cadena y lo envía por UART.
 * 
 * Conexiones utilizadas:
 * - UART0 RX: PA0
 * - UART0 TX: PA1
 * - ADC0 canal 19: PK3 (pin analógico)
 * - LEDs: PN0 y PN1
 * - Otros pines GPIOF y GPION configurados para salida (PF0 y PF4)
 * 
 * Comentarios:
 * - Se usa la librería uartstdio para facilitar la comunicación UART.
 * - El buffer 'data_out' se usa para convertir el valor ADC a string.
 * - La función Delay usa SysCtlDelay para retardos en ms.
 * ===========================================================================================
 */

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
#include "driverlib/adc.h"

#define SYS_CLOCK_FREQ 120000000
#define BUFFER_SIZE 128

char data[BUFFER_SIZE];
char data_out[BUFFER_SIZE];
uint32_t lectura = 0;
uint32_t FS1 = SYS_CLOCK_FREQ * 0.5; // Frecuencia para timer (50% del reloj)

void Delay(uint32_t);
void timer0A_handler(void);
char *utoa(long value, char * str, int base);

int main(void)
{
    // Configurar reloj del sistema a 120 MHz
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, SYS_CLOCK_FREQ);

    // Habilitar periféricos UART0, Timer0 y GPIOA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configurar pines PA0 y PA1 para UART0 RX y TX
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, 0x03);

    // Configurar UART0 a 9600 baudios
    UARTStdioConfig(0, 9600, SYS_CLOCK_FREQ);

    // Configurar Timer0 en modo periódico
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, FS1);

    // Habilitar interrupciones globales y del Timer0A
    IntMasterEnable();
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);

    // Habilitar periféricos ADC0, GPIOK, GPION y GPIOF
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Esperar hasta que los periféricos estén listos
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)) {}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)) {}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {}

    // Configurar ADC0 secuencia 3, disparo por procesador, paso 0 lectura canal 19 con interrupción y fin de secuencia
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH19);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);

    // Configurar pines PN0 y PN1 como salida (para LEDs)
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, 0x03);

    // Configurar pines PF0 y PF4 como salida (probablemente LEDs u otros)
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, 0x11);

    // Configurar pin PK3 como entrada ADC
    GPIOPinTypeADC(GPIO_PORTK_BASE, 0x08);

    // Limpiar la pantalla de terminal serial
    UARTprintf("\033[2J");

    // Bucle principal
    while(1)
    {
        // Iniciar conversión ADC por software
        ADCProcessorTrigger(ADC0_BASE, 3);

        // Esperar hasta que la conversión termine
        while(!ADCIntStatus(ADC0_BASE, 3, false)) {}

        // Limpiar bandera de interrupción ADC
        ADCIntClear(ADC0_BASE, 3);

        // Obtener dato leído por ADC
        ADCSequenceDataGet(ADC0_BASE, 3, &lectura);

        // Si lectura es mayor a 2000, encender LEDs en PN0 y PN1
        if(lectura > 2000)
        {
            GPIOPinWrite(GPIO_PORTN_BASE, 0x03, 0x03);
        }
        else
        {
            // Apagar LEDs PN0 y PN1
            GPIOPinWrite(GPIO_PORTN_BASE, 0x03, 0x00);
        }
    }
}

// Función para generar retraso en milisegundos
void Delay(uint32_t ms)
{
    SysCtlDelay((SYS_CLOCK_FREQ / 3000) * ms);
}

// Handler de interrupción del Timer0A
void timer0A_handler(void)
{
    // Limpiar bandera de interrupción
    TimerIntClear(TIMER0_BASE, TIMER_A);

    // Convertir valor ADC a cadena en base decimal
    utoa(lectura, data_out, 10);

    // Enviar valor por UART
    UARTprintf("Recibido: %s\n", data_out);
}
