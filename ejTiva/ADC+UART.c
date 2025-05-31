// =====================================================================================
// Código para Tiva C EK-TM4C1294XL
// Funcionalidad:
// Lee un valor analógico mediante el ADC (canal PK3) y lo muestra por UART.
// Además, enciende o apaga los LEDs del puerto N (PN0 y PN1) según el valor leído.
// El Timer0A genera interrupciones periódicas para imprimir el valor del ADC por UART.
//
// Conexiones utilizadas:
// UART0 - PA0 (RX), PA1 (TX)
// ADC0 - PK3 (Entrada analógica)
// LEDs - PN0 y PN1
// =====================================================================================

// Librerías
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
#include "driverlib/adc.h"
#include "utils/uartstdio.c"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Definiciones
#define SYS_CLOCK_FREQ 120000000  // Frecuencia del sistema (120 MHz)
#define BUFFER_SIZE 128           // Tamaño del buffer de salida UART

// Variables Globales
char data_out[BUFFER_SIZE];       // Buffer para salida UART
uint32_t lectura = 0;             // Variable donde se almacena la lectura del ADC

// Prototipos de funciones
void ConfigurarUART(void);        // Inicializa UART0
void ConfigurarADC(void);         // Inicializa ADC0 en canal PK3
void ConfigurarTimer(void);       // Inicializa Timer0A en modo periódico
void LeerADC(void);               // Realiza una lectura desde el ADC
void timer0A_handler(void);       // Manejador de interrupción para el Timer0A

// =====================================================================================
// Función Principal
// =====================================================================================
int main(void) {
    // Configuración del reloj del sistema a 120 MHz usando PLL y cristal de 25 MHz
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, SYS_CLOCK_FREQ);
    
    // Inicialización de UART, ADC y Timer
    ConfigurarUART();
    ConfigurarADC();
    ConfigurarTimer();

    // Limpiar pantalla del terminal UART y mensaje inicial
    UARTprintf("\033[2J");
    UARTprintf("\nSistema iniciado...\n");
    
    while (1) {
        LeerADC();  // Lectura continua del valor analógico
        
        // Control de LEDs del puerto N dependiendo del valor del ADC
        if (lectura > 2000) {
            GPIOPinWrite(GPIO_PORTN_BASE, 0x03, 0x03); // Enciende PN0 y PN1
        } else {
            GPIOPinWrite(GPIO_PORTN_BASE, 0x03, 0x00); // Apaga PN0 y PN1
        }
        
        // Delay aproximado de 100 ms
        SysCtlDelay(SYS_CLOCK_FREQ / 3 / 10);
    }
}

// =====================================================================================
// Configuración de UART0 (PA0 -> RX, PA1 -> TX) USB Rasp - Tiva
// =====================================================================================
void ConfigurarUART(void) {
    // Habilita el periférico UART0 y el puerto A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    
    // Configura los pines PA0 y PA1 como UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    // Inicializa UART0 a 9600 baudios
    UARTStdioConfig(0, 9600, SYS_CLOCK_FREQ);
}

// =====================================================================================
// Configuración de ADC0 en Secuencia 3 para leer PK3 (canal 19)
// =====================================================================================
void ConfigurarADC(void) {
    // Habilita el puerto K y el módulo ADC0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    
    // Configura el pin PK3 como entrada analógica
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);
    
    // Configura la secuencia 3 del ADC para lectura por software
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    // Paso 0: canal 19 (PK3), con interrupción y fin de secuencia
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH19 | ADC_CTL_IE | ADC_CTL_END);
    // Habilita la secuencia 3
    ADCSequenceEnable(ADC0_BASE, 3);
    // Limpia la bandera de interrupción
    ADCIntClear(ADC0_BASE, 3);
}

// =====================================================================================
// Configuración del Timer0A en modo periódico
// =====================================================================================
void ConfigurarTimer(void) {
    // Habilita el periférico del Timer0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    // Configura el Timer0A en modo periódico
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    // Establece el periodo de interrupción: 0.5 segundos
    TimerLoadSet(TIMER0_BASE, TIMER_A, SYS_CLOCK_FREQ / 2);
    
    // Habilita interrupciones globales y del Timer0A
    IntMasterEnable();
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

// =====================================================================================
// Lectura del valor ADC
// =====================================================================================
void LeerADC(void) {
    // Inicia la conversión ADC manualmente
    ADCProcessorTrigger(ADC0_BASE, 3);
    // Espera a que termine la conversión
    while (!ADCIntStatus(ADC0_BASE, 3, false));
    // Limpia bandera de interrupción
    ADCIntClear(ADC0_BASE, 3);
    // Obtiene el resultado de la conversión
    ADCSequenceDataGet(ADC0_BASE, 3, &lectura);
}

// =====================================================================================
// Rutina de Interrupción del Timer0A
// =====================================================================================
void timer0A_handler(void) {
    // Limpia la bandera de interrupción
    TimerIntClear(TIMER0_BASE, TIMER_A);
    // Realiza una lectura del ADC
    LeerADC();
    // Imprime el valor leído por UART
    UARTprintf("Valor ADC: %d\n", lectura);
}
