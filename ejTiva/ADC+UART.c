// =====================================================================================
// Código para Tiva C EK-TM4C1294XL
// Funcionalidad:
// Lee un valor analógico mediante el ADC (canal PK3) y lo muestra por UART.
// Además, enciende o apaga los LEDs del puerto N (PN0 y PN1) según el valor leído.
// El Timer0A genera interrupciones periódicas para imprimir el valor del ADC por UART.

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
#define SYS_CLOCK_FREQ 120000000
#define BUFFER_SIZE 128

// Variables Globales
char data_out[BUFFER_SIZE];
uint32_t lectura = 0;

// Prototipos de funciones
void ConfigurarUART(void);
void ConfigurarADC(void);
void ConfigurarTimer(void);
void LeerADC(void);
void timer0A_handler(void);

// =====================================================================================
// Función Principal
// =====================================================================================
int main(void) {
    // Configuración del reloj del sistema
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, SYS_CLOCK_FREQ);
    
    // Inicialización de periféricos
    ConfigurarUART();
    ConfigurarADC();
    ConfigurarTimer();

    UARTprintf("\033[2J"); // Limpiar pantalla
    UARTprintf("\nSistema iniciado...\n");
    
    while (1) {
        LeerADC();
        
        // Control de LEDs según valor ADC
        if (lectura > 2000) {
            GPIOPinWrite(GPIO_PORTN_BASE, 0x03, 0x03); // Encender PN0 y PN1
        } else {
            GPIOPinWrite(GPIO_PORTN_BASE, 0x03, 0x00); // Apagar PN0 y PN1
        }
        
        SysCtlDelay(SYS_CLOCK_FREQ / 3 / 10); // Delay ~100ms
    }
}

// =====================================================================================
// Configuración de UART0 (PA0 -> RX, PA1 -> TX)
// =====================================================================================
void ConfigurarUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    UARTStdioConfig(0, 9600, SYS_CLOCK_FREQ);
}

// =====================================================================================
// Configuración de ADC0 en Secuencia 3 para leer PK3 (canal 19)
// =====================================================================================
void ConfigurarADC(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3); // PK3 como entrada ADC
    
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH19 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
}

// =====================================================================================
// Configuración del Timer0A en modo periódico
// =====================================================================================
void ConfigurarTimer(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SYS_CLOCK_FREQ / 2); // Periodo de 0.5s
    
    IntMasterEnable();
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

// =====================================================================================
// Lectura del valor ADC
// =====================================================================================
void LeerADC(void) {
    ADCProcessorTrigger(ADC0_BASE, 3);
    while (!ADCIntStatus(ADC0_BASE, 3, false));
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, &lectura);
}

// =====================================================================================
// Rutina de Interrupción del Timer0A
// =====================================================================================
void timer0A_handler(void) {
    TimerIntClear(TIMER0_BASE, TIMER_A);
    LeerADC();
    UARTprintf("Valor ADC: %d\n", lectura);
}
