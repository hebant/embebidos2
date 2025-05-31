// =====================================================================================
// Código para Tiva C EK-TM4C1294XL
// Funcionalidad:
// - Lee un valor analógico mediante el ADC (canal PK3) y lo muestra por UART.
// - Enciende o apaga los LEDs del puerto N (PN0 y PN1) según el valor leído.
// - El Timer0A genera interrupciones periódicas para imprimir el valor del ADC por UART.
//
// Conexiones utilizadas:
// - UART0: PA0 (RX), PA1 (TX)
// - ADC0: PK3 (Entrada analógica)
// - LEDs: PN0 y PN1
// =====================================================================================

// Librerías necesarias
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

// Definición frecuencia del sistema
#define SYS_CLOCK_FREQ 120000000
#define BUFFER_SIZE 128

// Variables globales
char data_out[BUFFER_SIZE];  // Buffer para salida UART (no usado directamente aquí)
uint32_t lectura = 0;        // Variable donde se guarda el valor ADC leído

// Prototipos de funciones
void ConfigurarUART(void);
void ConfigurarADC(void);
void ConfigurarTimer(void);
void LeerADC(void);
void timer0A_handler(void);  // Manejador de interrupción del Timer0A

// =====================================================================================
// Función principal
// =====================================================================================
int main(void) {
    // Configura el reloj del sistema a 120 MHz (cristal de 25 MHz + PLL)
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, SYS_CLOCK_FREQ);
    
    // Inicializa los periféricos UART, ADC y Timer
    ConfigurarUART();
    ConfigurarADC();
    ConfigurarTimer();

    UARTprintf("\033[2J");  // Secuencia ANSI para limpiar pantalla terminal
    UARTprintf("\nSistema iniciado...\n");
    
    while (1) {
        // Lee el valor ADC
        LeerADC();
        
        // Control de LEDs PN0 y PN1: encender si lectura > 2000, apagar si no
        if (lectura > 2000) {
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0 | GPIO_PIN_1);
        } else {
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0x00);
        }
        
        // Retardo aproximadamente 100 ms para evitar lecturas muy rápidas
        SysCtlDelay(SYS_CLOCK_FREQ / 3 / 10);
    }
}

// =====================================================================================
// Configuración de UART0 (PA0 -> RX, PA1 -> TX) a 9600 baudios
// =====================================================================================
void ConfigurarUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);  // Habilitar UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);  // Habilitar GPIOA para UART0
    
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)); // Esperar listo UART0
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)); // Esperar listo GPIOA
    
    GPIOPinConfigure(GPIO_PA0_U0RX);  // Configurar PA0 como RX UART0
    GPIOPinConfigure(GPIO_PA1_U0TX);  // Configurar PA1 como TX UART0
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);  // Configurar pines UART
    
    UARTStdioConfig(0, 9600, SYS_CLOCK_FREQ);  // Inicializar UART0 con 9600 baudios
}

// =====================================================================================
// Configuración del ADC0 para leer el canal PK3 (canal 19) con secuencia 3
// =====================================================================================
void ConfigurarADC(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);   // Habilitar GPIOK
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);    // Habilitar ADC0
    
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);  // PK3 configurado como entrada analógica ADC
    
    // Configurar secuencia 3, trigger por procesador, canal 19 (PK3)
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH19 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
}

// =====================================================================================
// Configuración del Timer0A para interrupciones periódicas cada 0.5 segundos
// =====================================================================================
void ConfigurarTimer(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);    // Habilitar Timer0
    
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); // Configurar modo periódico
    TimerLoadSet(TIMER0_BASE, TIMER_A, SYS_CLOCK_FREQ / 2);  // Cargar valor para 0.5s
    
    IntMasterEnable();               // Habilitar interrupciones globales
    IntEnable(INT_TIMER0A);          // Habilitar interrupción del Timer0A
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);  // Habilitar interrupción por timeout
    TimerEnable(TIMER0_BASE, TIMER_A);                 // Iniciar Timer0A
}

// =====================================================================================
// Función para iniciar la conversión ADC y obtener el resultado
// =====================================================================================
void LeerADC(void) {
    ADCProcessorTrigger(ADC0_BASE, 3);          // Iniciar conversión en secuencia 3
    while (!ADCIntStatus(ADC0_BASE, 3, false)); // Esperar a que finalice conversión
    ADCIntClear(ADC0_BASE, 3);                   // Limpiar bandera de interrupción ADC
    
    ADCSequenceDataGet(ADC0_BASE, 3, &lectura); // Obtener valor ADC y guardarlo en 'lectura'
}

// =====================================================================================
// Manejador de interrupción Timer0A
// Se ejecuta cada 0.5 segundos para imprimir el valor ADC por UART
// =====================================================================================
void timer0A_handler(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);  // Limpiar bandera interrupción
    
    LeerADC();  // Leer valor ADC actualizado
    UARTprintf("Valor ADC: %d\n", lectura);  // Imprimir por UART
}
