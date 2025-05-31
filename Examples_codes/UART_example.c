/*
 * ================================================================================================
 * Código para EK-TM4C1294XL que recibe texto por UART y lo reenvía concatenado con un mensaje.
 * 
 * Funcionalidad:
 * - Configura UART0 (pines PA0 y PA1) a 115200 baudios.
 * - Recibe cadenas de texto por UART desde una terminal o dispositivo conectado.
 * - Al recibir un mensaje, lo concatena con el texto " desde Tiva \n" y lo reenvía por UART.
 * 
 * Conexiones utilizadas:
 * - UART RX: PA0
 * - UART TX: PA1
 * - LED: PN0 (configurado como salida, aunque no se usa)
 * - Botón: PJ0 (configurado como entrada con resistencia pull-up, no se utiliza en este código)
 * 
 * Comentarios:
 * - Se utiliza `UARTgets()` para leer cadenas completas desde UART.
 * - La función `UARTprintf()` imprime el mensaje modificado.
 * - Se usa `uartstdio.c` para simplificar la comunicación UART.
 * - El LED y el botón están inicializados pero no se usan en este programa.
 * ================================================================================================
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.c"

// Buffer para almacenar datos recibidos
char data[100];

// Función de error para depuración
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

int main(void)
{
    // Configura el sistema a 120 MHz
    SysCtlClockFreqSet(
        SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
        SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    // Habilita los periféricos necesarios
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);  // Para LED PN0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);  // Para UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);  // Para botón PJ0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);  // UART0

    // Configura pines PA0 y PA1 como UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, 0x03); // PA0 y PA1

    // Configura PN0 como salida (LED) y PJ0 como entrada con pull-up (botón)
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0); // LED
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);  // Botón
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // Pull-up

    // Inicializa UART a 115200 baudios
    UARTStdioConfig(0, 115200, 120000000);

    // Bucle principal
    while(1)
    {
        // Si hay caracteres disponibles por UART
        if (UARTCharsAvail(UART0_BASE)) {
            UARTgets(data, sizeof(data));  // Leer la cadena
            strcat(data, " desde Tiva \n"); // Concatenar mensaje
            UARTprintf(data);              // Enviar cadena por UART
        }
    }
}
