/*
 * Este codigo configura y utiliza la comunicacion UART en la EK-TM4C1294XL.
 * Recibe datos a traves de UART0, les agrega el mensaje " desde Tiva", y los reenvia.
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

char data[100]; // Buffer para almacenar los datos recibidos por UART

int main(void)
{
    // Configurar la frecuencia del sistema a 120 MHz
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
    
    // Habilitar los perifericos UART0 y el puerto GPIOA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    
    // Configurar los pines PA0 y PA1 para comunicacion UART0
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    // Configurar UART0 con una velocidad de 9600 baudios
    UARTStdioConfig(0, 9600, 120000000);
    
    while (1)
    {
        // Verificar si hay caracteres disponibles en UART0
        if (UARTCharsAvail(UART0_BASE)) {
            UARTgets(data, 100); // Leer los datos recibidos
            strcat(data, " desde Tiva\n"); // Agregar mensaje al final del texto recibido
            UARTprintf(data); // Enviar el mensaje de vuelta por UART
        }
    }
}