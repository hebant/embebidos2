/*
 * Este codigo configura y utiliza la comunicacion UART en la EK-TM4C1294XL.
 * Recibe datos a traves de UART0, les agrega el mensaje " desde Tiva", y los reenvia.
 * Adicionalmente, cuando se presiona el boton de usuario 1, envia "motor1" por UART.
 * Cuando se presiona el boton de usuario 2, envia "motor2" por UART.
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

// Buffer para almacenar los datos recibidos por UART
char data[100]; 

int main(void)
{
    // Configurar la frecuencia del sistema a 120 MHz usando cristal de 25 MHz y PLL
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
    
    // Habilitar los perifericos UART0 y el puerto GPIOA para UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    
    // Habilitar GPIOJ para los botones de usuario
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    
    // Configurar los pines PA0 y PA1 para comunicacion UART0 (RX y TX)
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    // Configurar botones de usuario en PJ0 y PJ1 como entradas con resistencia pull-up interna
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
    // Configurar UART0 con una velocidad de 9600 baudios para la comunicación serial
    UARTStdioConfig(0, 9600, 120000000);
    
    while (1)
    {
        // Verificar si hay caracteres disponibles para leer en UART0
        if (UARTCharsAvail(UART0_BASE)) {
            UARTgets(data, 100);             // Leer los datos recibidos por UART (hasta 100 caracteres)
            strcat(data, " desde Tiva\n");  // Agregar el texto " desde Tiva" al final de la cadena recibida
            UARTprintf(data);                // Enviar el mensaje modificado de vuelta por UART
        }
        
        // Verificar si se presiona el boton de usuario 1 conectado a PJ0
        if (GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == 0) {
            UARTprintf("motor1\n");          // Enviar la cadena "motor1" por UART
            SysCtlDelay(12000000);           // Esperar un tiempo para evitar rebotes del botón (delay ~0.1s)
        }
        
        // Verificar si se presiona el boton de usuario 2 conectado a PJ1
        if (GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) == 0) {
            UARTprintf("motor2\n");          // Enviar la cadena "motor2" por UART
            SysCtlDelay(12000000);           // Esperar un tiempo para evitar rebotes del botón (delay ~0.1s)
        }
    }
}
