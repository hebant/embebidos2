/*
 * Código para medir distancia con sensor ultrasónico HC-SR04
 * y enviar los datos por UART (a 9600 baudios) en la placa EK-TM4C1294XL.
 * Librerías utilizadas: TivaWare + uartstdio.c
 *
 * Conexiones:
 * - TRIG -> PB2
 * - ECHO -> PB3
 * - UART0 RX -> PA0
 * - UART0 TX -> PA1
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.c"

#define TRIG_PIN GPIO_PIN_2  // PB2
#define ECHO_PIN GPIO_PIN_3  // PB3

void delayMicroseconds(uint32_t us) {
    SysCtlDelay((SysCtlClockGet() / 3000000) * us);
}

void configurarUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTStdioConfig(0, 9600, 120000000);
}

void configurarSensor(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, TRIG_PIN);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, ECHO_PIN);
}

uint32_t medirDistancia(void) {
    uint32_t tiempo = 0;

    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, 0);
    delayMicroseconds(2);
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, TRIG_PIN);
    delayMicroseconds(10);
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, 0);

    // Esperar flanco de subida
    uint32_t timeout = 300000;
    while ((GPIOPinRead(GPIO_PORTB_BASE, ECHO_PIN) == 0) && timeout--);

    if (timeout == 0) return 0;

    // Medir duración del pulso
    tiempo = 0;
    timeout = 300000;
    while ((GPIOPinRead(GPIO_PORTB_BASE, ECHO_PIN) != 0) && timeout--) {
        delayMicroseconds(1);
        tiempo++;
    }

    if (timeout == 0) return 0;

    return (tiempo * 0.0343) / 2; // Convertir a cm
}

int main(void) {
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                        SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    configurarUART();
    configurarSensor();

    while (1) {
        uint32_t distancia = medirDistancia();
        UARTprintf("Distancia: %d cm\n", distancia);
        SysCtlDelay(SysCtlClockGet() / 10); // Delay ~100ms
    }
}
