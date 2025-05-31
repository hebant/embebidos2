/*
 * Código para medir distancia con sensor ultrasónico HC-SR04
 * y enviar los datos por UART (a 9600 baudios) en la placa EK-TM4C1294XL.
 * Librerías utilizadas: TivaWare + uartstdio.c
 *
 * Conexiones:
 * - TRIG -> PB2 (salida)
 * - ECHO -> PB3 (entrada)
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

#define TRIG_PIN GPIO_PIN_2  // Pin para TRIG del sensor HC-SR04, conectado a PB2
#define ECHO_PIN GPIO_PIN_3  // Pin para ECHO del sensor HC-SR04, conectado a PB3

// Función para retardos en microsegundos
// Calcula el número de ciclos de reloj para un retardo aproximado
void delayMicroseconds(uint32_t us) {
    // SysCtlDelay consume 3 ciclos por instrucción
    // Se ajusta el retardo para microsegundos multiplicando por (clock/3,000,000)
    SysCtlDelay((SysCtlClockGet() / 3000000) * us);
}

// Configura UART0 con pines PA0 (RX) y PA1 (TX) a 9600 baudios
void configurarUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);   // Habilitar GPIOA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);   // Habilitar UART0

    // Configurar multiplexor para pines UART0
    GPIOPinConfigure(GPIO_PA0_U0RX);  // PA0 como RX
    GPIOPinConfigure(GPIO_PA1_U0TX);  // PA1 como TX
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Inicializar UART0 con baudrate 9600 y reloj a 120 MHz
    UARTStdioConfig(0, 9600, 120000000);
}

// Configura pines para sensor HC-SR04: TRIG como salida y ECHO como entrada
void configurarSensor(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Habilitar GPIOB

    // TRIG (PB2) como salida digital
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, TRIG_PIN);
    // ECHO (PB3) como entrada digital
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, ECHO_PIN);
}

// Función para medir distancia usando sensor HC-SR04
// Retorna distancia en cm (uint32_t)
uint32_t medirDistancia(void) {
    uint32_t tiempo = 0;

    // Enviar pulso de trigger:
    // Primero mantener TRIG en bajo 2 us
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, 0);
    delayMicroseconds(2);

    // TRIG en alto 10 us para iniciar medición
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, TRIG_PIN);
    delayMicroseconds(10);

    // TRIG en bajo para esperar respuesta del sensor
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, 0);

    // Esperar flanco de subida en ECHO (pin PB3) con timeout
    uint32_t timeout = 300000;
    while ((GPIOPinRead(GPIO_PORTB_BASE, ECHO_PIN) == 0) && timeout--);
    if (timeout == 0) return 0;  // Timeout: no se detectó señal

    // Medir duración del pulso ECHO en microsegundos
    tiempo = 0;
    timeout = 300000;
    while ((GPIOPinRead(GPIO_PORTB_BASE, ECHO_PIN) != 0) && timeout--) {
        delayMicroseconds(1);
        tiempo++;
    }
    if (timeout == 0) return 0;  // Timeout: pulso muy largo o error

    // Calcular distancia en cm:
    // La velocidad del sonido es ~343 m/s = 0.0343 cm/us
    // Dividimos entre 2 porque el pulso es ida y vuelta
    return (tiempo * 0.0343) / 2;
}

int main(void) {
    // Configurar reloj del sistema a 120 MHz con cristal de 25 MHz y PLL
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                        SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    configurarUART();   // Inicializar UART0
    configurarSensor(); // Inicializar sensor ultrasónico

    while (1) {
        // Medir distancia con el sensor
        uint32_t distancia = medirDistancia();

        // Enviar la distancia medida por UART en formato legible
        UARTprintf("Distancia: %d cm\n", distancia);

        // Esperar aproximadamente 100 ms antes de siguiente medición
        SysCtlDelay(SysCtlClockGet() / 10);
    }
}
