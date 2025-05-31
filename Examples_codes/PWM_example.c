/*
 * ================================================================================================
 * Código para EK-TM4C1294XL que genera una señal PWM en el pin PF1 usando el módulo PWM0.
 * 
 * Funcionalidad:
 * - Configura el sistema a 120 MHz.
 * - Usa el pin PF1 como salida PWM (M0PWM1).
 * - El ciclo de trabajo (duty cycle) cambia cada segundo entre 100%, 50% y 0%.
 * - Se usa una función llamada `PWM(int Duty)` para ajustar el ciclo de trabajo deseado (0 a 100%).
 * 
 * Conexiones utilizadas:
 * - PWM0 salida 1 (M0PWM1) en el pin PF1
 * - GPIO PF0 y PF4 se configuran como salida, aunque no se usan directamente.
 * 
 * Comentarios:
 * - El periodo del PWM está definido como 1700 ciclos (correspondiente a una frecuencia deseada).
 * - El ciclo de trabajo se calcula como Duty% de esos 1700 ciclos.
 * - La función Delay introduce pausas de 1 segundo entre los cambios de ciclo de trabajo.
 * - Se usa `PWMGenConfigure` en modo descendente sin sincronización.
 * 
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
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "utils/uartstdio.c"

// Constante de frecuencia del sistema
#define SYS_CLOCK_FREQ 120000000

// Variables globales
int width;
volatile int duty_cycle = 1; // Ciclo de trabajo inicial
int Val_PWM = 0;

// Prototipos de funciones
void PWM(int Duty);
void Delay(uint32_t);

// Función principal
int main(void)
{
    // Configurar frecuencia del sistema a 120 MHz
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, SYS_CLOCK_FREQ);

    // Habilitar el módulo PWM0 y el puerto F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Configurar el pin PF1 para función PWM0 salida 1 (M0PWM1)
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, 0x02); // PF1 como salida PWM

    // Configurar el generador de PWM0 en modo descendente sin sincronización
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Establecer el periodo del PWM (frecuencia = 120 MHz / 1700 ≈ 70.5 kHz)
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 1700);

    // Establecer ciclo de trabajo inicial
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, duty_cycle);

    // Habilitar generador y salida del PWM
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

    // Configurar pines PF0 y PF4 como salida (aunque no se usan directamente)
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, 0x11);

    // Esperar hasta que los periféricos estén listos
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) {}

    // Bucle principal: cambiar ciclo de trabajo cada segundo
    while(1)
    {
        PWM(100);   // 100% de ciclo de trabajo (máxima potencia)
        Delay(1000); // Esperar 1 segundo

        PWM(50);    // 50% de ciclo de trabajo
        Delay(1000);

        PWM(0);     // 0% de ciclo de trabajo (apagado)
        Delay(1000);
    } 
}

// Función que ajusta el ciclo de trabajo del PWM
void PWM(int Duty){
    if (Duty >= 100) Duty = 100;
    if (Duty <= 0) Duty = 0;

    // Calcular el número de ciclos para el duty cycle deseado
    duty_cycle = (int)(Duty * (1700 / 100)); // 1700 es el periodo
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, duty_cycle + 1); // Se suma 1 para evitar 0 absoluto
}

// Función para generar retardo en milisegundos
void Delay(uint32_t ms){
    SysCtlDelay((SYS_CLOCK_FREQ / 3000) * ms);
}
