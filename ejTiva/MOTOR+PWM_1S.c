// =====================================================================================
// Código para Tiva C EK-TM4C1294XL
// Funcionalidad:
// Control de motor en un solo sentido utilizando PWM en el pin PF1.
// Se controla el motor con un puente H (solo un sentido de giro) y se modula la
// velocidad mediante PWM (3 velocidades: baja, media, máxima, y luego apagado).
//
// Pines utilizados:
// PWM salida: PF1 (M0PWM1)
// IN1 -> PA6 (GPIO digital para el puente H)
// IN2 -> PA7 (GPIO digital para el puente H)
// =====================================================================================

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
#include "utils/uartstdio.c"

#define SYS_CLOCK_FREQ 120000000

// Pines del puente H
#define IN1 GPIO_PIN_6  // PA6
#define IN2 GPIO_PIN_7  // PA7

volatile int duty_cycle = 1;

// Prototipos de funciones
void PWM(int Duty);
void Delay(uint32_t ms);
void Motor_Adelante(void);
void Motor_Apagado(void);

int main(void)
{
    // =================================================================================
    // Configuración del sistema
    // =================================================================================

    // Configurar el reloj del sistema a 120 MHz usando PLL
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                       SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, SYS_CLOCK_FREQ);

    // =================================================================================
    // Configuración del PWM en PF1 (M0PWM1)
    // =================================================================================
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);    // Habilitar módulo PWM0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);   // Habilitar Puerto F

    // Configurar PF1 como salida PWM
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);

    // Configurar generador PWM0_GEN_0 en modo descendente sin sincronización
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Establecer periodo del PWM (frecuencia)
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 1700);

    // Establecer ciclo de trabajo inicial
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, duty_cycle);

    // Habilitar generador y salida PWM
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);

    // =================================================================================
    // Configuración de pines de control del motor (puente H)
    // =================================================================================
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);  // Puerto A para IN1 e IN2
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, IN1 | IN2);

    // Activar motor en un solo sentido al inicio
    Motor_Adelante();

    // =================================================================================
    // Bucle principal: cambiar velocidad del motor cada 3 segundos
    // =================================================================================
    while(1)
    {
        PWM(20);    // Velocidad baja
        Delay(3000);

        PWM(50);    // Velocidad media
        Delay(3000);

        PWM(100);   // Velocidad máxima
        Delay(3000);

        PWM(0);     // Motor apagado
        Delay(3000);
    }
}

// =====================================================================================
// Función: PWM
// Parámetro: Duty (0-100%)
// Establece el ciclo de trabajo para la señal PWM en PF1
// =====================================================================================
void PWM(int Duty){
    if (Duty >= 100) Duty = 100;
    if (Duty <= 0) Duty = 0;

    duty_cycle = (int)(Duty * (1700 / 100));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, duty_cycle + 1);
}

// =====================================================================================
// Función: Delay
// Parámetro: ms (milisegundos)
// Implementa un retardo bloqueante utilizando SysCtlDelay()
// =====================================================================================
void Delay(uint32_t ms){
    SysCtlDelay((SYS_CLOCK_FREQ / 3000) * ms);
}

// =====================================================================================
// Función: Motor_Adelante
// Activa el motor para que gire en un solo sentido (IN1 = 1, IN2 = 0)
// =====================================================================================
void Motor_Adelante(void){
    GPIOPinWrite(GPIO_PORTA_BASE, IN1 | IN2, IN1); // IN1 = 1, IN2 = 0
}

// =====================================================================================
// Función: Motor_Apagado
// Apaga el motor (IN1 = 0, IN2 = 0)
// =====================================================================================
void Motor_Apagado(void){
    GPIOPinWrite(GPIO_PORTA_BASE, IN1 | IN2, 0);
}
