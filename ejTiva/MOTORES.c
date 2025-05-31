//=============================================================
// Código: Control de Motor DC con PWM y Dirección
// Placa: EK-TM4C1294XL
//
// Descripción:
// Este código controla un motor DC que gira hacia adelante durante 3 segundos 
// y luego se detiene por 3 segundos, repitiendo este ciclo indefinidamente.
//
// Conexiones utilizadas:
// - IN1 -> PA6 (control de dirección)
// - IN2 -> PA7 (control de dirección)
// - PWM Motor 1 -> PK4 (M0PWM6 - control de velocidad)
//=============================================================

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"

// Definición de pines
#define IN1_PIN GPIO_PIN_6     // PA6
#define IN2_PIN GPIO_PIN_7     // PA7
#define PWM1_PIN GPIO_PIN_4    // PK4 (M0PWM6)

//-----------------------------------------------------------------------------
// Función: Motor_Init
// Inicializa el PWM y los pines de dirección del motor
//-----------------------------------------------------------------------------
void Motor_Init(void) {
    // Habilitar periféricos
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);    // PWM0 para M0PWM6
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);   // Puerto K: PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);   // Puerto A: IN1/IN2

    // Esperar a que estén listos
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    // Configurar PK4 como salida PWM
    GPIOPinConfigure(GPIO_PK4_M0PWM6);                // Mapear PK4 -> M0PWM6
    GPIOPinTypePWM(GPIO_PORTK_BASE, PWM1_PIN);        // PWM en PK4

    // Configurar PA6 y PA7 como salidas digitales (dirección)
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN);

    // Configurar PWM0_GEN_3 (para M0PWM6) en modo descendente sin sincronización
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Calcular periodo para 20kHz
    uint32_t pwmClock = SysCtlClockGet() / 64;               // Divisor = 64
    uint32_t load = (pwmClock / 20000) - 1;                  // Frecuencia PWM = 20kHz

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, load);             // Establecer periodo

    // Inicialmente: motor detenido
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 0);
    PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}

//-----------------------------------------------------------------------------
// Función: Motor1_Forward
// Gira el motor hacia adelante con el ciclo de trabajo dado
//-----------------------------------------------------------------------------
void Motor1_Forward(uint32_t duty) {
    GPIOPinWrite(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN, IN1_PIN);  // IN1=1, IN2=0
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, duty);              // Ciclo de trabajo
}

//-----------------------------------------------------------------------------
// Función: Motor_Stop
// Detiene el motor (PWM=0, IN1=0, IN2=0)
//-----------------------------------------------------------------------------
void Motor_Stop(void) {
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN, 0);
}

//-----------------------------------------------------------------------------
// Función: delaySeconds
// Retardo bloqueante en segundos (basado en SysCtlDelay)
//-----------------------------------------------------------------------------
void delaySeconds(uint32_t seconds) {
    uint32_t delayCycles = SysCtlClockGet() * seconds;
    while (delayCycles--) {
        SysCtlDelay(1);  // Cada SysCtlDelay equivale a 3 ciclos de reloj
    }
}

//-----------------------------------------------------------------------------
// Función principal
//-----------------------------------------------------------------------------
int main(void) {
    // Configuración del sistema a 120 MHz
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                       SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    Motor_Init();  // Inicializar módulo de motor

    uint32_t duty = 4000;  // Ajustar para velocidad deseada (0 a carga máxima)

    while (1) {
        Motor1_Forward(duty);  // Motor gira hacia adelante 3 segundos
        delaySeconds(3);

        Motor_Stop();          // Motor se detiene 3 segundos
        delaySeconds(3);
    }
}
