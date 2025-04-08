//=============================================================
// Código: Control de Motor DC con PWM y dirección
// Placa: EK-TM4C1294XL
// Descripción:
// Este código hace girar un motor DC hacia adelante por 3 segundos 
// y luego lo detiene por 3 segundos, repitiendo este ciclo infinitamente.
//
// Conexiones utilizadas:
// - IN1 -> PA6 (control de dirección)
// - IN2 -> PA7 (control de dirección)
// - PWM Motor 1 -> PK4 (control de velocidad por PWM)
//=============================================================

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"

#define IN1_PIN GPIO_PIN_6  // PA6
#define IN2_PIN GPIO_PIN_7  // PA7
#define PWM1_PIN GPIO_PIN_4  // PK4 (PWM Motor 1)

void Motor_Init(void) {
    // Habilitar los periféricos de PWM y GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    // Configuración de los pines de PWM
    GPIOPinConfigure(GPIO_PK4_M0PWM6);
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4);

    // Configuración de los pines de dirección del motor
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN);

    // Configuración de PWM
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    uint32_t pwmClock = SysCtlClockGet() / 64;
    uint32_t load = (pwmClock / 20000) - 1;  // PWM a 20kHz
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, load);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 0);  // Motor detenido al inicio
    PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}

void Motor1_Forward(uint32_t duty) {
    // Dirección: avanzar (AIN1=1, AIN2=0)
    GPIOPinWrite(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN, IN1_PIN);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, duty);
}

void Motor_Stop(void) {
    // Motor detenido
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN, 0);
}

void delaySeconds(uint32_t seconds) {
    uint32_t delayCycles = (SysCtlClockGet() * seconds);
    while (delayCycles--) {
        SysCtlDelay(1);  // Un ciclo de SysCtlDelay son 3 ciclos de reloj
    }
}

int main(void) {
    // Configuración del reloj del sistema a 120 MHz
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                        SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    Motor_Init();  // Inicializar motor

    uint32_t duty = 4000;  // Ciclo de trabajo PWM (ajustable)

    while (1) {
        Motor1_Forward(duty);  // Motor avanza 3 segundos
        delaySeconds(3);

        Motor_Stop();          // Motor detenido 3 segundos
        delaySeconds(3);
    }
}
