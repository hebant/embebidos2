/*
===============================================================================
Nombre: Proyecto Final Integrado - UART + ADC + LEDs + Buzzer + Botones + Sensor Ultrasónico + Motor
Autor: Heber (optimizado con asistencia de IA)
Fecha: Abril 2025
Placa: EK-TM4C1294XL

Descripción:
    Este código integra múltiples funcionalidades:
    1. Lectura de potenciómetro via ADC (PK3/AIN19) para controlar velocidad del motor
    2. Control de LEDs y buzzer basado en valor ADC
    3. Botones para control de motores (PJ0/PJ1)
    4. Sensor ultrasónico HC-SR04 (PB2/PB3)
    5. Control de motor DC con PWM (PF1 - M0PWM1) velocidad proporcional al ADC
    6. Comunicación UART para monitorización y control

Configuraciones:
    - Reloj del sistema: 120 MHz
    - UART0 a 9600 baudios (PA0/PA1)
    - PWM para motor a 20kHz
    - Pines:
        * Trig: PB2, Echo: PB3
        * Motor: IN1(PA6), IN2(PA7), PWM1(PF1 - M0PWM1)
        * LEDs: PN0, PN1, PF0, PF4
        * Buzzer: PN4
        * Botones: PJ0, PJ1
===============================================================================
*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.c"

// Definiciones de pines
#define TRIG_PIN GPIO_PIN_2     // PB2
#define ECHO_PIN GPIO_PIN_3     // PB3
#define IN1_PIN GPIO_PIN_6      // PA6
#define IN2_PIN GPIO_PIN_7      // PA7
#define PWM1_PIN GPIO_PIN_1     // PF1 (PWM Motor 1 - M0PWM1)

// Variables globales
char uartBuffer[100];           // Buffer para UART
uint32_t adcValue;              // Valor ADC
uint32_t motorDutyCycle = 0;    // Ciclo de trabajo del motor (se calculará del ADC)
const uint32_t PWM_PERIOD = 1700; // Periodo PWM para ~20kHz

// Prototipos de funciones
void SystemClock_Init(void);
void UART_Init(void);
void ADC_Init(void);
void GPIO_Init(void);
void Ultrasonic_Init(void);
void Motor_Init(void);
void Motor_Forward(uint32_t duty);
void Motor_Stop(void);
uint32_t MeasureDistance(void);
void DelayMicroseconds(uint32_t us);
void DelayMilliseconds(uint32_t ms);
uint32_t MapADCToPWM(uint32_t adcVal);

int main(void)
{
    // Inicialización de periféricos
    SystemClock_Init();
    UART_Init();
    ADC_Init();
    GPIO_Init();
    Ultrasonic_Init();
    Motor_Init();

    while(1)
    {
        // 1. Manejo de UART (recepción y respuesta)
        if(UARTCharsAvail(UART0_BASE))
        {
            UARTgets(uartBuffer, sizeof(uartBuffer));
            strcat(uartBuffer, " desde Tiva\n");
            UARTprintf(uartBuffer);
        }

        // 2. Lectura de botones
        if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == 0)
        {
            UARTprintf("Botón 1 presionado\n");
            DelayMilliseconds(100);
        }

        if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) == 0)
        {
            UARTprintf("Botón 2 presionado\n");
            DelayMilliseconds(100);
        }

        // 3. Lectura ADC y control de LEDs/Buzzer/Motor
        ADCProcessorTrigger(ADC0_BASE, 3);
        while(!ADCIntStatus(ADC0_BASE, 3, false));
        ADCIntClear(ADC0_BASE, 3);
        ADCSequenceDataGet(ADC0_BASE, 3, &adcValue);
        
        // Mapear valor ADC (0-4095) a PWM (0-1700)
        motorDutyCycle = MapADCToPWM(adcValue);
        
        UARTprintf("ADC: %d, PWM: %d\n", adcValue, motorDutyCycle);
        
        // Control de LEDs y buzzer
        if(adcValue > 2047)
        {
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0 | GPIO_PIN_1);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_PIN_0 | GPIO_PIN_4);
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, GPIO_PIN_4); // Buzzer ON
        }
        else
        {
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0);
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0); // Buzzer OFF
        }

        // 4. Medición ultrasónica y control del motor
        uint32_t distance = MeasureDistance();
        UARTprintf("Distancia: %d cm\n", distance);
        
        if(distance > 10) {
            Motor_Forward(motorDutyCycle);
        } else {
            Motor_Stop();
            UARTprintf("Obstáculo cercano - Motor detenido\n");
        }

        DelayMilliseconds(100); // Pequeña pausa entre iteraciones
    }
}

// Mapea el valor ADC (0-4095) al rango PWM (0-PWM_PERIOD)
uint32_t MapADCToPWM(uint32_t adcVal)
{
    // Asegurarnos que el valor ADC no exceda el máximo
    if(adcVal > 4095) adcVal = 4095;
    
    // Mapear linealmente 0-4095 a 0-PWM_PERIOD
    return (adcVal * PWM_PERIOD) / 4095;
}

// Implementación de funciones (el resto permanece igual que en la versión anterior)

void SystemClock_Init(void)
{
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | 
                      SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);
}

void UART_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    UARTStdioConfig(0, 9600, 120000000);
}

void ADC_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH19 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
}

void GPIO_Init(void)
{
    // Habilitar periféricos GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    
    // Configurar botones (PJ0 y PJ1 con pull-up)
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
    // Configurar LEDs y buzzer como salidas
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
}

void Ultrasonic_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, TRIG_PIN);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, ECHO_PIN);
}

void Motor_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    
    // Configurar PWM para motor en PF1 (M0PWM1)
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, PWM1_PIN);
    
    // Configurar pines de dirección del motor
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN);
    
    // Configurar generador de PWM (usando GEN_0 para M0PWM1)
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWM_PERIOD);  // Periodo para ~20kHz
    
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);  // Inicialmente detenido
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

void Motor_Forward(uint32_t duty)
{
    GPIOPinWrite(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN, IN1_PIN);  // Sentido adelante
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, duty);
}

void Motor_Stop(void)
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN, 0);
}

uint32_t MeasureDistance(void)
{
    uint32_t pulseWidth = 0;
    
    // Generar pulso de trigger
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, 0);
    DelayMicroseconds(2);
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, TRIG_PIN);
    DelayMicroseconds(10);
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, 0);
    
    // Esperar flanco de subida en Echo
    uint32_t timeout = 300000;
    while((GPIOPinRead(GPIO_PORTB_BASE, ECHO_PIN) == 0) && timeout--);
    if(timeout == 0) return 0;
    
    // Medir ancho del pulso
    pulseWidth = 0;
    timeout = 300000;
    while((GPIOPinRead(GPIO_PORTB_BASE, ECHO_PIN) != 0) && timeout--) {
        DelayMicroseconds(1);
        pulseWidth++;
    }
    
    if(timeout == 0) return 0;
    
    return (pulseWidth * 0.0343); // Convertir a cm
}

void DelayMicroseconds(uint32_t us)
{
    SysCtlDelay((SysCtlClockGet() / 3000000) * us);
}

void DelayMilliseconds(uint32_t ms)
{
    SysCtlDelay((SysCtlClockGet() / 3000) * ms);
}