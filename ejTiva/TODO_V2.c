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

// Definiciones de pines para facilitar su uso y mantenimiento
#define TRIG_PIN GPIO_PIN_2     // Pin para disparar el sensor ultrasónico (PB2)
#define ECHO_PIN GPIO_PIN_3     // Pin para recibir eco ultrasónico (PB3)
#define IN1_PIN GPIO_PIN_6      // Pin de dirección motor IN1 (PA6)
#define IN2_PIN GPIO_PIN_7      // Pin de dirección motor IN2 (PA7)
#define PWM1_PIN GPIO_PIN_1     // Pin PWM para motor (PF1, PWM módulo 0 salida 1)

// Variables globales usadas en todo el programa
char uartBuffer[100];           // Buffer para almacenar cadenas recibidas por UART
uint32_t adcValue;              // Variable para almacenar valor leído del ADC
uint32_t motorDutyCycle = 0;    // Ciclo de trabajo para PWM del motor, basado en adcValue
const uint32_t PWM_PERIOD = 1700; // Periodo PWM para frecuencia de ~20kHz

// Prototipos de funciones para inicialización y control
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
    // Inicializar todos los periféricos antes del ciclo principal
    SystemClock_Init();
    UART_Init();
    ADC_Init();
    GPIO_Init();
    Ultrasonic_Init();
    Motor_Init();

    while(1)
    {
        // 1. Manejar comunicación UART: si hay datos disponibles, leerlos y responder
        if(UARTCharsAvail(UART0_BASE))
        {
            UARTgets(uartBuffer, sizeof(uartBuffer));  // Leer cadena desde UART
            strcat(uartBuffer, " desde Tiva\n");       // Añadir mensaje extra
            UARTprintf(uartBuffer);                     // Enviar respuesta por UART
        }

        // 2. Lectura de botones conectados a PJ0 y PJ1 con debounce básico
        if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == 0) // Botón 1 presionado (activo bajo)
        {
            UARTprintf("Botón 1 presionado\n");
            DelayMilliseconds(100);  // Retardo para evitar rebotes
        }

        if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) == 0) // Botón 2 presionado
        {
            UARTprintf("Botón 2 presionado\n");
            DelayMilliseconds(100);
        }

        // 3. Leer ADC para obtener valor del potenciómetro y controlar LEDs, buzzer y motor
        ADCProcessorTrigger(ADC0_BASE, 3);              // Iniciar conversión ADC
        while(!ADCIntStatus(ADC0_BASE, 3, false));      // Esperar que termine conversión
        ADCIntClear(ADC0_BASE, 3);                       // Limpiar bandera de interrupción
        ADCSequenceDataGet(ADC0_BASE, 3, &adcValue);    // Leer dato ADC
        
        // Mapear valor ADC (0-4095) a ciclo PWM (0-1700)
        motorDutyCycle = MapADCToPWM(adcValue);
        
        UARTprintf("ADC: %d, PWM: %d\n", adcValue, motorDutyCycle); // Enviar datos por UART
        
        // Control de LEDs y buzzer según valor del ADC (>2047 prende LEDs y buzzer)
        if(adcValue > 2047)
        {
            // Encender LEDs en PN0, PN1 y PF0, PF4, y buzzer en PN4
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0 | GPIO_PIN_1);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_PIN_0 | GPIO_PIN_4);
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, GPIO_PIN_4); // Buzzer ON
        }
        else
        {
            // Apagar LEDs y buzzer
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0);
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0); // Buzzer OFF
        }

        // 4. Medir distancia con sensor ultrasónico y controlar motor en consecuencia
        uint32_t distance = MeasureDistance();
        UARTprintf("Distancia: %d cm\n", distance);
        
        if(distance > 10) {
            Motor_Forward(motorDutyCycle); // Si distancia segura, avanzar motor con velocidad según ADC
        } else {
            Motor_Stop();                  // Obstáculo cercano, detener motor
            UARTprintf("Obstáculo cercano - Motor detenido\n");
        }

        DelayMilliseconds(100); // Pausa entre iteraciones para estabilidad
    }
}

// Función para mapear el valor ADC (0-4095) a ciclo PWM (0-PWM_PERIOD)
uint32_t MapADCToPWM(uint32_t adcVal)
{
    if(adcVal > 4095) adcVal = 4095; // Limitar máximo valor ADC
    return (adcVal * PWM_PERIOD) / 4095; // Escalar linealmente a PWM_PERIOD
}

// Inicializa reloj del sistema a 120 MHz usando cristal externo y PLL
void SystemClock_Init(void)
{
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | 
                      SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);
}

// Inicializa UART0 en GPIO PA0 (RX) y PA1 (TX) con 9600 baudios para comunicación serial
void UART_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    UARTStdioConfig(0, 9600, 120000000);
}

// Inicializa ADC0 en canal 19 (PK3) para lectura analógica del potenciómetro
void ADC_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3); // PK3 como entrada ADC
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0); // Secuencia 3
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH19 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
}

// Inicializa GPIO para botones en PJ0 y PJ1, LEDs en PN0, PN1, PF0, PF4, y buzzer en PN4
void GPIO_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    
    // Configurar botones como entradas con resistencia pull-up interna
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
    // Configurar LEDs y buzzer como salidas digitales
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
}

// Inicializa GPIO para sensor ultrasónico: PB2 (Trig) salida, PB3 (Echo) entrada
void Ultrasonic_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, TRIG_PIN);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, ECHO_PIN);
}

// Inicializa PWM para control del motor en PF1 y GPIO para dirección en PA6 y PA7
void Motor_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    
    GPIOPinConfigure(GPIO_PF1_M0PWM1);    // Configurar pin PF1 para función PWM módulo 0 salida 1
    GPIOPinTypePWM(GPIO_PORTF_BASE, PWM1_PIN);
    
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN); // Pines dirección motor como salidas
    
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); // Modo PWM
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWM_PERIOD);  // Establecer periodo PWM (frecuencia)
    
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);  // Ciclo inicial 0% (motor apagado)
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true); // Activar salida PWM
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);             // Habilitar generador PWM
}

// Función para avanzar motor en dirección adelante con ciclo de trabajo dado
void Motor_Forward(uint32_t duty)
{
    GPIOPinWrite(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN, IN1_PIN);  // IN1=1, IN2=0 -> avance
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, duty);              // Ajustar ciclo PWM velocidad
}

// Función para detener motor (PWM en 0 y apagar dirección)
void Motor_Stop(void)
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);                 // Ciclo PWM 0%
    GPIOPinWrite(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN, 0);       // Apagar pines dirección
}

// Función para medir distancia con sensor ultrasónico HC-SR04
uint32_t MeasureDistance(void)
{
    uint32_t pulseWidth = 0;
    
    // Generar pulso trigger de 10us para iniciar medición
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, 0);
    DelayMicroseconds(2);
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, TRIG_PIN);
    DelayMicroseconds(10);
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, 0);
    
    // Esperar hasta que pin Echo suba (inicio del pulso)
    uint32_t timeout = 300000;
    while((GPIOPinRead(GPIO_PORTB_BASE, ECHO_PIN) == 0) && timeout--);
    if(timeout == 0) return 0;  // Timeout, no se detectó eco
    
    // Medir duración del pulso Echo (ancho de pulso)
    pulseWidth = 0;
    timeout = 300000;
    while((GPIOPinRead(GPIO_PORTB_BASE, ECHO_PIN) != 0) && timeout--) {
        DelayMicroseconds(1);
        pulseWidth++;
    }
    
    if(timeout == 0) return 0; // Timeout, pulso demasiado largo
    
    // Convertir ancho de pulso a distancia en cm
    return (pulseWidth * 0.0343); // Velocidad sonido: 343 m/s = 0.0343 cm/us
}

// Función para retardos aproximados en microsegundos (no exacto)
void DelayMicroseconds(uint32_t us)
{
    // SysCtlDelay = 3 ciclos de reloj por iteración
    SysCtlDelay((SysCtlClockGet() / 3000000) * us);
}

// Función para retardos en milisegundos usando DelayMicroseconds
void DelayMilliseconds(uint32_t ms)
{
    SysCtlDelay((SysCtlClockGet() / 3000) * ms);
}
