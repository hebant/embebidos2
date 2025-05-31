/*
===============================================================================
Nombre: Proyecto Final Integrado - UART + ADC + LEDs + Buzzer + Botones + Sensor Ultrasónico + Motor
Autor: Heber (optimizado con asistencia de IA)
Fecha: Abril 2025
Placa: EK-TM4C1294XL

Descripción:
    Este código integra múltiples funcionalidades usando la placa Tiva C:
    1. Lectura de un potenciómetro con el ADC (canal AIN19 / PK3)
    2. Control de LEDs y buzzer según el valor leído por el ADC
    3. Uso de botones (PJ0/PJ1) para simular control del motor (envían datos por UART)
    4. Medición de distancia con un sensor ultrasónico HC-SR04
    5. Control de un motor DC con PWM (pin PF1)
    6. Comunicación UART para enviar datos al monitor serial y recibir comandos

Configuraciones:
    - Reloj del sistema: 120 MHz
    - UART0 a 9600 baudios (PA0/PA1)
    - PWM para motor a 20kHz (~1700 ciclos)
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

// Librerías de hardware de la Tiva C
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
#define TRIG_PIN GPIO_PIN_2     // PB2 - Señal de disparo del sensor ultrasónico
#define ECHO_PIN GPIO_PIN_3     // PB3 - Señal de eco del sensor ultrasónico
#define IN1_PIN GPIO_PIN_6      // PA6 - Entrada de dirección del motor
#define IN2_PIN GPIO_PIN_7      // PA7 - Entrada de dirección del motor
#define PWM1_PIN GPIO_PIN_1     // PF1 - Salida PWM para el motor (M0PWM1)

// Variables globales
char uartBuffer[100];           // Buffer para almacenamiento temporal de cadenas UART
uint32_t adcValue;              // Almacena valor leido del ADC
uint32_t motorDutyCycle = 850;  // Ciclo de trabajo del PWM (~50% de 1700)

// Prototipos de funciones utilizadas
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

int main(void)
{
    // Inicialización de todos los periféricos usados
    SystemClock_Init();
    UART_Init();
    ADC_Init();
    GPIO_Init();
    Ultrasonic_Init();
    Motor_Init();

    while(1)
    {
        // 1. Revisión de recepción UART
        if(UARTCharsAvail(UART0_BASE))
        {
            UARTgets(uartBuffer, sizeof(uartBuffer));         // Leer datos del monitor serial
            strcat(uartBuffer, " desde Tiva\n");              // Agregar texto personalizado
            UARTprintf(uartBuffer);                           // Enviar de vuelta por UART
        }

        // 2. Lectura de los botones en PJ0 y PJ1
        if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_0) == 0)     // Botón PJ0 presionado
        {
            UARTprintf("motor1\n");                           // Enviar mensaje por UART
            DelayMilliseconds(100);                           // Pequeño delay para evitar rebotes
        }

        if(GPIOPinRead(GPIO_PORTJ_BASE, GPIO_PIN_1) == 0)     // Botón PJ1 presionado
        {
            UARTprintf("motor2\n");
            DelayMilliseconds(100);
        }

        // 3. Lectura del ADC y control de LEDs y buzzer
        ADCProcessorTrigger(ADC0_BASE, 3);                    // Disparar conversión
        while(!ADCIntStatus(ADC0_BASE, 3, false));            // Esperar a que termine
        ADCIntClear(ADC0_BASE, 3);                            // Limpiar bandera de interrupción
        ADCSequenceDataGet(ADC0_BASE, 3, &adcValue);          // Obtener valor del ADC
        
        UARTprintf("ADC Value: %d\n", adcValue);              // Mostrar valor en UART
        
        // Encender LEDs y buzzer si el valor del ADC es mayor a la mitad (~2047)
        if(adcValue > 2047)
        {
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0 | GPIO_PIN_1); // LEDs PN0 y PN1 ON
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_PIN_0 | GPIO_PIN_4); // LEDs PF0 y PF4 ON
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, GPIO_PIN_4); // Buzzer ON
        }
        else
        {
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);     // LEDs OFF
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0);     // LEDs OFF
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0);                  // Buzzer OFF
        }

        // 4. Medición ultrasónica y control de motor
        uint32_t distance = MeasureDistance();                 // Obtener distancia en cm
        UARTprintf("Distancia: %d cm\n", distance);           // Mostrar distancia

        if(distance > 10)                                      // Si no hay obstáculo cerca
        {
            Motor_Forward(motorDutyCycle);                    // Mover motor hacia adelante
        }
        else
        {
            Motor_Stop();                                     // Detener motor si algo está cerca
        }

        DelayMilliseconds(10000); // Delay largo (10 segundos) para evitar spam en UART
    }
}

// Inicializa el reloj del sistema a 120 MHz
void SystemClock_Init(void)
{
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | 
                      SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);
}

// Inicializa UART0 para comunicación serial
void UART_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);  // Configura PA0 como RX
    GPIOPinConfigure(GPIO_PA1_U0TX);  // Configura PA1 como TX
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    UARTStdioConfig(0, 9600, 120000000); // UART0 a 9600 baudios
}

// Inicializa el ADC para leer el canal AIN19 (PK3)
void ADC_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);  // PK3 -> AIN19
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0); // Secuencia 3, un solo paso
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH19 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
}

// Configura los GPIO para botones, LEDs y buzzer
void GPIO_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); // Botones
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); // LEDs y buzzer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // LEDs

    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Entradas con pull-up
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4); // LEDs y buzzer
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4); // LEDs
}

// Configura los pines del sensor ultrasónico
void Ultrasonic_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, TRIG_PIN); // PB2 como salida
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, ECHO_PIN);  // PB3 como entrada
}

// Inicializa el PWM y pines de control del motor
void Motor_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)); // Esperar que esté listo
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    
    GPIOPinConfigure(GPIO_PF1_M0PWM1); // Configura PF1 para PWM1
    GPIOPinTypePWM(GPIO_PORTF_BASE, PWM1_PIN);
    
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN); // Pines de dirección

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); // Configuración PWM
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 1700); // Periodo = 1700 ciclos (~20kHz)

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0); // Inicialmente sin movimiento
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}

// Gira el motor hacia adelante con el duty indicado
void Motor_Forward(uint32_t duty)
{
    GPIOPinWrite(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN, IN1_PIN); // PA6 = 1, PA7 = 0
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, duty);              // Aplica PWM
}

// Detiene el motor
void Motor_Stop(void)
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0); // Duty = 0
    GPIOPinWrite(GPIO_PORTA_BASE, IN1_PIN | IN2_PIN, 0); // IN1 e IN2 en 0
}

// Mide la distancia usando el sensor ultrasónico HC-SR04
uint32_t MeasureDistance(void)
{
    uint32_t pulseWidth = 0;

    // Genera pulso de 10us en TRIG
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, 0);
    DelayMicroseconds(2);
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, TRIG_PIN);
    DelayMicroseconds(10);
    GPIOPinWrite(GPIO_PORTB_BASE, TRIG_PIN, 0);

    // Espera el inicio del pulso de eco
    uint32_t timeout = 300000;
    while((GPIOPinRead(GPIO_PORTB_BASE, ECHO_PIN) == 0) && timeout--);
    if(timeout == 0) return 0;

    // Cuenta duración del pulso de eco
    pulseWidth = 0;
    timeout = 300000;
    while((GPIOPinRead(GPIO_PORTB_BASE, ECHO_PIN) != 0) && timeout--) {
        DelayMicroseconds(1);
        pulseWidth++;
    }

    if(timeout == 0) return 0;

    return (pulseWidth * 0.0343); // Conversión a cm (con velocidad del sonido)
}

// Delay en microsegundos (usando SysCtlDelay)
void DelayMicroseconds(uint32_t us)
{
    SysCtlDelay((SysCtlClockGet() / 3000000) * us);
}

// Delay en milisegundos
void DelayMilliseconds(uint32_t ms)
{
    SysCtlDelay((SysCtlClockGet() / 3000) * ms);
}
