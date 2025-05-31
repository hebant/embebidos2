// =====================================================================================
// Código para Tiva C EK-TM4C1294XL
// Funcionalidad:
// Lee el valor de un potenciómetro conectado al pin PK3 (canal 19 del ADC0).
// Enciende los LEDs de los puertos N (PN0, PN1) y F (PF0, PF4) si el valor ADC es mayor
// a 2047 (aproximadamente la mitad del rango de 12 bits).
//
// Conexiones utilizadas:
// Potenciómetro - PK3 (Entrada analógica, ADC0, canal 19)
// LEDs - PN0, PN1, PF0, PF4
// =====================================================================================

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"

int main(void)
{
    uint32_t ui32ADCValue;  // Variable para guardar el valor ADC

    // =================================================================================
    // Configuración del sistema
    // =================================================================================

    // Configurar el reloj del sistema a 120 MHz usando PLL
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                       SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    // Habilitar periféricos necesarios
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);    // ADC0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);   // Puerto K (PK3)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);   // Puerto N (PN0, PN1)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);   // Puerto F (PF0, PF4)

    // =================================================================================
    // Configuración de pines
    // =================================================================================

    // PK3 como entrada analógica para el ADC (canal 19)
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);

    // PN0 y PN1 como salidas digitales
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // PF0 y PF4 como salidas digitales
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    // =================================================================================
    // Configuración del ADC0 (Secuencia 3, una sola muestra, trigger por procesador)
    // =================================================================================

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);  // Secuencia 3
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                             ADC_CTL_CH19 | ADC_CTL_IE | ADC_CTL_END); // Canal 19, interrupción y fin
    ADCSequenceEnable(ADC0_BASE, 3);   // Habilitar la secuencia
    ADCIntClear(ADC0_BASE, 3);         // Limpiar interrupción pendiente

    // =================================================================================
    // Bucle principal
    // =================================================================================
    while (1)
    {
        // Iniciar conversión del ADC
        ADCProcessorTrigger(ADC0_BASE, 3);

        // Esperar a que finalice la conversión
        while(!ADCIntStatus(ADC0_BASE, 3, false));

        // Limpiar la bandera de interrupción
        ADCIntClear(ADC0_BASE, 3);

        // Obtener el resultado de la conversión
        ADCSequenceDataGet(ADC0_BASE, 3, &ui32ADCValue);

        // Si el valor ADC > 2047, encender todos los LEDs
        if (ui32ADCValue > 2047)
        {
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                         GPIO_PIN_0 | GPIO_PIN_1);  // Encender PN0 y PN1
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4,
                         GPIO_PIN_0 | GPIO_PIN_4);  // Encender PF0 y PF4
        }
        else
        {
            // Si no, apagar todos los LEDs
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0);
        }
    }
}
