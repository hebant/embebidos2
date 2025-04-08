// Lectura de Potenciómetro con ADC0 (PK3 CH 19))
// Encendido/Apagado de LEDs según valor ADC
// Placa: EK-TM4C1294XL

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

    // Configuración del Reloj a 120 MHz
    SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    // Habilitar Periféricos
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);   // Habilitar ADC0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);  // Habilitar Puerto K (PK3)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);  // Habilitar LEDs Puerto N
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  // Habilitar LEDs Puerto F

    // Configurar PK3 como entrada ADC (AIN19)
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);

    // Configurar PN0 y PN1 como salidas
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configurar PF0 y PF4 como salidas
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    // Configuración del ADC0, secuencia 3 (1 muestra, trigger por procesador)
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH19 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);

    while (1)
    {
        // Iniciar conversión ADC
        ADCProcessorTrigger(ADC0_BASE, 3);

        // Esperar a que termine conversión
        while(!ADCIntStatus(ADC0_BASE, 3, false));

        // Limpiar interrupción ADC
        ADCIntClear(ADC0_BASE, 3);

        // Leer valor ADC
        ADCSequenceDataGet(ADC0_BASE, 3, &ui32ADCValue);

        // Comparar valor ADC y controlar LEDs
        if (ui32ADCValue > 2047)
        {
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_0 | GPIO_PIN_1);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_PIN_0 | GPIO_PIN_4);
        }
        else
        {
            GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0);
        }
    }
}