# ------------------------------------------------------------
# Proyecto: Control de Temperatura Automático (Simulado) con Raspberry Pi
#
# Descripción:
# Este código simula el control de temperatura en una sala.
# - El usuario ingresa manualmente el valor de temperatura.
# - Si la temperatura es mayor a 20°C se activa un ventilador (Cooler),
#   controlado mediante un puente H (Motor DC).
# - Si la temperatura es menor a 2°C se activa un LED rojo (Heater).
# - Si está entre 2°C y 20°C todo se apaga.
#
# Conexiones:
# Ventilador (Motor DC controlado con Puente H):
#   - IN1 -> GPIO 5
#   - IN2 -> GPIO 6
#
# LED Rojo (Heater):
#   - Ánodo (+) -> Resistencia -> GPIO 27
#   - Cátodo (-) -> GND
#
# ------------------------------------------------------------

import RPi.GPIO as GPIO
import time

# Configuración de los pines
IN1 = 5   # Control Motor - Puente H
IN2 = 6   # Control Motor - Puente H
HEATER = 27  # LED rojo

GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(HEATER, GPIO.OUT)

print("----- Sistema de Control de Temperatura -----")

try:
    while True:
        temperatura = float(input("Ingrese la temperatura (°C): "))

        if temperatura > 20:
            print("Temperatura alta - Activando ventilador (Motor).")
            GPIO.output(IN1, True)
            GPIO.output(IN2, False)
            GPIO.output(HEATER, False)

        elif temperatura < 2:
            print("Temperatura baja - Activando LED rojo (Heater).")
            GPIO.output(HEATER, True)
            GPIO.output(IN1, False)
            GPIO.output(IN2, False)

        else:
            print("Temperatura dentro del rango adecuado.")
            GPIO.output(IN1, False)
            GPIO.output(IN2, False)
            GPIO.output(HEATER, False)

        time.sleep(1)

except KeyboardInterrupt:
    print("Finalizando programa...")

finally:
    GPIO.cleanup()
