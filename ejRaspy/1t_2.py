# =============================================================================================
# Proyecto: Control de Temperatura Automático (Simulado) con Raspberry Pi
#
# Descripción:
# Este programa simula un sistema automático de control de temperatura para una sala.
# - Si la temperatura ingresada es mayor a 20°C, se activa un ventilador (motor DC) usando
#   un puente H (controlado por los pines GPIO 5 y 6).
# - Si la temperatura es menor a 2°C, se enciende un LED rojo (simula un calefactor).
# - Si la temperatura está entre 2°C y 20°C, todo se apaga.
#
# Este código permite comprobar visualmente (con motor y LED) el funcionamiento de un sistema
# de climatización básico controlado por temperatura.
#
# Conexiones:
# Ventilador (Motor DC con Puente H):
#   - IN1 -> GPIO 5 (Pin físico 29)
#   - IN2 -> GPIO 6 (Pin físico 31)
#
# LED Rojo (Heater):
#   - Ánodo (+) -> Resistencia -> GPIO 27 (Pin físico 13)
#   - Cátodo (-) -> GND
#
# Librerías utilizadas:
#   - RPi.GPIO: Para manejar los pines GPIO de la Raspberry Pi.
#   - time: Para generar retardos.
#
# Autor: Heber
# =============================================================================================

import RPi.GPIO as GPIO
import time

# --------------------------- Configuración de Pines ---------------------------
IN1 = 5       # Pin GPIO para dirección del motor (ventilador)
IN2 = 6       # Pin GPIO para dirección contraria del motor (no se usa en reversa aquí)
HEATER = 27   # Pin GPIO conectado al LED rojo que simula el calefactor

# --------------------------- Inicialización GPIO ------------------------------
GPIO.setmode(GPIO.BCM)          # Usar numeración BCM (número GPIO)
GPIO.setup(IN1, GPIO.OUT)       # Configurar IN1 como salida
GPIO.setup(IN2, GPIO.OUT)       # Configurar IN2 como salida
GPIO.setup(HEATER, GPIO.OUT)    # Configurar HEATER como salida

# --------------------------- Mensaje inicial -------------------------------
print("----- Sistema de Control de Temperatura -----")

# --------------------------- Bucle principal -------------------------------
try:
    while True:
        temperatura = float(input("Ingrese la temperatura (°C): "))  # Entrada del usuario

        if temperatura > 20:
            print("Temperatura alta - Activando ventilador (Motor).")
            GPIO.output(IN1, True)     # Gira en una dirección (activa el ventilador)
            GPIO.output(IN2, False)    # Dirección única
            GPIO.output(HEATER, False) # Heater apagado

        elif temperatura < 2:
            print("Temperatura baja - Activando LED rojo (Heater).")
            GPIO.output(HEATER, True)  # Encender calefactor (LED)
            GPIO.output(IN1, False)    # Apagar motor
            GPIO.output(IN2, False)

        else:
            print("Temperatura dentro del rango adecuado. Todo apagado.")
            GPIO.output(IN1, False)    # Apagar motor
            GPIO.output(IN2, False)
            GPIO.output(HEATER, False) # Apagar calefactor

        time.sleep(1)  # Esperar antes de nueva lectura

# --------------------------- Salida segura del programa --------------------
except KeyboardInterrupt:
    print("\nFinalizando programa...")

finally:
    GPIO.cleanup()  # Liberar recursos GPIO
