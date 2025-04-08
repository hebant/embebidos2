# ------------------------------------------------------------
# Proyecto: Simulación de un garaje automático con Raspberry Pi
#
# Descripción:
# Este código simula un garaje automático. Se utiliza un sensor
# ultrasónico HC-SR04 para detectar objetos a menos de 7 cm de distancia.
# Si se detecta un objeto, se activa un motor (controlado por un relay o un
# puente H) que simula levantar la puerta del garaje.
#
# Conexiones:
# Sensor Ultrasónico HC-SR04:
#   - VCC  -> 5V de la Raspberry Pi
#   - GND  -> GND de la Raspberry Pi
#   - TRIG  -> GPIO 23 (Pin físico 16)
#   - ECHO  -> GPIO 24 (Pin físico 18)
#
# Motor (Relay o Puente H):
#   - IN1   -> GPIO 18 (Pin físico 12)
#
# ------------------------------------------------------------

import RPi.GPIO as GPIO
import time

# Configuración de los pines
TRIG = 23
ECHO = 24
MOTOR = 18

# Configuración inicial
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(MOTOR, GPIO.OUT)

def medir_distancia():
    # Enviar pulso de disparo (10us)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Esperar respuesta del eco
    while GPIO.input(ECHO) == 0:
        inicio = time.time()
    while GPIO.input(ECHO) == 1:
        fin = time.time()

    # Calcular distancia
    duracion = fin - inicio
    distancia = (duracion * 34300) / 2  # Velocidad del sonido = 34300 cm/s
    return distancia

try:
    while True:
        distancia = medir_distancia()
        print(f"Distancia medida: {distancia:.2f} cm")

        if distancia < 7:
            print("Objeto detectado - Abriendo garaje...")
            GPIO.output(MOTOR, True)  # Encender motor
            time.sleep(5)  # Simula el tiempo que tarda en abrir
            GPIO.output(MOTOR, False)  # Apagar motor
            print("Garaje abierto.\n")
        else:
            GPIO.output(MOTOR, False)  # Motor apagado
        
        time.sleep(1)

except KeyboardInterrupt:
    print("Finalizando programa...")

finally:
    GPIO.cleanup()
