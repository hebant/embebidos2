# =============================================================================================
# Proyecto: Simulación de un Garaje Automático con Raspberry Pi
#
# Descripción:
# Este programa controla un sistema de garaje automatizado usando un sensor ultrasónico HC-SR04
# y un motor (puede ser un relevador o puente H). Cuando el sensor detecta un objeto a menos
# de 7 cm (por ejemplo, un auto acercándose), se activa el motor que simula abrir la puerta.
#
# Conexiones:
# Sensor Ultrasónico HC-SR04:
#   - VCC   -> 5V de la Raspberry Pi
#   - GND   -> GND de la Raspberry Pi
#   - TRIG  -> GPIO 23 (pin físico 16)
#   - ECHO  -> GPIO 24 (pin físico 18)
#
# Motor (Relay o Puente H para control de motor):
#   - IN1   -> GPIO 18 (pin físico 12)
#
# Librerías utilizadas:
#   - RPi.GPIO: Para el manejo de pines GPIO de la Raspberry Pi.
#   - time: Para medir tiempos de respuesta y crear retardos.
#
# Autor: Heber
# =============================================================================================

import RPi.GPIO as GPIO
import time

# --------------------------- Configuración de Pines ---------------------------
TRIG = 23      # Pin GPIO para enviar pulso del sensor ultrasónico
ECHO = 24      # Pin GPIO para recibir eco del sensor
MOTOR = 18     # Pin GPIO que controla el motor/relevador

# --------------------------- Inicialización GPIO ------------------------------
GPIO.setmode(GPIO.BCM)              # Usar numeración BCM
GPIO.setup(TRIG, GPIO.OUT)          # TRIG como salida
GPIO.setup(ECHO, GPIO.IN)           # ECHO como entrada
GPIO.setup(MOTOR, GPIO.OUT)         # MOTOR como salida

# --------------------------- Función para medir distancia ---------------------
def medir_distancia():
    # Enviar un pulso de 10 microsegundos al pin TRIG
    GPIO.output(TRIG, True)
    time.sleep(0.00001)  # 10 µs
    GPIO.output(TRIG, False)

    # Esperar a que ECHO se active (inicio del eco)
    while GPIO.input(ECHO) == 0:
        inicio = time.time()

    # Esperar a que ECHO se desactive (fin del eco)
    while GPIO.input(ECHO) == 1:
        fin = time.time()

    # Calcular duración del pulso y convertirlo a distancia (cm)
    duracion = fin - inicio
    distancia = (duracion * 34300) / 2  # Velocidad del sonido = 34300 cm/s
    return distancia

# --------------------------- Bucle principal -------------------------------
try:
    while True:
        distancia = medir_distancia()  # Obtener distancia medida
        print(f"Distancia medida: {distancia:.2f} cm")

        if distancia < 7:  # Si el objeto está a menos de 7 cm
            print("Objeto detectado - Abriendo garaje...")
            GPIO.output(MOTOR, True)   # Activar motor
            time.sleep(5)             # Mantener encendido 5 segundos
            GPIO.output(MOTOR, False) # Apagar motor
            print("Garaje abierto.\n")
        else:
            GPIO.output(MOTOR, False) # Mantener motor apagado

        time.sleep(1)  # Esperar 1 segundo antes de la siguiente medición

# --------------------------- Salida segura del programa --------------------
except KeyboardInterrupt:
    print("Finalizando programa...")

finally:
    GPIO.cleanup()  # Liberar los pines GPIO al finalizar
