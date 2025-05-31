# =====================================================================================
# Código para Raspberry Pi con puente H L298N y comunicación serial con Tiva C EK-TM4C1294XL
# Funcionalidad:
# - Recibe comandos por UART desde la Tiva para activar los motores 1 y 2.
# - Controla los pines GPIO para direccionar y manejar velocidad con PWM.
# - Permite modificar el ciclo de trabajo (duty cycle) de los motores en tiempo real
#   mediante entrada por SSH sin bloquear la lectura UART.
#
# Conexiones utilizadas:
# Raspberry Pi GPIO:
#   MOTOR1_IN1 -> GPIO 17 (IN1 L298N motor 1)
#   MOTOR1_IN2 -> GPIO 27 (IN2 L298N motor 1)
#   MOTOR2_IN3 -> GPIO 22 (IN3 L298N motor 2)
#   MOTOR2_IN4 -> GPIO 23 (IN4 L298N motor 2)
#   ENA        -> GPIO 18 (PWM para motor 1)
#   ENB        -> GPIO 19 (PWM para motor 2)
# Comunicación serial:
#   Puerto serie /dev/ttyACM0 a 9600 baudios con Tiva C EK-TM4C1294XL
# =====================================================================================

from gpiozero import OutputDevice, PWMOutputDevice
from time import sleep
import serial
import threading
import sys
import select

# Configuración de pines GPIO
MOTOR1_IN1 = 17
MOTOR1_IN2 = 27
MOTOR2_IN3 = 22
MOTOR2_IN4 = 23
ENA = 18
ENB = 19

# Configuración de los dispositivos GPIO
motor1_in1 = OutputDevice(MOTOR1_IN1)
motor1_in2 = OutputDevice(MOTOR1_IN2)
motor2_in3 = OutputDevice(MOTOR2_IN3)
motor2_in4 = OutputDevice(MOTOR2_IN4)
motor1_pwm = PWMOutputDevice(ENA, initial_value=0)
motor2_pwm = PWMOutputDevice(ENB, initial_value=0)

# Configurar la comunicación serial con la Tiva
ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
ser.reset_input_buffer()

# Variable global para almacenar el duty cycle
duty_cycle = 0.5  # Valor inicial predeterminado

# Función para cambiar el duty cycle en tiempo real
def solicitar_duty_cycle():
    global duty_cycle
    print("Introduce un nuevo duty cycle (0.0 a 1.0) en cualquier momento:")

    while True:
        # Verifica si hay entrada del usuario sin bloquear
        if select.select([sys.stdin], [], [], 0)[0]:
            nuevo_duty = sys.stdin.readline().strip()
            try:
                nuevo_duty = float(nuevo_duty)
                if 0.0 <= nuevo_duty <= 1.0:
                    duty_cycle = nuevo_duty
                    print(f"Nuevo ciclo de trabajo configurado: {duty_cycle}")
                else:
                    print("Introduce un número entre 0.0 y 1.0.")
            except ValueError:
                print("Entrada inválida. Introduce un número válido.")

# Función para leer datos de la Tiva y activar motores
def leer_uart():
    global duty_cycle
    print("Esperando datos de la Tiva...")

    while True:
        try:
            if ser.in_waiting > 0:  # Revisar si hay datos disponibles
                value = ser.readline().decode('utf-8').rstrip()
                print(f"Datos recibidos: {value}")

                if value == "motor1":
                    print(f"Activando Motor 1 con duty cycle {duty_cycle}")
                    motor1_in1.on()
                    motor1_in2.off()
                    motor1_pwm.value = duty_cycle  
                    sleep(2)
                    motor1_in1.off()
                    motor1_in2.off()
                    motor1_pwm.off()

                elif value == "motor2":
                    print(f"Activando Motor 2 con duty cycle {duty_cycle}")
                    motor2_in3.on()
                    motor2_in4.off()
                    motor2_pwm.value = duty_cycle  
                    sleep(2)
                    motor2_in3.off()
                    motor2_in4.off()
                    motor2_pwm.off()

        except Exception as e:
            print("Error:", e)

# Crear e iniciar los hilos
hilo_duty = threading.Thread(target=solicitar_duty_cycle, daemon=True)
hilo_uart = threading.Thread(target=leer_uart, daemon=True)

hilo_duty.start()
hilo_uart.start()

# Mantener el programa en ejecución
try:
    while True:
        sleep(1)
except KeyboardInterrupt:
    print("Programa terminado.")
