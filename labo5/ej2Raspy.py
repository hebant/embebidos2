from gpiozero import OutputDevice, PWMOutputDevice
from time import sleep
import serial

# Configurar los pines GPIO de la Raspberry Pi para controlar el puente H L298N
MOTOR1_IN1 = 17  # GPIO 17 conectado a IN1 del L298N
MOTOR1_IN2 = 27  # GPIO 27 conectado a IN2 del L298N
MOTOR2_IN3 = 22  # GPIO 22 conectado a IN3 del L298N
MOTOR2_IN4 = 23  # GPIO 23 conectado a IN4 del L298N
ENA = 18         # GPIO 18 conectado a ENA (control de velocidad motor 1)
ENB = 19         # GPIO 19 conectado a ENB (control de velocidad motor 2)

# Configuración de los dispositivos GPIO con gpiozero
motor1_in1 = OutputDevice(MOTOR1_IN1)
motor1_in2 = OutputDevice(MOTOR1_IN2)
motor2_in3 = OutputDevice(MOTOR2_IN3)
motor2_in4 = OutputDevice(MOTOR2_IN4)
motor1_pwm = PWMOutputDevice(ENA)  # PWM para motor 1
motor2_pwm = PWMOutputDevice(ENB)  # PWM para motor 2

# Configurar la comunicación serial con la Tiva
ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
ser.reset_input_buffer()  # Limpia cualquier byte innecesario en el buffer

print("Esperando datos de la Tiva...")

while True:
    try:
        if ser.in_waiting > 0:  # Revisar si hay datos disponibles
            value = ser.readline().decode('utf-8').rstrip()  # Leer y decodificar el dato recibido
            print("Datos recibidos:", value)
            
            if value == "motor1":
                print("Activando Motor 1")
                motor1_in1.on()
                motor1_in2.off()
                motor1_pwm.value = 0.5  # 50% de velocidad (duty cycle)
                sleep(2)
                motor1_in1.off()
                motor1_in2.off()
                motor1_pwm.off()
            
            elif value == "motor2":
                print("Activando Motor 2")
                motor2_in3.on()
                motor2_in4.off()
                motor2_pwm.value = 0.75  # 75% de velocidad (duty cycle)
                sleep(2)
                motor2_in3.off()
                motor2_in4.off()
                motor2_pwm.off()

            elif value.startswith("motor1_speed"):
                # Cambiar la velocidad de motor1 basado en el comando recibido
                try:
                    speed = float(value.split("_")[-1]) / 100  # Convertir a un valor entre 0 y 1
                    print(f"Cambiando velocidad de Motor 1 a {speed * 100}%")
                    motor1_pwm.value = speed
                except ValueError:
                    print("Comando de velocidad inválido para Motor 1.")
            
            elif value.startswith("motor2_speed"):
                # Cambiar la velocidad de motor2 basado en el comando recibido
                try:
                    speed = float(value.split("_")[-1]) / 100  # Convertir a un valor entre 0 y 1
                    print(f"Cambiando velocidad de Motor 2 a {speed * 100}%")
                    motor2_pwm.value = speed
                except ValueError:
                    print("Comando de velocidad inválido para Motor 2.")

    except Exception as e:
        print("Error:", e)
    except KeyboardInterrupt:
        print("Saliendo...")
        break