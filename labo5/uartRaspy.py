#PARA VER LOS MENSAJES ENVIADOS DE LA TIVA EN LA RASPY

import serial
from time import sleep

# Configurar la comunicación serial con la Tiva
ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
ser.reset_input_buffer()  # Limpia cualquier byte innecesario en el buffer

print("Esperando datos de la Tiva...")

while True:
    try:
        if ser.in_waiting > 0:  # Revisar si hay datos disponibles
            value = ser.readline().decode('utf-8').rstrip()  # Leer y decodificar el dato recibido
            print("Datos recibidos:", value)
    except Exception as e:
        print("Error:", e)