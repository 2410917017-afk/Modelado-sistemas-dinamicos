import serial
import time
import numpy as np
from collections import deque

# Configuración del puerto serial

PuertoSerial = ''
Baudrate = ''

data_array = np.empty((0, 2), dtype=float)

buffer = deque()
VENTANA = 5

tiempoM = 5

try:
    arduino = serial.Serial(PuertoSerial, Baudrate, timeout = 1)
    # timeout=1 significa que espera 1 segundo máximo por datos
    time.sleep(2)  
except Exception:
    print("Error al abrir el puerto serial :(")
    arduino = None  # si falla, se asigna None (pero sigue ejecutando sin Arduino)

def media_movil(valor):
    """
    Calcula el promedio móvil de los últimos VENTANA valores.
    Evita que pequeñas variaciones de detección causen saltos bruscos.
    """
    global suma

    buffer.append(valor)         # Agregar nuevo valor
    suma += valor                # Sumar al total

    if len(buffer) > VENTANA:    # Si excedemos la ventana
        suma -= buffer.popleft() # Remover el más antiguo

    return suma / len(buffer)    # Retornar promedio


try:
    raw = input('Iniciar recolección de datos? (S/N)')
    if raw != 'S' or raw != 'N' :
        print("Error en el comando de inicio")
    
    while True:

        if raw == 'N':

            break
        elif raw == 'S':
            inicio = time.time()

            line = arduino.readline().decode("utf-8").strip()
            if line:
                valores = [float(v) for v in line.split(",")]
                data = np.vstack([data, valores])
                data1 = data[:, 0]
                data2 = data[:, 1]
                
        
                    

except KeyboardInterrupt or (tiempoM - inicio <= 1):
    print("\nLectura finalizada")
    data1 = np.array(data[:, 0])
    data2 = np.array(data[:, 1])


finally:
    arduino.close()


