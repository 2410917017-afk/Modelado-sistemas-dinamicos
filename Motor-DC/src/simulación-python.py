import serial
import numpy as np
from scipy.optimize import curve_fit
import time
import matplotlib.pyplot as plt

# Configuración del puerto serial

PuertoSerial = ''
Baudrate = ''

t_data = []
i_data = []
w_data = []


tiempoM = 5

plt.ion()

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

line1, = ax1.plot([], [], 'b-', label="Corriente")
line2, = ax2.plot([], [], 'r-', label="Velocidad")
ax1.legend()
ax2.legend()
ax1.grid(True)
ax2.grid(True)

suposiciones_iniciales = [4.0, 1.0, 0.0]

def estimate_dc_motor_params(time, voltage, current, speed):
    """
    Estima parámetros de un motor DC usando mínimos cuadrados.
    
    time:   array de tiempos (s)
    voltage: array de voltajes aplicados (V)
    current: array de corrientes medidas (A)
    speed:   array de velocidades angulares medidas (rad/s)
    """
    # Validación de datos
    if not (len(time) == len(voltage) == len(current) == len(speed)):
        raise ValueError("Todos los vectores deben tener la misma longitud.")
    if len(time) < 5:
        raise ValueError("Se requieren al menos 5 muestras para estimar parámetros.")

    # Derivadas numéricas
    dt = np.gradient(time)
    di_dt = np.gradient(current, dt)
    dw_dt = np.gradient(speed, dt)

    # --- Estimación de parámetros eléctricos ---
    # Ecuación: V = R_a * i + L_a * di/dt + K_e * ω
    X_elec = np.column_stack((current, di_dt, speed))
    params_elec, _, _, _ = np.linalg.lstsq(X_elec, voltage, rcond=None)
    R_a, L_a, K_e = params_elec

    # --- Estimación de parámetros mecánicos ---
    # Ecuación: J * dω/dt + B * ω = K_t * i
    X_mech = np.column_stack((dw_dt, speed))
    params_mech, _, _, _ = np.linalg.lstsq(X_mech, current * K_e, rcond=None)
    J, B = params_mech
    K_t = K_e  # En motores DC de imán permanente, Kt ≈ Ke

    return {
        "R_a (Ohm)": R_a,
        "L_a (H)": L_a,
        "K_e (V·s/rad)": K_e,
        "K_t (N·m/A)": K_t,
        "J (kg·m²)": J,
        "B (N·m·s/rad)": B
    }



try:
    arduino = serial.Serial(PuertoSerial, Baudrate, timeout = 1)
    # timeout=1 significa que espera 1 segundo máximo por datos
    time.sleep(2)  


except Exception:
    print("Error al abrir el puerto serial :(")
    arduino = None  # si falla, se asigna None (pero sigue ejecutando sin Arduino)


try:
    raw = input('Iniciar recolección de datos? (S/N)')
    if raw != 'S' or raw != 'N' :
        print("Error en el comando de inicio")
    
    while True:
        

        if raw == 'N':

            break
        elif raw == 'S':
            tiempo = time.time()

            line = arduino.readline().decode("utf-8").strip()
            datos = line.split(",")

            if line: 
                t_data.append(tiempo)
                i_data.append(datos[0])
                w_data.append(datos[1])

                line1.set_xdata(t_data)
                line1.set_ydata(i_data)

                line2.set_xdata(t_data)
                line2.set_ydata(w_data)

                V = 5 * np.ones_like(t_data)

                fig.canvas.draw()
                fig.canvas.flush_events()



except KeyboardInterrupt or (tiempoM - inicio <= 1):
    print("Finalizado :)")
    arduino.close()

    params = estimate_dc_motor_params(t_data, V, i_data, w_data)
    print("\nParámetros estimados del motor DC:")
    for k, v in params.items():
        print(f"{k}: {v:.6f}")

    plt.ioff()
    plt.show()




