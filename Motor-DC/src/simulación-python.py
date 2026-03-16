"""
Simulación y Estimación de Parámetros de Motor DC
===============================================

Este programa realiza:
1. Adquisición de datos en tiempo real desde Arduino (corriente y velocidad)
2. Estimación de parámetros del motor DC usando mínimos cuadrados
3. Simulación del modelo dinámico con los parámetros estimados
4. Comparación visual entre datos experimentales y simulación

"""

import serial
import numpy as np
from scipy.optimize import curve_fit
import time
import matplotlib.pyplot as plt

# =============================================================================
# CONFIGURACIÓN DEL SISTEMA
# =============================================================================

# Configuración del puerto serial para comunicación con Arduino
PuertoSerial = ''  # Ejemplo: 'COM3' 
Baudrate = '115200'      # Velocidad de comunicación, ej: 9600, 115200

# Variables globales para almacenar datos experimentales
t_data = []  # Lista de tiempos (timestamps)
i_data = []  # Lista de corrientes medidas (A)
w_data = []  # Lista de velocidades medidas (rad/s)

# Parámetros de la simulación
tiempoM = 5  # Duración máxima de la medición en segundos

# =============================================================================
# CONFIGURACIÓN DE GRÁFICAS EN TIEMPO REAL
# =============================================================================

# Activar modo interactivo para actualización en tiempo real
plt.ion()

# Crear figura con dos subgráficas (2 filas, 1 columna)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

# Crear líneas iniciales vacías para las gráficas en tiempo real
line1, = ax1.plot([], [], 'b-', label="Corriente")  # Línea azul para corriente
line2, = ax2.plot([], [], 'r-', label="Velocidad")  # Línea roja para velocidad

# Configurar leyendas y rejillas
ax1.legend()
ax2.legend()
ax1.grid(True)
ax2.grid(True)

# =============================================================================
# FUNCIONES DE MODELADO Y SIMULACIÓN
# =============================================================================

def simulate_dc_motor_model(params, V, t_experimental):
    """
    Simula el modelo dinámico del motor DC usando los parámetros estimados.
    Retorna i_sim y w_sim sobre el mismo vector de tiempo experimental.

    Parámetros:
    - params: Diccionario con parámetros estimados del motor
    - V: Voltaje aplicado (constante)
    - t_experimental: Vector de tiempo experimental

    Retorna:
    - i_sim: Corriente simulada (array)
    - w_sim: Velocidad simulada (array)
    """
    from scipy.integrate import odeint

    # Extraer parámetros del diccionario para facilitar el acceso
    R_a = params["R_a (Ohm)"]        # Resistencia del armadura
    L_a = params["L_a (H)"]          # Inductancia del armadura
    K_e = params["K_e (V·s/rad)"]    # Constante de fuerza contraelectromotriz
    K_t = params["K_t (N·m/A)"]      # Constante de torque
    J = params["J (kg·m²)"]          # Momento de inercia del rotor
    B = params["B (N·m·s/rad)"]      # Coeficiente de fricción viscosa

    # Protección contra inductancia muy pequeña (evita división por cero)
    if abs(L_a) < 1e-6:
        L_a = 1e-6

    # Definir el sistema de ecuaciones diferenciales ordinarias (ODEs)
    # Estado del sistema: y = [i, w] donde i=corriente, w=velocidad angular
    def motor_ode(y, t, V_input):
        i, w = y  # Desempaquetar estado actual

        # Ecuación eléctrica del circuito del armadura:
        # L_a * di/dt + R_a * i + K_e * w = V
        # Reordenando: di/dt = (V - R_a*i - K_e*w) / L_a
        di_dt = (V_input - R_a * i - K_e * w) / L_a

        # Ecuación mecánica del rotor:
        # J * dw/dt + B * w = K_t * i
        # Reordenando: dw/dt = (K_t*i - B*w) / J
        dw_dt = (K_t * i - B * w) / J

        return [di_dt, dw_dt]  # Retornar derivadas

    # Condiciones iniciales: motor en reposo
    y0 = [0, 0]  # i0=0 A, w0=0 rad/s

    # Resolver el sistema de ODEs usando integración numérica
    # args=(V,) pasa el voltaje como parámetro adicional a motor_ode
    solution = odeint(motor_ode, y0, t_experimental, args=(V,))

    # Extraer resultados de la solución
    i_sim = solution[:, 0]  # Corriente simulada
    w_sim = solution[:, 1]  # Velocidad simulada

    return i_sim, w_sim

def estimate_dc_motor_params(time, voltage, current, speed):
    """
    Estima los parámetros del motor DC usando mínimos cuadrados lineales.

    El modelo del motor DC tiene 6 parámetros principales:
    - R_a: Resistencia del armadura [Ohm]
    - L_a: Inductancia del armadura [H]
    - K_e: Constante de fuerza contraelectromotriz [V·s/rad]
    - K_t: Constante de torque [N·m/A]
    - J: Momento de inercia del rotor [kg·m²]
    - B: Coeficiente de fricción viscosa [N·m·s/rad]

    Método: Resuelve el sistema sobredeterminado usando mínimos cuadrados
    """

    # Calcular derivadas numéricas usando diferencias finitas
    dt = np.gradient(time)           # Paso de tiempo entre muestras
    di_dt = np.gradient(current, dt) # Derivada de la corriente
    dw_dt = np.gradient(speed, dt)   # Derivada de la velocidad

    # =========================================================================
    # ESTIMACIÓN DE PARÁMETROS ELÉCTRICOS
    # =========================================================================
    # Ecuación del circuito eléctrico: V = R_a * i + L_a * di/dt + K_e * ω
    # Reordenando: V - R_a*i - L_a*di/dt - K_e*ω = 0
    # Forma matricial: X_elec * [R_a, L_a, K_e]^T = V

    # Matriz de diseño para parámetros eléctricos
    X_elec = np.column_stack((current, di_dt, speed))
    # Resolver usando mínimos cuadrados: X_elec * params_elec = voltage
    params_elec, _, _, _ = np.linalg.lstsq(X_elec, voltage, rcond=None)
    R_a, L_a, K_e = params_elec

    # =========================================================================
    # ESTIMACIÓN DE PARÁMETROS MECÁNICOS
    # =========================================================================
    # Ecuación mecánica: J * dω/dt + B * ω = K_t * i
    # Reordenando: J*dω/dt + B*ω - K_t*i = 0
    # Forma matricial: X_mech * [J, B]^T = K_t * i

    # Matriz de diseño para parámetros mecánicos
    X_mech = np.column_stack((dw_dt, speed))
    # Resolver: X_mech * params_mech = K_t * current
    params_mech, _, _, _ = np.linalg.lstsq(X_mech, current * K_e, rcond=None)
    J, B = params_mech
    K_t = K_e  # En motores DC de imán permanente, Kt ≈ Ke (igualdad teórica)

    # Retornar parámetros en un diccionario con unidades
    return {
        "R_a (Ohm)": R_a,
        "L_a (H)": L_a,
        "K_e (V·s/rad)": K_e,
        "K_t (N·m/A)": K_t,
        "J (kg·m²)": J,
        "B (N·m·s/rad)": B
    }



# =============================================================================
# PROGRAMA PRINCIPAL - ADQUISICIÓN DE DATOS Y PROCESAMIENTO
# =============================================================================

try:
    # Solicitar confirmación al usuario para iniciar la medición
    raw = input('Iniciar recolección de datos? (S/N)')

    # Validar entrada del usuario
    if raw != 'S' and raw != 'N':
        print("Error en el comando de inicio")

    # Bucle principal de adquisición de datos
    while True:

        # Si el usuario elige 'N', salir del programa
        if raw == 'N':
            break

        # Si el usuario elige 'S', iniciar adquisición de datos
        elif raw == 'S':

            # Establecer conexión serial con Arduino
            arduino = serial.Serial(PuertoSerial, Baudrate, timeout=2)
            # timeout=2 significa que espera máximo 2 segundos por datos del Arduino

            # Esperar 2 segundos para que se estabilice la conexión
            time.sleep(2)

            # Bucle de adquisición de datos en tiempo real
            while True:
                # Registrar timestamp actual
                tiempo = time.time()

                # Leer línea completa desde el puerto serial
                line = arduino.readline().decode("utf-8").strip()

                # Separar los datos (formato esperado: "corriente,velocidad")
                datos = line.split(",")

                # Verificar que se recibió una línea válida con datos
                if line:
                    # Almacenar datos experimentales
                    # Nota: tiempo relativo al primer dato para normalizar
                    t_data.append(tiempo - t_data[0] if t_data else 0)
                    i_data.append(float(datos[0]))  # Convertir corriente a float
                    w_data.append(float(datos[1]))  # Convertir velocidad a float

                    # Actualizar gráfica de corriente en tiempo real
                    line1.set_xdata(t_data)
                    line1.set_ydata(i_data)

                    # Actualizar gráfica de velocidad en tiempo real
                    line2.set_xdata(t_data)
                    line2.set_ydata(w_data)

                    # Crear vector de voltaje constante (asumiendo 5V aplicado)
                    V = 5 * np.ones_like(t_data)

                    # Actualizar las gráficas en la ventana
                    fig.canvas.draw()
                    fig.canvas.flush_events()

                # Condición de parada: tiempo máximo alcanzado
                if tiempoM - (tiempo - (t_data[0] if t_data else tiempo)) <= 1:
                    break

# Manejo de interrupción por teclado (Ctrl+C)
except KeyboardInterrupt:
    print("Finalizado :)")
    arduino.close()

    # =========================================================================
    # PROCESAMIENTO FINAL DE DATOS
    # =========================================================================

    # Convertir listas a arrays de NumPy para procesamiento numérico eficiente
    t_data = np.array(t_data)
    i_data = np.array(i_data)
    w_data = np.array(w_data)

    # Estimar parámetros del motor usando mínimos cuadrados
    params = estimate_dc_motor_params(t_data, V, i_data, w_data)

    # Mostrar parámetros estimados
    print("\nParámetros estimados del motor DC:")
    for k, v in params.items():
        print(f"{k}: {v:.6f}")

    # =========================================================================
    # SIMULACIÓN Y COMPARACIÓN VISUAL
    # =========================================================================

    # Simular el comportamiento del motor con los parámetros estimados
    i_sim, w_sim = simulate_dc_motor_model(params, V, t_data)

    # Cambiar a modo no interactivo para la visualización final
    plt.ioff()

    # Limpiar las gráficas anteriores
    ax1.clear()
    ax2.clear()

    # =========================================================================
    # GRÁFICA 1: CORRIENTE - COMPARACIÓN EXPERIMENTAL VS SIMULADA
    # =========================================================================
    ax1.plot(t_data, i_data, 'b-', linewidth=2, label='Corriente Experimental')
    ax1.plot(t_data, i_sim, 'b--', linewidth=2, label='Corriente Simulada')
    ax1.set_xlabel('Tiempo (s)')
    ax1.set_ylabel('Corriente (A)')
    ax1.set_title('Comparación: Corriente Experimental vs Simulada')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # =========================================================================
    # GRÁFICA 2: VELOCIDAD - COMPARACIÓN EXPERIMENTAL VS SIMULADA
    # =========================================================================
    ax2.plot(t_data, w_data, 'r-', linewidth=2, label='Velocidad Experimental')
    ax2.plot(t_data, w_sim, 'r--', linewidth=2, label='Velocidad Simulada')
    ax2.set_xlabel('Tiempo (s)')
    ax2.set_ylabel('Velocidad (rad/s)')
    ax2.set_title('Comparación: Velocidad Experimental vs Simulada')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Ajustar automáticamente el espaciado entre subgráficas
    fig.tight_layout()

    # Mostrar las gráficas finales
    plt.show()




