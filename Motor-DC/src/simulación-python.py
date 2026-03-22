"""
Simulación y Estimación de Parámetros de Motor DC
===============================================
"""

import serial
import numpy as np
import time
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# =============================================================================
# CONFIGURACIÓN DEL SISTEMA
# =============================================================================

# Configuración del puerto serial para comunicación con Arduino
PuertoSerial = 'COM3'  # Ejemplo: 'COM3' - CAMBIAR según tu puerto
Baudrate = 115200      # Velocidad de comunicación, ej: 9600, 115200

# Variables globales para almacenar datos experimentales
t_data = []  # Lista de tiempos (timestamps)
i_data = []  # Lista de corrientes medidas (A)
w_data = []  # Lista de velocidades medidas (rad/s)

# Parámetros de la simulación
t_comienzo = 0
t_fin = 5  # Duración máxima de la medición en segundos
num_puntos = 600

Volt = 5
t_inicio = 0  # Variable global para almacenar el tiempo inicial

# =============================================================================
# CONFIGURACIÓN DE GRÁFICAS EN TIEMPO REAL
# =============================================================================

# Activar modo interactivo para actualización en tiempo real
plt.ion()

# Crear figura con dos subgráficas (2 filas, 1 columna)
fig_rt, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

# Crear líneas iniciales vacías para las gráficas en tiempo real
line1, = ax1.plot([], [], 'b-', label="Corriente")  
line2, = ax2.plot([], [], 'r-', label="Velocidad")  

# Configurar leyendas y rejillas
ax1.legend()
ax2.legend()
ax1.grid(True)
ax2.grid(True)

# =============================================================================
# FUNCIONES DE MODELADO Y SIMULACIÓN
# =============================================================================

def simulate_dc_motor_model(params, V_arr, t_arr):
    """
    Simula el modelo dinámico del motor DC usando los parámetros estimados.
    """
    R_a = params["R_a (Ohm)"]        
    L_a = params["L_a (H)"]          
    Ke = params["K_e (V·s/rad)"]     
    Jm = params["J (kg·m²)"]          
    B = params["B (N·m·s/rad)"]      

    if abs(L_a) < 1e-6:
        L_a = 1e-6

    def motor_dc(t, x, V_arr_inner, t_arr_inner, R, K, L, b, J):
        i, omega = x
        # Interpolar el voltaje para el instante exacto t
        V_t = np.interp(t, t_arr_inner, V_arr_inner)
        
        di_dt = (V_t - R * i - K * omega) / L
        domega_dt = (K * i - b * omega) / J
        return [di_dt, domega_dt]
    
    i0, w0 = 0, 0
    t_span = [t_arr[0], t_arr[-1]]
    ode_args = (V_arr, t_arr, R_a, Ke, L_a, B, Jm)

    # Integraciones numéricas
    sol1 = solve_ivp(motor_dc, t_span, [i0, w0], method='RK45', t_eval=t_arr, args=ode_args)
    sol2 = solve_ivp(motor_dc, t_span, [i0, w0], method='RK23', t_eval=t_arr, args=ode_args)
    sol3 = solve_ivp(motor_dc, t_span, [i0, w0], method='BDF', t_eval=t_arr, args=ode_args)
    sol4 = solve_ivp(motor_dc, t_span, [i0, w0], method='Radau', t_eval=t_arr, args=ode_args)

    if not (sol1.success and sol2.success and sol3.success and sol4.success):
        raise RuntimeError("Error en una o más integraciones numéricas.")

    return (sol1.y[0], sol1.y[1], sol2.y[0], sol2.y[1],
            sol3.y[0], sol3.y[1], sol4.y[0], sol4.y[1])

def estimate_dc_motor_params(time_arr, voltage, current, speed):
    """
    Estima los parámetros del motor DC usando mínimos cuadrados lineales.
    """
    # CORRECCIÓN: Derivadas numéricas correctas en NumPy
    di_dt = np.gradient(current, time_arr) 
    dw_dt = np.gradient(speed, time_arr)  

    # ESTIMACIÓN DE PARÁMETROS ELÉCTRICOS
    X_elec = np.column_stack((current, di_dt, speed))
    params_elec, _, _, _ = np.linalg.lstsq(X_elec, voltage, rcond=None)
    R_a, L_a, K_e = params_elec

    # ESTIMACIÓN DE PARÁMETROS MECÁNICOS
    X_mech = np.column_stack((dw_dt, speed))
    params_mech, _, _, _ = np.linalg.lstsq(X_mech, current * K_e, rcond=None)
    J, B = params_mech
    K_t = K_e  

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
    raw = input('Iniciar recolección de datos? (S/N): ').strip().upper()

    if raw == 'S':
        if not PuertoSerial:
            print("Error: Puerto serial no configurado.")
            exit()

        try:
            arduino = serial.Serial(PuertoSerial, Baudrate, timeout=2)
            time.sleep(2) # Tiempo para estabilizar el Arduino
            t_inicio = time.time()
            
            print("Recolectando datos... Presiona Ctrl+C para detener o espera 5 segundos.")

            while True:
                tiempo = time.time()

                try:
                    line = arduino.readline().decode("utf-8").strip()
                except Exception as e:
                    print(f"Error al leer datos: {e}")
                    continue

                datos = line.split(",")

                if line and len(datos) >= 2:
                    try:
                        t_data.append(tiempo - t_inicio)
                        i_data.append(float(datos[0]))  
                        w_data.append(float(datos[1]))  

                        # Actualizar gráficas en tiempo real
                        line1.set_xdata(t_data)
                        line1.set_ydata(i_data)
                        ax1.relim()
                        ax1.autoscale_view()

                        line2.set_xdata(t_data)
                        line2.set_ydata(w_data)
                        ax2.relim()
                        ax2.autoscale_view()

                        fig_rt.canvas.draw()
                        fig_rt.canvas.flush_events()
                    except ValueError:
                        continue

                # Condición de parada automática
                if tiempo - t_inicio >= t_fin:
                    print("Tiempo máximo alcanzado.")
                    break
        except KeyboardInterrupt:
            print("\nAdquisición finalizada por el usuario.")

finally:
    try:
        arduino.close()
    except:
        pass

# =========================================================================
# PROCESAMIENTO FINAL Y SIMULACIÓN
# =========================================================================

if len(t_data) == 0:
    print("Error: No se recolectaron datos.")
    exit()

# Apagar modo interactivo para que las gráficas finales no se cierren
plt.ioff()
plt.close(fig_rt) # Cerrar la gráfica de tiempo real

# Convertir a numpy arrays
t_data = np.array(t_data)
i_data = np.array(i_data)
w_data = np.array(w_data)

# Vector de voltaje aplicado (5V constante)
V_exp = 5 * np.ones_like(t_data)

# Estimar parámetros
params = estimate_dc_motor_params(t_data, V_exp, i_data, w_data)
print("\nParámetros estimados del motor DC:")
for k, v in params.items():
    print(f"{k}: {v:.6f}")

# Simular con tiempo un poco más fino
t_eval = np.linspace(t_data[0], t_data[-1], 500)
V_sim = 5 * np.ones_like(t_eval)

(i_sim1, w_sim1, 
 i_sim2, w_sim2,
 i_sim3, w_sim3,
 i_sim4, w_sim4) = simulate_dc_motor_model(params, V_sim, t_eval)

# Interpolar experimentales para graficar a la par
i_data_interp = np.interp(t_eval, t_data, i_data)
w_data_interp = np.interp(t_eval, t_data, w_data)

# =========================================================================
# VISUALIZACIÓN FINAL COMPACTADA
# =========================================================================

fig, axes = plt.subplots(2, 4, figsize=(16, 8), sharex=True)

methods = ['RK45', 'RK23', 'BDF', 'Radau']
i_sims = [i_sim1, i_sim2, i_sim3, i_sim4]
w_sims = [w_sim1, w_sim2, w_sim3, w_sim4]

for col in range(4):
    # Gráficas de corriente (Fila 0)
    axes[0, col].plot(t_eval, i_data_interp, 'blue', alpha=0.5, label='Experimental', linewidth=2)
    axes[0, col].plot(t_eval, i_sims[col], 'red', label='Simulado', linestyle='--')   
    axes[0, col].set_title(f"Corriente - {methods[col]}")
    axes[0, col].set_ylabel("Corriente (A)")
    axes[0, col].legend()
    axes[0, col].grid()

    # Gráficas de velocidad (Fila 1)
    axes[1, col].plot(t_eval, w_data_interp, 'blue', alpha=0.5, label='Experimental', linewidth=2)
    axes[1, col].plot(t_eval, w_sims[col], 'red', label='Simulado', linestyle='--')   
    axes[1, col].set_title(f"Velocidad - {methods[col]}")
    axes[1, col].set_xlabel("Tiempo (s)")
    axes[1, col].set_ylabel("Velocidad (rad/s)")
    axes[1, col].legend()
    axes[1, col].grid()

fig.tight_layout()
plt.show()