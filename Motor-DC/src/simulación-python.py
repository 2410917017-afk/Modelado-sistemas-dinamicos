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
PuertoSerial = 'COM7'  
Baudrate = 115200      # Velocidad de comunicación

# Variables globales para almacenar datos experimentales
t_data = []  # Lista de tiempos (timestamps)
i_data = []  # Lista de corrientes medidas (A)
w_data = []  # Lista de velocidades medidas (rad/s)

# Parámetros de la simulación
t_comienzo = 0
t_fin = 5  # Duración máxima de la medición en segundos
num_puntos = 2000

Volt = 12
t_inicio = 0  # Variable global para almacenar el tiempo inicial

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
    Filtra datos transitorios iniciales para mejorar la estimación.
    """
    # FILTRAR: Descartar primeros 10% de datos (transitorios de arranque)
    idx_start = int(len(time_arr) * 0.1)
    
    time_arr = time_arr[idx_start:]
    voltage = voltage[idx_start:]
    current = current[idx_start:]
    speed = speed[idx_start:]
    
    # VALIDAR: Descartar datos anómalos con rango razonable
    current = np.clip(current, -5, 5)  # Rango razonable para corriente (A)
    speed = np.clip(speed, -100, 100)  # Rango razonable para velocidad (rad/s)
    
    # CORRECCIÓN: Derivadas numéricas correctas en NumPy
    di_dt = np.gradient(current, time_arr) 
    dw_dt = np.gradient(speed, time_arr)  

    # ESTIMACIÓN DE PARÁMETROS ELÉCTRICOS
    X_elec = np.column_stack((current, di_dt, speed))
    params_elec, _, _, _ = np.linalg.lstsq(X_elec, voltage, rcond=None)
    R_a, L_a, K_e = params_elec

    # Validar valores físicos razonables para parámetros eléctricos
    R_a = np.clip(R_a, 0.1, 100)      # Resistencia: 0.1-100 Ohm
    L_a = np.clip(L_a, 0, 1)          # Inductancia: 0-1 H
    K_e = np.clip(K_e, 0.001, 1)      # Constante: 0.001-1 V·s/rad

    # ESTIMACIÓN DE PARÁMETROS MECÁNICOS
    X_mech = np.column_stack((dw_dt, speed))
    params_mech, _, _, _ = np.linalg.lstsq(X_mech, current * K_e, rcond=None)
    J, B = params_mech
    
    # Validar valores físicos razonables para parámetros mecánicos
    J = np.clip(J, 1e-6, 1)           # Inercia: 1e-6 a 1 kg·m²
    B = np.clip(B, 1e-4, 1)           # Amortiguamiento: pequeño pero positivo
    
    K_t = K_e  # Por reciprocidad: K_t = K_e en motor DC

    return {
        "R_a (Ohm)": abs(R_a),
        "L_a (H)": abs(L_a),
        "K_e (V·s/rad)": abs(K_e),
        "K_t (N·m/A)": abs(K_t),
        "J (kg·m²)": abs(J),
        "B (N·m·s/rad)": abs(B)
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
            arduino = serial.Serial(PuertoSerial, Baudrate, timeout=3)
            time.sleep(2)
            
            # ===== SINCRONIZACIÓN: Esperar confirmación de Arduino =====
            print("Esperando confirmación de Arduino...")
            arduino_listo = False
            tiempo_espera = time.time()
            
            while time.time() - tiempo_espera < 10:  # Timeout 10 segundos
                try:
                    confirmacion = arduino.readline().decode("utf-8").strip()
                    if "ARDUINO_LISTO" in confirmacion:
                        print("✓ Arduino listo para comunicarse")
                        arduino_listo = True
                        break
                except:
                    continue
            
            if not arduino_listo:
                print("✗ Error: Arduino no respondió. Verifica la conexión.")
                exit()
            
            # ===== ENVIAR COMANDO DE ARRANQUE AL MOTOR =====
            print("Arrancando motor...")
            arduino.write(b"START\n")
            
            # Esperar confirmación de arranque
            while time.time() - tiempo_espera < 15:
                try:
                    respuesta = arduino.readline().decode("utf-8").strip()
                    if "MOTOR_INICIADO" in respuesta:
                        print("✓ Motor arrancado correctamente")
                        break
                except:
                    continue
            
            # ===== ESPERAR ESTABILIZACIÓN (régimen permanente) =====
            print("Esperando estabilización (2 segundos)...")
            time.sleep(2)
            
            t_inicio = time.time()
            t_datos_inicio = None  # Timestamp del Arduino en el primer dato válido
            
            print("Recolectando datos... Presiona Ctrl+C para detener o espera 5 segundos.")
            print("Formato esperado: tiempo_arduino(ms),corriente(A),velocidad(rad/s)")

            while True:
                tiempo_actual = time.time()

                try:
                    line = arduino.readline().decode("utf-8").strip()
                except Exception as e:
                    print(f"⚠ Error al leer datos: {e}")
                    continue

                datos = line.split(",")

                # Esperamos 3 campos: tiempo_arduino, corriente, velocidad
                if line and len(datos) >= 3:
                    try:
                        t_arduino = int(datos[0])      # Timestamp del Arduino (ms desde arranque)
                        i_medida = float(datos[1])     # Corriente (A)
                        w_medida = float(datos[2])     # Velocidad (rad/s)
                        
                        # SINCRONIZACIÓN: Registrar tiempo del Arduino en primer dato
                        if t_datos_inicio is None:
                            t_datos_inicio = t_arduino
                        
                        # Convertir tiempo Arduino a segundos desde arranque
                        t_sync = (t_arduino - t_datos_inicio) / 1000.0
                        
                        t_data.append(t_sync)
                        i_data.append(i_medida)
                        w_data.append(w_medida)
                        
                    except ValueError:
                        continue

                # Condición de parada automática
                if tiempo_actual - t_inicio >= t_fin:
                    print(f"✓ Tiempo máximo ({t_fin}s) alcanzado.")
                    break
                    
        except KeyboardInterrupt:
            print("\n✓ Adquisición finalizada por el usuario.")
        finally:
            try:
                # Detener el motor
                arduino.write(b"STOP\n")
                time.sleep(0.5)
                arduino.close()
            except:
                pass

except Exception as e:
    print(f"Error en el programa: {e}")
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

# Convertir a numpy arrays
t_data = np.array(t_data)
i_data = np.array(i_data)
w_data = np.array(w_data)

# =========================================================================
# ANÁLISIS DEL TIEMPO DE MUESTREO
# =========================================================================

print("\n" + "="*70)
print("ANÁLISIS DEL TIEMPO DE MUESTREO")
print("="*70)

# Calcular intervalos entre muestras
dt_samples = np.diff(t_data)
ts_mean = np.mean(dt_samples)
ts_std = np.std(dt_samples)
ts_min = np.min(dt_samples)
ts_max = np.max(dt_samples)
fs_mean = 1.0 / ts_mean if ts_mean > 0 else np.inf

print(f"Número total de muestras: {len(t_data)}")
print(f"Duración total: {t_data[-1]:.2f} segundos")
print(f"\nIntervalo de muestreo (Ts):")
print(f"  Promedio: {ts_mean*1000:.2f} ms ({fs_mean:.2f} Hz)")
print(f"  Desv. Est.: {ts_std*1000:.2f} ms")
print(f"  Mínimo: {ts_min*1000:.2f} ms")
print(f"  Máximo: {ts_max*1000:.2f} ms")
print(f"  Variación: {(ts_std/ts_mean)*100:.1f}%")

# Analizar consistencia del muestreo
ts_nominal = 0.090  # Arduino cada 90ms (velocidad)
variacion_permisible = 0.10  # 10% de variación

if ts_std / ts_mean < variacion_permisible:
    print(f"✓ Muestreo CONSISTENTE (variación < {variacion_permisible*100}%)")
else:
    print(f"⚠ Muestreo VARIABLE (variación > {variacion_permisible*100}%)")
    print(f"  Esto puede afectar la precisión de las derivadas")

# Detectar frecuencia de Nyquist
nyquist_freq = fs_mean / 2
print(f"\nFrecuencia de Nyquist: {nyquist_freq:.2f} Hz")
print(f"Dinámica del motor DC típica: < 100 Hz")

if nyquist_freq > 100:
    print(f"✓ Frecuencia de muestreo ADECUADA para dinamica del motor")
else:
    print(f"⚠ Frecuencia de muestreo BAJA para dinamica completa del motor")

# Vector de voltaje aplicado
V_exp = Volt * np.ones_like(t_data)

print("\n" + "="*70)

# Estimar parámetros
params = estimate_dc_motor_params(t_data, V_exp, i_data, w_data)
print("\nParámetros estimados del motor DC:")
for k, v in params.items():
    print(f"{k}: {v:.6f}")

# Simular con tiempo un poco más fino
t_eval = np.linspace(t_data[0], t_data[-1], 500)
V_sim = 12 * np.ones_like(t_eval)

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

fig, axes = plt.subplots(2, 4, figsize=(18, 8), sharex=True)

methods = ['RK45', 'RK23', 'BDF', 'Radau']
i_sims = [i_sim1, i_sim2, i_sim3, i_sim4]
w_sims = [w_sim1, w_sim2, w_sim3, w_sim4]

for col in range(4):
    # Gráficas de corriente (Fila 0)
    axes[0, col].plot(t_eval, i_data_interp, 'blue', alpha=0.6, label='Experimental', linewidth=2.5)
    axes[0, col].plot(t_eval, i_sims[col], 'red', label='Simulado', linestyle='--', linewidth=1.5)   
    axes[0, col].set_title(f"Corriente - {methods[col]}", fontsize=11, fontweight='bold')
    axes[0, col].set_ylabel("Corriente (A)", fontsize=10)
    axes[0, col].legend(loc='best', fontsize=9)
    axes[0, col].grid(True, alpha=0.3)

    # Gráficas de velocidad (Fila 1)
    axes[1, col].plot(t_eval, w_data_interp, 'blue', alpha=0.6, label='Experimental', linewidth=2.5)
    axes[1, col].plot(t_eval, w_sims[col], 'red', label='Simulado', linestyle='--', linewidth=1.5)   
    axes[1, col].set_title(f"Velocidad - {methods[col]}", fontsize=11, fontweight='bold')
    axes[1, col].set_xlabel("Tiempo (s)", fontsize=10)
    axes[1, col].set_ylabel("Velocidad (rad/s)", fontsize=10)
    axes[1, col].legend(loc='best', fontsize=9)
    axes[1, col].grid(True, alpha=0.3)

# Agregar información de muestreo en el título general
fig.suptitle(f'Comparación Datos Experimentales vs Simulación\n' + 
             f'Fs={fs_mean:.2f} Hz, Ts={ts_mean*1000:.2f} ms, Muestras={len(t_data)}',
             fontsize=12, fontweight='bold', y=1.00)

fig.tight_layout()
plt.show()