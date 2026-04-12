"""
DAQ + Estimación de parámetros + Simulación — Motor DC
Coordinado con Arduino simplificado (motor arranca en setup).
Estimación: Savitzky-Golay para derivadas numéricas estables.
"""

import serial
import numpy as np
import time
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.signal import savgol_filter


# CONFIGURACIÓN

PORT   = 'COM11'
BAUD   = 115200
T_FIN  = 2.0   # segundos de captura
VOLT   = 12.0  # voltaje aplicado (V)


# ADQUISICIÓN DE DATOS

t_raw, i_raw, w_raw, v_raw = [], [], [], []

print("Conectando a Arduino...")
arduino = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(0.5)                  # Solo esperar estabilización del USB
arduino.reset_input_buffer()     # Limpiar buffer
print(f"Recolectando {T_FIN}s de datos (desde el arranque)...")

t0 = time.time()
while time.time() - t0 < T_FIN:
    try:
        arduino.write(b'\x01')
        line = arduino.readline().decode('utf-8', errors='ignore').strip()
        parts = line.split(',')
        if len(parts) == 4:
            t_ms = int(parts[0])
            v    = float(parts[1])
            i    = float(parts[2])
            w    = float(parts[3])
            t_raw.append(t_ms / 1000.0)
            v_raw.append(v)
            i_raw.append(i)
            w_raw.append(w)
    except (ValueError, UnicodeDecodeError):
        pass

arduino.close()
print(f"✓ {len(t_raw)} muestras capturadas.")

if len(t_raw) < 10:
    print("Error: datos insuficientes.")
    exit()

# Convertir y normalizar tiempo
t = np.array(t_raw) - t_raw[0]
i = np.array(i_raw)
w = np.array(w_raw)
vnp = np.array(v_raw)


# ANÁLISIS DE MUESTREO

dt   = np.diff(t)
fs   = 1.0 / np.mean(dt)
print(f"\nFs promedio : {fs:.1f} Hz  |  Ts promedio : {np.mean(dt)*1000:.2f} ms")
print(f"Variación Ts: {np.std(dt)/np.mean(dt)*100:.1f}%")


# ESTIMACIÓN DE PARÁMETROS — Nivel 1: Savitzky-Golay

def estimar_params(t, i, w):
    """
    Mínimos cuadrados con derivadas calculadas via Savitzky-Golay.
    - No descarta el transitorio (es donde J y L son identificables).
    - SG filtra y deriva simultáneamente → mucho más estable que np.gradient.
    - La ventana debe ser impar y > polyorder. Ajustar según densidad de muestras.
    """
    i = np.clip(i, -5,   5  )
    w = np.clip(w, -200, 200)
    V_vec = vnp

    # Ventana SG: ~10% del total de muestras, mínimo 7, siempre impar
    n = len(t)
    win = max(7, int(n * 0.10) | 1)   # '| 1' fuerza impar
    poly = 3                            # Grado del polinomio

    # Señales suavizadas (para visualización y para el lado derecho de las ecuaciones)
    i_sg = savgol_filter(i, win, poly)
    w_sg = savgol_filter(w, win, poly)

    # Derivadas: SG calcula la derivada analítica del polinomio local → muy limpio
    ts_medio = np.mean(np.diff(t))     # Ts promedio (necesario para el argumento delta)
    di_dt = savgol_filter(i, win, poly, deriv=1, delta=ts_medio)
    dw_dt = savgol_filter(w, win, poly, deriv=1, delta=ts_medio)

    # --- Parámetros eléctricos: V = R·i + L·di/dt + Ke·w ---
    A_elec = np.column_stack((i_sg, di_dt, w_sg))
    R, L, Ke = np.linalg.lstsq(A_elec, V_vec, rcond=None)[0]

    # --- Parámetros mecánicos: Ke·i = J·dw/dt + B·w ---
    A_mec = np.column_stack((dw_dt, w_sg))
    J, B  = np.linalg.lstsq(A_mec, abs(Ke) * i_sg, rcond=None)[0]

    params = {
        "R_a (Ohm)"     : abs(np.clip(R,  0.1,   50  )),
        "L_a (H)"       : abs(np.clip(L,  1e-4, 0.1 )),
        "K_e (V·s/rad)" : abs(np.clip(Ke, 0.01, 0.5 )),
        "K_t (N·m/A)"   : abs(np.clip(Ke, 0.01, 0.5 )),
        "J (kg·m²)"     : abs(np.clip(J,  1e-5, 0.1 )),
        "B (N·m·s/rad)" : abs(np.clip(B,  1e-5, 0.1 )),
    }
    # Devolver señales suavizadas para graficarlas aparte
    return params, i_sg, w_sg

params, i_suave, w_suave = estimar_params(t, i, w)
print("\nParámetros estimados (Savitzky-Golay):")
for k, v in params.items():
    print(f"  {k}: {v:.6f}")


# SIMULACIÓN DEL MOTOR CON 4 MÉTODOS NUMÉRICOS

def simular_motor(params, t_eval, t_data, v_data):
    R  = params["R_a (Ohm)"]
    L  = max(params["L_a (H)"], 1e-4)  # Protección adicional contra L muy pequeño
    Ke = params["K_e (V·s/rad)"]
    J  = max(params["J (kg·m²)"], 1e-5)  # Protección: J mínimo
    B  = params["B (N·m·s/rad)"]

    # ── v(t) real interpolado — el solver lo evaluará en sus pasos internos ──
    # np.interp es O(log n) con búsqueda binaria → no penaliza el rendimiento
    def V_interp(t_actual):
        return np.interp(t_actual, t_data, v_data)

    def odes(t_actual, x):
        V = V_interp(t_actual)      # <── voltaje real en ese instante
        return [(V - R*x[0] - Ke*x[1]) / L,
                (Ke*x[0]  -  B*x[1]) / J]

    # Jacobiano sigue siendo el mismo — no depende de V
    jac = np.array([[-R/L,  -Ke/L],
                    [ Ke/J,  -B/J]])
    jac_fn = lambda t_actual, x: jac

    tspan   = (t_eval[0], t_eval[-1])
    dur     = tspan[1] - tspan[0]

    configs = {
        'RK45' : dict(method='RK45',                rtol=5e-3, atol=1e-4,
                      max_step=dur/100),
        'RK23' : dict(method='RK23',                rtol=5e-3, atol=1e-4,
                      max_step=dur/100),
        'BDF'  : dict(method='BDF',   jac=jac_fn, rtol=1e-3, atol=1e-5,
                      max_step=dur/50),
        'Radau': dict(method='Radau', jac=jac_fn, rtol=1e-3, atol=1e-5,
                      max_step=dur/50),
    }

    resultados = {}
    tiempos    = {}

    for nombre, cfg in configs.items():
        t0  = time.perf_counter()
        sol = solve_ivp(odes, tspan, [0.0, 0.0],
                        t_eval=t_eval,
                        dense_output=False,
                        **cfg)
        elapsed = time.perf_counter() - t0

        if sol.success:
            resultados[nombre] = (sol.y[0], sol.y[1])
            tiempos[nombre]    = elapsed
            print(f"  {nombre:6s} ✓  {elapsed*1000:6.1f} ms  |  pasos={sol.t.size}")
        else:
            print(f"  {nombre:6s} ✗  {sol.message}")

    return resultados, tiempos

t_sim = np.linspace(t[0], t[-1], 500)
resultados, tiempos = simular_motor(params, t_sim, t, vnp)


# VISUALIZACIÓN

metodos  = list(resultados.keys())
i_raw_interp  = np.interp(t_sim, t, i)
w_raw_interp  = np.interp(t_sim, t, w)
i_sg_interp   = np.interp(t_sim, t, i_suave)
w_sg_interp   = np.interp(t_sim, t, w_suave)

fig, axes = plt.subplots(2, len(metodos), figsize=(16, 7), sharex=True)
fig.suptitle(
    f"Motor DC — Raw / Filtrado / Simulación  |  "
    f"Fs={fs:.1f} Hz  |  N={len(t)} muestras\n"
    f"Parámetros estimados — R={params["R_a (Ohm)"]} Ohm  L={params["L_a (H)"]} H"
    f"  K={params["K_e (V·s/rad)"]}  J={params["J (kg·m²)"]} (kg·m²)"
    f"  B={params["B (N·m·s/rad)"]} (N·m·s/rad)",
    fontweight='bold', fontsize=10
)

for col, metodo in enumerate(metodos):
    i_sim, w_sim = resultados[metodo]

    # --- Corriente ---
    ax = axes[0, col]
    ax.plot(t_sim, i_raw_interp, color='steelblue', lw=1,   alpha=0.35, label='Raw')
    ax.plot(t_sim, i_sg_interp,  color='steelblue', lw=2,   alpha=0.9,  label='Filtrado')
    ax.plot(t_sim, i_sim,        color='tomato',    lw=1.5, ls='--',    label=metodo)
    ax.set_title(metodo)
    ax.set_ylabel("Corriente (A)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.4)

    # --- Velocidad ---
    ax = axes[1, col]
    ax.plot(t_sim, w_raw_interp, color='seagreen', lw=1,   alpha=0.35, label='Raw')
    ax.plot(t_sim, w_sg_interp,  color='seagreen', lw=2,   alpha=0.9,  label='Filtrado')
    ax.plot(t_sim, w_sim,        color='tomato',   lw=1.5, ls='--',    label=metodo)
    ax.set_ylabel("Velocidad (rad/s)")
    ax.set_xlabel("Tiempo (s)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.4)

fig.tight_layout()
plt.show()