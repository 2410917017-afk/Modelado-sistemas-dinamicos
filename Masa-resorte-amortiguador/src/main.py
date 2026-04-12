import cv2 as cv
import numpy as np
import time
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.optimize import minimize

# -----------------------------------------------
# Parámetros nominales del modelo masa-resorte-amortiguador
# (se usan como punto de partida para la estimación)
# -----------------------------------------------
m = 0.1739      # masa (kg)
k = 750.0       # constante del resorte (N/m)
b = 2.5         # fricción (N·s/m)

# Condiciones iniciales 
x0_manual = 0.025      # desplazamiento inicial (m)
v0_manual = 0.0        # velocidad inicial (m/s)

t_fin = 5.0
num_puntos = 1000

contador = 0
offset = 0

# --------------------------------------------------
# Función genérica del sistema (acepta parámetros)
# --------------------------------------------------
def sistema_gen(t, y, m_p, k_p, b_p):
    dxdt = y[1]
    dvdt = -(b_p / m_p) * y[1] - (k_p / m_p) * y[0]
    return [dxdt, dvdt]

def simular(params, t_eval, t0, tf, ci):
    """Integra el sistema con parámetros dados; devuelve x(t) o None si falla."""
    m_p, k_p, b_p, x0_p, v0_p = params
    try:
        sol = solve_ivp(
            lambda t, y: sistema_gen(t, y, m_p, k_p, b_p),
            [t0, tf], [x0_p, v0_p],
            method='RK45', t_eval=t_eval, dense_output=False,
            rtol=1e-8, atol=1e-10
        )
        return sol.y[0] if sol.success else None
    except Exception:
        return None

# --------------------------------------------------
# Adquisición de vídeo
# --------------------------------------------------
y1, y2 = 100, 500
x1, x2 = 200, 400
mm_px  = 0.357142857
duration = 5

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Error: no se pudo abrir la cámara")
    exit()
time.sleep(2)

t_vec, x_vec = [], []
T0 = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    T   = time.time() - T0
    ROI = frame #frame[y1:y2, x1:x2]
    hsv = cv.cvtColor(ROI, cv.COLOR_BGR2HSV)

    mask1 = cv.inRange(hsv, np.array([0,   100, 70]),  np.array([10,  255, 255]))
    mask2 = cv.inRange(hsv, np.array([170, 100, 70]),  np.array([180, 255, 255]))
    maskR = mask1 + mask2

    contours, _ = cv.findContours(maskR, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if contours:
        contour = max(contours, key=cv.contourArea)
        x, y, w, h = cv.boundingRect(contour)
        pos_px = y + h/2  # Centro vertical del rectángulo (masa puntual)
        pos_mm = pos_px * mm_px

        if contador == 3:
            offset = pos_mm

        pos_mm = pos_mm - offset

        if contador> 3:
            t_vec.append(T)
            x_vec.append(pos_mm)

        #cv.rectangle(ROI, (x, y), (x + w, y + h), (0, 255, 0), 2)
        #cv.putText(ROI, f"x={pos_mm:.2f} mm", (10, 30),
        #           cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    contador += 1
    
    #cv.imshow('ROI', ROI)
    if (cv.waitKey(1) & 0xFF == ord('q')) or (T > duration):
        break 

cap.release()
cv.destroyAllWindows()

# --------------------------------------------------
# Análisis y estimación de parámetros
# --------------------------------------------------
if len(t_vec) < 10:
    print("No se detectaron suficientes datos.")
    exit()

t_vec = np.array(t_vec)
x_vec = np.array(x_vec)

# Convertir mm → m y setear offset en la primera posición detectada
x_exp = x_vec / 1000.0
x_offset = x_exp[0]  # Primera posición como referencia (cero)
x_exp = x_exp - x_offset
t_exp = t_vec

# --------------------------------------------------
# Detección automática del inicio del movimiento
# --------------------------------------------------
# Calcula la velocidad numérica (derivada)
velocidad_num = np.gradient(x_exp, t_exp)

# Umbral de movimiento: media + 3*std de los primeros puntos "estáticos"
umbral_velocidad = 3.0 * np.std(velocidad_num[:min(20, len(velocidad_num))])

# Busca el primer índice donde |velocidad| supera el umbral
idx_inicio = np.argmax(np.abs(velocidad_num) > umbral_velocidad)
if np.abs(velocidad_num[idx_inicio]) <= umbral_velocidad:
    # Si no hay movimiento detectado, usa los primeros datos
    idx_inicio = 0

t_comienzo = t_exp[idx_inicio]
print(f"\n✓ Movimiento detectado en t = {t_comienzo:.3f} s (índice {idx_inicio})")

# Grid temporal común para comparación (desde el movimiento detectado)
t_fin_real = min(t_exp[-1], t_fin)
t_common   = np.linspace(t_comienzo, t_fin_real, num_puntos)

# Interpolar datos experimentales en el grid común
x_exp_interp = np.interp(t_common, t_exp, x_exp)

# --------------------------------------------------
# Función de costo: SSE entre simulado y experimental
# --------------------------------------------------
def costo(params):
    m_p, k_p, b_p, x0_p, v0_p = params
    # Restricciones físicas (barrera de penalización)
    if m_p <= 0 or k_p <= 0 or b_p < 0:
        return 1e12
    x_sim = simular(params, t_common, t_comienzo, t_fin_real,
                    ci=[x0_p, v0_p])
    if x_sim is None:
        return 1e12
    return float(np.sum((x_sim - x_exp_interp) ** 2))

# Punto de partida: parámetros nominales + CI manuales
p0 = [m, k, b, x0_manual, v0_manual]

print("\nEstimando parámetros por mínimos cuadrados (Nelder-Mead)...")
resultado = minimize(
    costo, p0,
    method='Nelder-Mead',
    options={
        'xatol': 1e-9,
        'fatol': 1e-12,
        'maxiter': 20_000,
        'maxfev':  50_000,
        'adaptive': True,   # Nelder-Mead adaptativo (mejor para ≥4 params)
    }
)

m_est, k_est, b_est, x0_est, v0_est = resultado.x

# Métricas de calidad del ajuste
x_sim_est = simular(resultado.x, t_common, t_comienzo, t_fin_real,
                    ci=[x0_est, v0_est])
SSE  = np.sum((x_sim_est - x_exp_interp) ** 2)
RMSE = np.sqrt(SSE / len(x_exp_interp)) * 1000   # en mm
SS_tot = np.sum((x_exp_interp - np.mean(x_exp_interp)) ** 2)
R2   = 1 - SSE / SS_tot if SS_tot > 0 else np.nan

print("\n--- Parámetros estimados ---")
print(f"  m  = {m_est:.6f} kg   (nominal: {m:.4f})")
print(f"  k  = {k_est:.4f} N/m  (nominal: {k:.1f})")
print(f"  b  = {b_est:.6f} N·s/m (nominal: {b:.2f})")
print(f"  x0 = {x0_est*1000:.3f} mm")
print(f"  v0 = {v0_est*1000:.3f} mm/s")
print(f"\n  RMSE = {RMSE:.4f} mm   |   R² = {R2:.6f}")
print(f"  Convergencia: {resultado.message}")

# Frecuencia natural y razón de amortiguamiento estimados
wn_est  = np.sqrt(k_est / m_est)
zeta_est = b_est / (2 * np.sqrt(k_est * m_est))
wn_nom  = np.sqrt(k / m)
zeta_nom = b / (2 * np.sqrt(k * m))
print(f"\n  ωn estimada = {wn_est:.3f} rad/s  (nominal: {wn_nom:.3f})")
print(f"  ζ  estimada = {zeta_est:.5f}      (nominal: {zeta_nom:.5f})")

# --------------------------------------------------
# Simulaciones con parámetros ESTIMADOS y 4 métodos
# --------------------------------------------------
t_eval = np.linspace(t_comienzo, t_fin_real, num_puntos)
ci_est = [x0_est, v0_est]

def integrar_metodo(metodo):
    sol = solve_ivp(
        lambda t, y: sistema_gen(t, y, m_est, k_est, b_est),
        [t_comienzo, t_fin_real], ci_est,
        method=metodo, t_eval=t_eval,
        rtol=1e-8, atol=1e-10
    )
    if not sol.success:
        raise RuntimeError(f"Falla en {metodo}: {sol.message}")
    return sol

sol1 = integrar_metodo('RK45')    # ode45
sol2 = integrar_metodo('RK23')    # ode23
sol3 = integrar_metodo('BDF')     # ode23s
sol4 = integrar_metodo('Radau')   # ode15s

# --------------------------------------------------
# Figura: comparación de los 4 métodos
# --------------------------------------------------
fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True, sharey=True)
fig.suptitle(
    f"Masa-resorte-amortiguador — Raw / Simulación  |   "
    f"N={len(t_common)} muestras\n"
    f"Parámetros estimados — m={m_est:.4f} kg  k={k_est:.2f} N/m  "
    f"b={b_est:.4f} N·s/m\n"
    f"ωn={wn_est:.2f} rad/s  ζ={zeta_est:.4f}",#  RMSE={RMSE:.4f} mm  R²={R2:.5f}",
    fontweight='bold', fontsize=10
)

configs = [
    (axes[0, 0], sol1, 'RK45 (ode45)'),
    (axes[0, 1], sol2, 'RK23 (ode23)'),
    (axes[1, 0], sol3, 'BDF  (ode23s)'),
    (axes[1, 1], sol4, 'Radau (ode15s)'),
]

for ax, sol, titulo in configs:
    ax.plot(t_common, x_exp_interp * 1000, 'steelblue',
            linewidth=2, alpha=0.9, label='Raw')
    ax.plot(sol.t,    sol.y[0]       * 1000, color='tomato',
            linewidth=1.5, linestyle='--', label=titulo)
    ax.set_title(titulo, fontsize=10)
    ax.set_xlabel("Tiempo (s)")
    ax.set_ylabel("Posición (mm)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.4)

fig.tight_layout()
plt.show()