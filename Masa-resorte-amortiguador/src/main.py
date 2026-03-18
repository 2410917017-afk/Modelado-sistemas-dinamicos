import cv2 as cv
import numpy as np
import time
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# --------------------------------------------------
# parámetros del modelo masa‑resorte‑amortiguador
# --------------------------------------------------
m = 0.1739      # masa (kg)
k = 75000       # constante del resorte (N/m)
b = 1.5  # fricción (N·s/m)

x0 = -0.01    # desplazamiento inicial (m)
v0 = 0.0        # velocidad inicial (m/s)

t_inicio = 0
t_fin = 5.0
num_puntos = 100

def sistema(t, y):
    """
    y[0] = x (posición)
    y[1] = v (velocidad)
    """
    dxdt = y[1]
    dvdt = -(b / m) * y[1] - (k / m) * y[0]
    return [dxdt, dvdt]

# cálculo del modelo (no depende de la cámara)
t_eval = np.linspace(t_inicio, t_fin, num_puntos)
sol = solve_ivp(sistema, [t_inicio, t_fin], [x0, v0], t_eval=t_eval)

if not sol.success:
    raise RuntimeError("Error en la integración numérica.")

# --------------------------------------------------
# procesamiento de vídeo
# --------------------------------------------------
y1, y2 = 100, 500    # ROI en píxeles
x1, x2 = 200, 400
mm_px = 0.357142857  # mm por píxel (calibración)
duration = 5         # segundos de adquisición

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Error: no se pudo abrir la cámara")
    exit()
time.sleep(2)

t_vec = []
x_vec = []
T0 = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    T = time.time() - T0
    ROI = frame[y1:y2, x1:x2]
    hsv = cv.cvtColor(ROI, cv.COLOR_BGR2HSV)

    Red_low1 = np.array([0, 100, 70])
    Red_High1 = np.array([10, 255, 255])
    Red_low2 = np.array([170, 100, 70])
    Red_High2 = np.array([180, 255, 255])

    mask1 = cv.inRange(hsv, Red_low1, Red_High1)
    mask2 = cv.inRange(hsv, Red_low2, Red_High2)
    maskR = mask1 + mask2

    contours, _ = cv.findContours(maskR, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if contours:
        contour = max(contours, key=cv.contourArea)
        x, y, w, h = cv.boundingRect(contour)
        pos_px = y + h
        pos_mm = pos_px * mm_px

        t_vec.append(T)
        x_vec.append(pos_mm)

        cv.rectangle(ROI, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv.putText(ROI, f"x={pos_mm:.2f} mm", (10, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    cv.imshow('ROI', ROI)
    if (cv.waitKey(1) & 0xFF == ord('q')) or (T > duration):
        break

cap.release()
cv.destroyAllWindows()

# --------------------------------------------------
# análisis de los datos recogidos
# --------------------------------------------------

if len(t_vec) >= 2:
    t_vec = np.array(t_vec)
    x_vec = np.array(x_vec)

    # primero convierto a metros y luego quito el offset
    x_exp = (x_vec / 1000.0)                      # mm → m
    x_exp = x_exp - np.mean(x_exp[:10])          # anular la deriva inicial
    t_exp = t_vec

    # velocidad inicial experimental
    v0 = (x_exp[1] - x_exp[0]) / (t_exp[1] - t_exp[0])
    y0 = [x_exp[0], v0]


    plt.figure()
    plt.plot(t_exp-1.3, x_exp, label="Experimental")   # Grafica los datos de la camara
    plt.plot(sol.t, sol.y[0], label="Modelo")      # Grafica datos del modelo
    plt.xlabel("Tiempo (s)")
    plt.ylabel("Posición (m)")
    plt.legend()
    plt.grid(True)
    plt.show()
else:
    print('No se detectaron suficientes datos')