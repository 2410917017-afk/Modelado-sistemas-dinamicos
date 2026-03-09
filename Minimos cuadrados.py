import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# ==========================================
# 1. DEFINE TU MODELO FÍSICO (LA ECUACIÓN)
# ==========================================
# Cambia esta función por la ecuación teórica de tu sistema.
# REGLA: El primer argumento SIEMPRE debe ser la variable independiente (ej. tiempo 't', posición 'x').
# Los argumentos siguientes son los parámetros físicos que la computadora debe "descubrir".

def modelo_fisico(t, A, tau, C):
    """
    Ejemplo de plantilla: Descarga de un capacitor en un circuito RC.
    Ecuación: V(t) = A * exp(-t / tau) + C
    - t: Tiempo (variable independiente)
    - A: Amplitud de la respuesta transitoria (diferencia entre V_inicial y V_final)
    - tau: Constante de tiempo del circuito (R*C) [segundos]
    - C: Voltaje asintótico (valor de equilibrio) [voltios]
    
    NOTA FÍSICA: En un RC puro descargándose, C ≈ 0. Si C ≠ 0, hay fuente externa.
    """
    return A * np.exp(-t / tau) + C

# ==========================================
# 2. INGRESA TUS DATOS EXPERIMENTALES
# ==========================================
# En la vida real, aquí cargarías un archivo con np.loadtxt('datos.csv') o pandas.
# Para que la plantilla funcione ahora mismo, simularemos datos de laboratorio con "ruido".

# Variable independiente (ej. cronómetro de 0 a 10 segundos)
t_exp = np.linspace(0, 10, 50) 

# Simulamos las mediciones reales asumiendo que el sistema físico tiene
# valores reales de A=5, tau=2, C=1, y le agregamos ruido aleatorio de los sensores.
y_exp = 5 * np.exp(-t_exp / 2) + 1 + np.random.normal(0, 0.2, len(t_exp)) 

# ==========================================
# 3. APLICA EL MÉTODO DE MÍNIMOS CUADRADOS
# ==========================================
# p0 son tus "suposiciones iniciales" de los parámetros. 
# Si no tienes idea, puedes borrar 'p0', pero darle una pista al algoritmo ayuda a que converja más rápido.
suposiciones_iniciales = [4.0, 1.0, 0.0]

# BOUNDS: Limita los parámetros a rangos físicamente realistas
# (min_A, max_A), (min_tau, max_tau), (min_C, max_C)
limites = (
    [0, 1e-3, -np.inf],        # Límites inferiores: A>0, tau>0, C sin límite
    [np.inf, np.inf, np.inf]   # Límites superiores: sin límites superiores
)

try:
    # curve_fit es el motor matemático. Devuelve dos cosas:
    # 1. parametros_optimos: Los valores descubiertos para A, tau y C.
    # 2. matriz_covarianza: Información estadística sobre qué tan "seguro" está el modelo de esos resultados.
    parametros_optimos, matriz_covarianza = curve_fit(
        f=modelo_fisico, 
        xdata=t_exp, 
        ydata=y_exp, 
        p0=suposiciones_iniciales,
        bounds=limites,
        maxfev=10000  # Máximo de iteraciones
    )
    
    A_est, tau_est, C_est = parametros_optimos
    
    # Cálculo de la incertidumbre (error estándar) de cada parámetro físico
    errores = np.sqrt(np.diag(matriz_covarianza))
    err_A, err_tau, err_C = errores
    
except RuntimeError as e:
    print(f"ERROR: El algoritmo no pudo converger: {e}")
    print("Intenta: 1) Mejorar las suposiciones iniciales (p0)")
    print("         2) Cambiar los límites (bounds)")
    print("         3) Verificar que tu modelo sea apropiado para los datos")
    exit()

# ==========================================
# 4. IMPRIME LOS RESULTADOS CON INCERTIDUMBRE
# ==========================================
print("--- Parámetros Físicos Extraídos ---")
print(f"Amplitud Transitoria (A) : {A_est:.3f} ± {err_A:.3f} V")
print(f"Constante Tiempo (tau)   : {tau_est:.3f} ± {err_tau:.3f} s")
print(f"Valor Asintótico (C)     : {C_est:.3f} ± {err_C:.3f} V")

# ==========================================
# 4B. MÉTRICAS DE BONDAD DEL AJUSTE
# ==========================================
# Calcula residuales y el coeficiente de determinación (R²)
y_modelo_puntos = modelo_fisico(t_exp, A_est, tau_est, C_est)
residuales = y_exp - y_modelo_puntos

# R² = 1 - (SS_res / SS_tot)
SS_res = np.sum(residuales**2)  # Suma de cuadrados de residuales
SS_tot = np.sum((y_exp - np.mean(y_exp))**2)  # Suma total de cuadrados
R_cuadrado = 1 - (SS_res / SS_tot)

# Chi-cuadrado reducido (χ²_red)
n_datos = len(y_exp)
n_params = 3
chi_cuadrado = SS_res / (n_datos - n_params)

print("\n--- Calidad del Ajuste ---")
print(f"Coeficiente R²           : {R_cuadrado:.4f} (ideal: ≈ 1.0)")
print(f"χ² reducido              : {chi_cuadrado:.4f} (ideal: ≈ 1.0)")
print(f"Error medio residual     : {np.mean(np.abs(residuales)):.4f}")

# ==========================================
# 5. VISUALIZA EL MODELO VS TUS DATOS
# ==========================================
# Generamos una línea suave usando la ecuación y los parámetros que el algoritmo descubrió
t_linea = np.linspace(min(t_exp), max(t_exp), 200)
y_modelo = modelo_fisico(t_linea, A_est, tau_est, C_est)


# Subplot 1: Ajuste del modelo
plt.scatter(t_exp, y_exp, color='black', marker='o', s=40, alpha=0.6, label='Mediciones (Sensores)')
plt.plot(t_linea, y_modelo, color='red', linewidth=2, label='Ajuste Mínimos Cuadrados')
plt.xlabel('Tiempo (s)')
plt.ylabel('Voltaje (V)')
plt.title(f'Extracción de Parámetros: Circuito RC (R² = {R_cuadrado:.4f})')
plt.legend()
plt.grid(True, linestyle='--', alpha=0.5)


plt.savefig('ajuste_minimos_cuadrados.png', dpi=150)
print("\n✓ Gráfico guardado como 'ajuste_minimos_cuadrados.png'")