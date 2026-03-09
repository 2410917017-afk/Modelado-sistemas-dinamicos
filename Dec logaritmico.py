"""
  - Decremento Logarítmico (δ)
  - Razón de Amortiguamiento (ζ)
  - Frecuencia natural (ωn)
  - Período de oscilación
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import warnings
warnings.filterwarnings('ignore')


def analizar_decremento_logaritmico(t, señal, distance=10, amplitude_minima=None, 
                                    nombre_sistema="Sistema"):
    """
    Calcula el decremento logarítmico y parámetros de amortiguamiento.
    
    Parámetros:
    -----------
    t : array
        Vector de tiempo
    señal : array
        Datos de la señal (misma longitud que t)
    distance : int (default=10)
        Distancia mínima entre picos en puntos de datos.
        Aumenta si hay ruido excesivo, disminuye si se pierden picos.
    amplitude_minima : float (default=None)
        Amplitud mínima para considerar un pico válido.
        Si None, se estima automáticamente.
    nombre_sistema : str
        Nombre descriptivo del sistema para los reportes
    
    Retorna:
    --------
    dict: Diccionario con todos los parámetros calculados y validaciones
    """
    

    if len(t) != len(señal):
        return {"error": "Vectores t y señal tienen longitudes diferentes"}
    
    if len(t) < 10:
        return {"error": "Necesitas al menos 10 puntos de datos"}
    
    # ──────────────────────────────────────────────────────────────────────
    # PASO 2: Detección de picos (máximos locales)
    # ──────────────────────────────────────────────────────────────────────
    indices_picos, propiedades_picos = find_peaks(señal, distance=distance)
    

    tiempos_picos = t[indices_picos]
    amplitudes_picos = señal[indices_picos]
    
    # ──────────────────────────────────────────────────────────────────────
    # PASO 3: Filtrar picos válidos (positivos y con amplitud mínima)
    # ──────────────────────────────────────────────────────────────────────
    if amplitude_minima is None:
        amplitude_minima = np.max(np.abs(amplitudes_picos)) * 0.01  # 1% del máximo
    
    mascara_validos = np.abs(amplitudes_picos) > amplitude_minima
    tiempos_picos = tiempos_picos[mascara_validos]
    amplitudes_picos = amplitudes_picos[mascara_validos]
    
    n_picos_validos = len(amplitudes_picos)
    
    if n_picos_validos < 2:
        return {"error": f"Insuficientes picos válidos tras filtrado (encontrados: {n_picos_validos})"}
    
    # ──────────────────────────────────────────────────────────────────────
    # PASO 4: Cálculo de decrementos logarítmicos entre picos sucesivos
    # ──────────────────────────────────────────────────────────────────────
    decrementos = []
    indices_decrementos_validos = []
    
    for i in range(len(amplitudes_picos) - 1):
        amp_actual = np.abs(amplitudes_picos[i])
        amp_siguiente = np.abs(amplitudes_picos[i + 1])
        
        if amp_siguiente > 0:  # Evitar división por cero
            d = np.log(amp_actual / amp_siguiente)
            
            # Validar que sea un número físico válido
            if not np.isnan(d) and not np.isinf(d) and d > 0:
                decrementos.append(d)
                indices_decrementos_validos.append(i)
    
    if len(decrementos) == 0:
        return {"error": "No se pudieron calcular decrementos logarítmicos válidos"}
    
    # ──────────────────────────────────────────────────────────────────────
    # PASO 5: Cálculo del decremento promedio y razón de amortiguamiento
    # ──────────────────────────────────────────────────────────────────────
    delta_promedio = np.mean(decrementos)
    delta_std = np.std(decrementos)
    
    # Razón de amortiguamiento: ζ = δ / √(4π² + δ²)
    denominador = np.sqrt(4 * np.pi**2 + delta_promedio**2)
    zeta = delta_promedio / denominador
    
    # Frecuencia natural amortiguada: ωd = 1 / Δt_picos
    deltas_tiempo = np.diff(tiempos_picos)
    delta_t_promedio = np.mean(deltas_tiempo)
    frecuencia_amortiguada = 2 * np.pi / (2 * delta_t_promedio)  # rad/s
    periodo_amortiguado = 2 * delta_t_promedio
    
    # Frecuencia natural sin amortiguamiento: ωn = ωd / √(1 - ζ²)
    if zeta < 1:  # Sistema subamortiguado
        omega_n = frecuencia_amortiguada / np.sqrt(1 - zeta**2)
        frecuencia_natural = omega_n / (2 * np.pi)  # Hz
    else:
        omega_n = np.nan
        frecuencia_natural = np.nan
    

    # ──────────────────────────────────────────────────────────────────────
    # RETORNO: Compilar todos los resultados
    # ──────────────────────────────────────────────────────────────────────
    resultados = {
        "sistema": nombre_sistema,
        "exitoso": True,
        
        # Parámetros principales
        "delta": delta_promedio,
        "delta_std": delta_std,
        "zeta": zeta,
        "omega_n": omega_n,
        "frecuencia_natural_Hz": frecuencia_natural,
        
        # Parámetros secundarios
        "frecuencia_amortiguada": frecuencia_amortiguada,
        "periodo_amortiguado": periodo_amortiguado,
        
        # Estadísticas del análisis
        "n_picos_totales_detectados": len(indices_picos),
        "n_picos_validos": n_picos_validos,
        "n_decrementos_usados": len(decrementos),
        "amplitud_minima_filtro": amplitude_minima,
        
        # Datos para visualización
        "tiempos_picos": tiempos_picos,
        "amplitudes_picos": amplitudes_picos,
        "decrementos_individuales": decrementos,
        "indices_decrementos": indices_decrementos_validos
    }
    
    return resultados


# ════════════════════════════════════════════════════════════════════════════
# ═══════════════════  PARÁMETROS CONFIGURABLES  ═════════════════════════════
# ════════════════════════════════════════════════════════════════════════════

# 1. ORIGEN DE DATOS: Elige una opción
MODO = "archivo"  # Opciones: "simulado" | "archivo" | "personalizado"

if MODO == "simulado":
    # ──────────────────────────────────────────────────────────────────────
    # OPCIÓN A: Simular datos (para pruebas)
    # ──────────────────────────────────────────────────────────────────────
    print("Modo: Datos simulados")
    
    # PARÁMETROS SIMULADOS (ajusta estos valores)
    t_inicio = 0
    t_final = 15          # Duración total en segundos
    n_puntos = 1000       # Número de muestras
    
    frecuencia_natural = 2.0    # Hz (cambia esto)
    zeta_simulado = 0.12        # Razón de amortiguamiento [0-1] (cambia esto)
    amplitud = 1.0              # Amplitud inicial (cambia esto)
    ruido_nivel = 0.05          # Nivel de ruido gaussiano (0=sin ruido)
    
    # Generar tiempo y señal teórica
    t = np.linspace(t_inicio, t_final, n_puntos)
    omega_n = 2 * np.pi * frecuencia_natural
    omega_d = omega_n * np.sqrt(1 - zeta_simulado**2)
    
    señal_teorica = amplitud * np.exp(-zeta_simulado * omega_n * t) * np.cos(omega_d * t)
    señal_experimental = señal_teorica + np.random.normal(0, ruido_nivel, len(t))
    
    NOMBRE_ARCHIVO_SALIDA = "decremento_logaritmico_analisis"

elif MODO == "archivo":
    # ──────────────────────────────────────────────────────────────────────
    # OPCIÓN B: Cargar datos desde archivo CSV
    # ──────────────────────────────────────────────────────────────────────
    print("Modo: Cargar desde archivo")
    
    ARCHIVO_DATOS = "datos.csv"
    # El archivo debe tener formato: tiempo,amplitud (sin encabezado)
    try:
        datos = np.loadtxt(ARCHIVO_DATOS, delimiter=',')
        t = datos[:, 0]
        señal_experimental = datos[:, 1]
    except FileNotFoundError:
        print(f"Error: No se encontró {ARCHIVO_DATOS}")
        exit()
    
    NOMBRE_ARCHIVO_SALIDA = "decremento_logaritmico_analisis"

else:
    # ──────────────────────────────────────────────────────────────────────
    # OPCIÓN C: Personalizado (modifica aquí tu propio código)
    # ──────────────────────────────────────────────────────────────────────
    print("Modo: Personalizado")
    # Aquí insertas tu propia señal
    pass


# 2. PARÁMETROS DE DETECCIÓN DE PICOS
DISTANCIA_PICOS = 25          # Distancia mínima entre picos (aumenta si hay ruido)
AMPLITUD_MINIMA = None         # None = automático, o especifica un valor


# ════════════════════════════════════════════════════════════════════════════
# ═══════════════════════  EJECUCIÓN DEL ANÁLISIS  ════════════════════════════
# ════════════════════════════════════════════════════════════════════════════

print("\n" + "="*80)
print("ANÁLISIS DE DECREMENTO LOGARÍTMICO")
print("="*80)

# Ejecutar análisis
resultados = analizar_decremento_logaritmico(
    t=t,
    señal=señal_experimental,
    distance=DISTANCIA_PICOS,
    amplitude_minima=AMPLITUD_MINIMA,
    nombre_sistema="Sistema Experimental"
)

# Verificar si hubo errores
if "error" in resultados:
    print(f"\n❌ ERROR: {resultados['error']}")
    print("\nSugerencias:")
    print("  - Aumenta DISTANCIA_PICOS si hay ruido")
    print("  - Verifica que tu señal tenga la forma esperada")
    print("  - Comprueba que el archivo de datos sea correcto")
    exit()

# ════════════════════════════════════════════════════════════════════════════
# MOSTRAR RESULTADOS
# ════════════════════════════════════════════════════════════════════════════


print(f"Decremento Logarítmico (δ)          : {resultados['delta']:.6f} ± {resultados['delta_std']:.6f}")
print(f"Razón de Amortiguamiento (ζ)        : {resultados['zeta']:.6f} ({resultados['zeta']*100:.2f}%)")
print(f"  → Tipo: {'Subamortiguado' if resultados['zeta'] < 1 else 'Críticamente amortiguado' if abs(resultados['zeta']-1) < 0.01 else 'Sobreamortiguado'}")

print(f"Frecuencia Natural (ωn)             : {resultados['omega_n']:.4f} rad/s")
print(f"Frecuencia Natural (fn)             : {resultados['frecuencia_natural_Hz']:.4f} Hz")
print(f"Frecuencia Amortiguada (ωd)         : {resultados['frecuencia_amortiguada']:.4f} rad/s")
print(f"Período Amortiguado (Td)            : {resultados['periodo_amortiguado']:.4f} s")

# ════════════════════════════════════════════════════════════════════════════
# VISUALIZACIÓN
# ════════════════════════════════════════════════════════════════════════════


plt.plot(t, señal_experimental, 'b-', alpha=0.6, label='Señal experimental', linewidth=1.5)
plt.scatter(resultados['tiempos_picos'], resultados['amplitudes_picos'], 
           color='red', s=80, marker='o', zorder=5, label=f"Picos ({resultados['n_picos_validos']})")
plt.xlabel('Tiempo (s)')
plt.ylabel('Amplitud')
plt.title('Detección de Picos')
plt.grid(True, alpha=0.3)
plt.legend()

plt.savefig(f"{NOMBRE_ARCHIVO_SALIDA}.png", dpi=150, bbox_inches='tight')
print(f"\n✓ Gráfico guardado: {NOMBRE_ARCHIVO_SALIDA}.png")
print("="*80 + "\n")