# 🔬 Modelado y Simulación de Sistemas Dinámicos

<div align="center">

![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?style=for-the-badge&logo=python&logoColor=white)
![NumPy](https://img.shields.io/badge/NumPy-013243?style=for-the-badge&logo=numpy&logoColor=white)
![SciPy](https://img.shields.io/badge/SciPy-8CAAE6?style=for-the-badge&logo=scipy&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
![Matplotlib](https://img.shields.io/badge/Matplotlib-11557C?style=for-the-badge&logo=python&logoColor=white)

*Prácticas de modelado matemático, estimación de parámetros y simulación numérica de sistemas dinámicos físicos.*

</div>

---

## 📋 Descripción General

Este repositorio reúne una colección de prácticas orientadas al análisis, modelado y simulación de sistemas dinámicos. Cada proyecto parte de un sistema físico real, aplica fundamentos matemáticos para obtener su modelo, y emplea métodos numéricos y de estimación para validar el comportamiento simulado frente a datos experimentales o mediciones reales.

Las prácticas cubren desde sistemas mecánicos clásicos hasta identificación de parámetros con datos reales, incorporando herramientas modernas como visión artificial para la captura de variables de estado.

---

## 📁 Estructura del Repositorio

```
📦 modelado-sistemas-dinamicos/
├── 📂 01_pendulo_simple/
│   ├── pendulo_modelo.py
│   ├── simulacion_numerica.py
│   ├── comparacion_metodos.py
│   └── README.md
│
├── 📂 02_motor_dc/
│   ├── estimacion_parametros.py
│   ├── regresion_minimos_cuadrados.py
│   ├── simulacion_motor.py
│   └── README.md
│
├── 📂 03_masa_resorte_amortiguador/
│   ├── captura_vision_artificial.py
│   ├── modelado_sistema.py
│   ├── estimacion_parametros.py
│   ├── simulacion_mra.py
│   └── README.md
│
├── requirements.txt
└── README.md
```

---

## 🧪 Prácticas

---

### 1. 🔵 Péndulo Simple — Modelado y Simulación Numérica

**Ubicación:** `01_pendulo_simple/`

#### Descripción

Modelado del péndulo simple como sistema dinámico no lineal a partir de las ecuaciones de movimiento derivadas de la mecánica Lagrangiana. Se analiza el comportamiento del sistema tanto en la aproximación lineal (ángulos pequeños) como en el modelo no lineal completo.

#### Objetivos
- Derivar las ecuaciones diferenciales del péndulo simple
- Implementar y comparar múltiples métodos numéricos de integración
- Analizar el error y la estabilidad de cada método
- Visualizar trayectorias en el espacio de fase

#### Modelo Matemático

El sistema se describe mediante la ecuación diferencial no lineal:

$$\ddot{\theta} + \frac{g}{L}\sin(\theta) = 0$$

Linealizada para ángulos pequeños ($\sin\theta \approx \theta$):

$$\ddot{\theta} + \frac{g}{L}\theta = 0$$

#### Técnicas y Métodos Aplicados

| Método | Tipo | Orden |
|---|---|---|
| Euler Explícito | Un paso | 1° orden |
| Euler Mejorado (Heun) | Un paso | 2° orden |
| Runge-Kutta 4 | Un paso | 4° orden |
| Adams-Bashforth | Multipaso | Variable |

- **Comparación de métodos numéricos:** análisis de error global, estabilidad y costo computacional en función del paso de integración $h$
- **Espacio de fase:** visualización de trayectorias $(θ, \dot{θ})$ para condiciones iniciales variadas
- **Análisis de conservación de energía** como indicador de calidad del integrador

---

### 2. ⚡ Motor DC — Estimación de Parámetros y Simulación

**Ubicación:** `02_motor_dc/`

#### Descripción

Identificación paramétrica de un motor de corriente continua a partir de datos experimentales de velocidad angular y corriente. Se ajusta el modelo eléctrico-mecánico del motor y se valida la simulación contra la respuesta medida.

#### Objetivos
- Formular el modelo matemático del motor DC (subsistema eléctrico y mecánico)
- Estimar los parámetros del sistema usando mínimos cuadrados y regresión polinomial
- Simular la respuesta del motor ante diferentes entradas
- Validar el modelo comparando simulación vs. datos experimentales

#### Modelo Matemático

**Subsistema eléctrico:**

$$L\frac{di}{dt} + Ri = V - K_e\omega$$

**Subsistema mecánico:**

$$J\frac{d\omega}{dt} + B\omega = K_t i$$

Donde los parámetros a identificar son: $R$, $L$, $K_e$, $K_t$, $J$, $B$.

#### Técnicas Aplicadas

- **Mínimos Cuadrados (LS):** estimación directa de parámetros a partir del sistema lineal sobredeterminado $A\theta = b$, resuelto como $\hat{\theta} = (A^TA)^{-1}A^Tb$
- **Regresión Polinomial:** ajuste de curvas características (par vs. corriente, velocidad vs. voltaje) para identificación en estado estacionario
- **Validación cruzada:** comparación del error de predicción entre el modelo identificado y datos no usados en el entrenamiento
- **Análisis de residuales:** evaluación de la calidad del ajuste

---

### 3. 🟢 Sistema Masa-Resorte-Amortiguador — Modelado con Visión Artificial

**Ubicación:** `03_masa_resorte_amortiguador/`

#### Descripción

Implementación completa del ciclo modelado → medición → estimación → simulación para un sistema masa-resorte-amortiguador físico. La posición de la masa se obtiene en tiempo real mediante **visión artificial** con OpenCV (detección de marcadores de color o ArUco), generando datos experimentales para la identificación paramétrica.

#### Objetivos
- Capturar la posición de la masa mediante procesamiento de video
- Estimar los parámetros $k$ (rigidez) y $c$ (amortiguamiento) a partir de datos medidos
- Simular el sistema con los parámetros identificados
- Validar el modelo comparando la simulación con el movimiento real

#### Modelo Matemático

$$m\ddot{x} + c\dot{x} + kx = f(t)$$

En forma de espacio de estados:

$$\dot{\mathbf{x}} = \begin{bmatrix} 0 & 1 \\ -k/m & -c/m \end{bmatrix}\mathbf{x} + \begin{bmatrix} 0 \\ 1/m \end{bmatrix}f(t)$$

#### Pipeline de Visión Artificial

```
Video/Cámara → Preprocesamiento → Detección de marcador → Extracción de posición
     ↓
Serie de tiempo x(t) → Estimación de velocidad/aceleración (diferenciación numérica)
     ↓
Identificación paramétrica → Modelo validado → Simulación
```

#### Técnicas Aplicadas

- **OpenCV:** umbralizado HSV, detección de contornos y centroide, o marcadores ArUco para tracking robusto
- **Diferenciación numérica:** estimación de $\dot{x}$ y $\ddot{x}$ a partir de la señal de posición (diferencias finitas + filtro Savitzky-Golay)
- **Mínimos Cuadrados:** estimación de $k$ y $c$ reformulando la ecuación diferencial como sistema lineal en los parámetros
- **Regresión polinomial:** ajuste de la envolvente de la respuesta amortiguada para estimación de la tasa de decaimiento $\zeta\omega_n$
- **Simulación y validación:** integración numérica con RK4 y comparación gráfica con los datos reales

---

## 🛠️ Tecnologías y Dependencias

```txt
numpy          # Álgebra lineal y operaciones numéricas
scipy          # Integración ODE, señales y optimización
matplotlib     # Visualización y gráficas
opencv-python  # Visión artificial (Práctica 3)
pandas         # Manejo de datos experimentales
```

Instalar dependencias:

```bash
pip install -r requirements.txt
```

---

## 🚀 Cómo Ejecutar

```bash
# Clonar el repositorio
git clone https://github.com/usuario/modelado-sistemas-dinamicos.git
cd modelado-sistemas-dinamicos

# Instalar dependencias
pip install -r requirements.txt

# Ejecutar práctica de péndulo
cd 01_pendulo_simple
python comparacion_metodos.py

# Ejecutar estimación del motor DC
cd ../02_motor_dc
python estimacion_parametros.py

# Ejecutar sistema masa-resorte con visión
cd ../03_masa_resorte_amortiguador
python captura_vision_artificial.py
```

---

## 📊 Resultados y Visualizaciones

Cada práctica genera automáticamente:

- 📈 Gráficas de respuesta temporal del sistema
- 🔄 Diagramas de espacio de fase
- 📉 Comparativas de error entre métodos
- 🎯 Gráficas de ajuste del modelo vs. datos experimentales
- 📋 Reporte de parámetros identificados con métricas de bondad de ajuste ($R^2$, RMSE)

---

## 📚 Fundamentos Teóricos

| Área | Temas |
|---|---|
| Modelado | Leyes de Newton, Lagrangiano, espacio de estados |
| Métodos Numéricos | Euler, Runge-Kutta, Adams-Bashforth, análisis de error |
| Identificación | Mínimos cuadrados, regresión polinomial, validación cruzada |
| Visión Artificial | Segmentación por color, detección de marcadores, tracking |
| Señales | Diferenciación numérica, filtrado, FFT |

---

## 📄 Licencia

Este repositorio es de uso académico y educativo. Consultar el archivo `LICENSE` para más detalles.

---

<div align="center">

*Modelado · Simulación · Identificación de Sistemas*

</div>
