# 🎛️ Simulador de Control PID Avanzado

Un simulador interactivo desarrollado en Streamlit para el estudio y análisis de sistemas de control PID, diseñado específicamente para la asignatura de **Regulación Automática** en Ingeniería.

## 📋 Descripción

Este simulador permite analizar el comportamiento de diferentes tipos de sistemas de control con y sin controlador PID, proporcionando una herramienta educativa completa para comprender los conceptos fundamentales del control automático.

### ✨ Características Principales

- **Múltiples tipos de sistemas**: Primer orden, segundo orden e integrador
- **Simulación en tiempo real** con parámetros ajustables
- **Análisis comparativo** entre sistemas con y sin control
- **Métricas de desempeño** calculadas automáticamente
- **Interfaz intuitiva** con controles interactivos
- **Visualizaciones avanzadas** con Plotly
- **Guía educativa integrada**

## 🚀 Instalación

### Prerrequisitos

Asegúrate de tener Python 3.7 o superior instalado en tu sistema.

### Instalación de dependencias

```bash
pip install streamlit numpy matplotlib plotly
```

O usando el archivo de requerimientos:

```bash
pip install -r requirements.txt
```

## 🏃‍♂️ Ejecución

Para ejecutar el simulador:

```bash
streamlit run simulador_pid.py
```

El simulador se abrirá automáticamente en tu navegador web en la dirección `http://localhost:8501`.

## 🎯 Funcionalidades

### Tipos de Sistemas Disponibles

#### 1. **Sistema de Primer Orden**
- **Función de transferencia**: G(s) = K/(τs + 1)
- **Parámetros ajustables**:
  - K: Ganancia estática (0.1 - 5.0)
  - τ: Constante de tiempo (0.1 - 5.0)

#### 2. **Sistema de Segundo Orden**
- **Función de transferencia**: G(s) = K·ωₙ²/(s² + 2ζωₙs + ωₙ²)
- **Parámetros ajustables**:
  - K: Ganancia estática (0.1 - 5.0)
  - ωₙ: Frecuencia natural (0.1 - 5.0)
  - ζ: Factor de amortiguamiento (0.1 - 2.0)

#### 3. **Sistema Integrador**
- **Función de transferencia**: G(s) = K/s
- **Parámetros ajustables**:
  - K: Ganancia (0.1 - 5.0)

### Controlador PID

El controlador implementa la ecuación clásica:

**PID(s) = Kp + Ki/s + Kd·s**

- **Kp**: Ganancia proporcional (0.0 - 10.0)
- **Ki**: Ganancia integral (0.0 - 5.0)
- **Kd**: Ganancia derivativa (0.0 - 2.0)

### Métricas de Desempeño

El simulador calcula automáticamente:

- ⏱️ **Tiempo de establecimiento** (±2% del valor final)
- 📈 **Sobrepaso máximo** (en porcentaje)
- ⚡ **Tiempo de subida** (10% a 90% del valor final)
- 🎯 **Error en estado estacionario**

## 📊 Visualizaciones

### Gráfica Principal
- Comparación entre respuesta sin control y con PID
- Señal de referencia (escalón unitario)
- Respuesta temporal del sistema

### Análisis Detallado
- **Error del sistema** a lo largo del tiempo
- **Señal de control** generada por el PID
- **Respuesta completa** con referencia
- **Comparación directa** de ambas respuestas

## 🎓 Uso Educativo

### Objetivos de Aprendizaje

1. **Comprensión de sistemas dinámicos**:
   - Comportamiento de sistemas de primer y segundo orden
   - Efecto de los parámetros en la respuesta temporal

2. **Diseño de controladores PID**:
   - Efecto de cada ganancia (Kp, Ki, Kd)
   - Compromiso entre velocidad y estabilidad

3. **Análisis de desempeño**:
   - Interpretación de métricas de calidad
   - Optimización de parámetros

### Casos de Estudio Recomendados

#### Experimento 1: Efecto de Kp
1. Establecer Ki = 0, Kd = 0
2. Variar Kp de 0.1 a 5.0
3. Observar el efecto en velocidad de respuesta y estabilidad

#### Experimento 2: Acción Integral
1. Usar Kp óptimo del experimento anterior
2. Incrementar Ki gradualmente
3. Observar la eliminación del error estacionario

#### Experimento 3: Acción Derivativa
1. Con Kp y Ki ajustados
2. Añadir acción derivativa (Kd)
3. Analizar la reducción del sobrepaso

## 🔧 Personalización

### Modificación de Parámetros

Para cambiar los rangos de los parámetros, edita las líneas correspondientes en el código:

```python
kp = st.sidebar.slider(
    "Ganancia Proporcional (Kp)", 
    min_value=0.0, max_value=10.0, value=1.0, step=0.1
)
```

### Adición de Nuevos Sistemas

Para añadir un nuevo tipo de sistema:

1. Crear la función del modelo matemático
2. Añadir la opción en el selectbox
3. Implementar los parámetros en el sidebar
4. Integrar en la función `pid_simulation()`

## 📚 Fundamentos Teóricos

### Sistema de Primer Orden

La ecuación diferencial que gobierna el sistema es:
```
τ · dy/dt + y = K · u
```

### Sistema de Segundo Orden

Las ecuaciones de estado son:
```
dx₁/dt = x₂
dx₂/dt = -ωₙ² · x₁ - 2ζωₙ · x₂ + Kωₙ² · u
```

### Controlador PID

La ecuación del controlador en el dominio del tiempo:
```
u(t) = Kp · e(t) + Ki · ∫e(t)dt + Kd · de(t)/dt
```

## 🐛 Solución de Problemas

### Problemas Comunes

1. **El simulador no inicia**:
   - Verificar la instalación de todas las dependencias
   - Comprobar la versión de Python (≥3.7)

2. **Gráficas no se muestran**:
   - Actualizar Plotly: `pip install --upgrade plotly`
   - Limpiar caché del navegador

3. **Respuesta inestable**:
   - Reducir las ganancias del PID
   - Verificar los parámetros del sistema

### Limitaciones Conocidas

- La simulación usa integración numérica simple (Euler)
- No incluye saturación del actuador
- No considera ruido en las mediciones

## 📄 Licencia

Este proyecto está desarrollado con fines educativos para la asignatura de Regulación Automática en Ingeniería.

## 👥 Autor

Pablo López Osorio - Profesor del Departamento de Ingeniería en Automática, Electrónica, Arquitectura y Redes de Computadores

## 📞 Soporte

Para reportar problemas o sugerir mejoras, utiliza el sistema de issues del repositorio.

---

**¡Experimenta y aprende sobre control PID de manera interactiva!** 🎛️

*Recuerda: La mejor manera de aprender control es experimentando con diferentes configuraciones y observando sus efectos.*
