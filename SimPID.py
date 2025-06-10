import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.express as px

# Configuración de la página
st.set_page_config(
    page_title="Simulador Control PID",
    page_icon="🎛️",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Título principal
st.title("🎛️ Simulador de Control PID Avanzado")
st.markdown("**Asignatura: Regulación Automática** | *Ingeniería*")
st.markdown("---")

# Funciones para diferentes sistemas
def first_order_model(u, k, tau, dt=0.01):
    """Modelo de primer orden: G(s) = K/(τs + 1)"""
    y = np.zeros_like(u)
    y_prev = 0
    
    for i in range(len(u)):
        # Ecuación diferencial: τ*dy/dt + y = K*u
        # Discretización: y[n] = y[n-1] + (dt/τ)*(K*u[n] - y[n-1])
        y[i] = y_prev + (dt/tau) * (k * u[i] - y_prev)
        y_prev = y[i]
    
    return y

def second_order_model(u, k, wn, zeta, dt=0.01):
    """Modelo de segundo orden: G(s) = K*wn²/(s² + 2*ζ*wn*s + wn²)"""
    y = np.zeros_like(u)
    x1, x2 = 0, 0  # Estados iniciales
    
    for i in range(len(u)):
        # Ecuaciones de estado para forma canónica controlable
        # dx1/dt = x2
        # dx2/dt = -wn²*x1 - 2*ζ*wn*x2 + k*wn²*u
        dx1 = x2
        dx2 = -wn*wn*x1 - 2*zeta*wn*x2 + k*wn*wn*u[i]
        
        x1 += dx1 * dt
        x2 += dx2 * dt
        y[i] = x1
    
    return y

def integrator_model(u, k, dt=0.01):
    """Modelo integrador: G(s) = K/s"""
    y = np.zeros_like(u)
    integral = 0
    
    for i in range(len(u)):
        integral += k * u[i] * dt
        y[i] = integral
    
    return y

def pid_simulation(kp, ki, kd, system_type, system_params, t_sim=10, dt=0.01):
    """Simulación completa del sistema con PID"""
    t = np.arange(0, t_sim, dt)
    setpoint = np.ones_like(t)  # Escalón unitario
    
    # Simulación sin control (lazo abierto)
    u_open = setpoint
    if system_type == "Primer Orden":
        y_open = first_order_model(u_open, system_params['k'], system_params['tau'], dt)
    elif system_type == "Segundo Orden":
        y_open = second_order_model(u_open, system_params['k'], system_params['wn'], system_params['zeta'], dt)
    else:  # Integrador
        y_open = integrator_model(u_open, system_params['k'], dt)
    
    # Simulación con PID (lazo cerrado)
    y_closed = np.zeros_like(t)
    error = np.zeros_like(t)
    control_signal = np.zeros_like(t)
    
    # Variables del PID
    integral = 0
    prev_error = 0
    
    # Estados para cada tipo de sistema
    if system_type == "Primer Orden":
        y_plant = 0
    elif system_type == "Segundo Orden":
        x1, x2 = 0, 0
    else:  # Integrador
        integral_plant = 0
    
    for i in range(len(t)):
        # Error
        if i == 0:
            current_output = 0
        else:
            current_output = y_closed[i-1]
        
        error[i] = setpoint[i] - current_output
        
        # Controlador PID
        integral += error[i] * dt
        if i > 0:
            derivative = (error[i] - prev_error) / dt
        else:
            derivative = 0
        
        control_signal[i] = kp * error[i] + ki * integral + kd * derivative
        
        # Modelo de planta según el tipo
        if system_type == "Primer Orden":
            k, tau = system_params['k'], system_params['tau']
            y_plant = y_plant + (dt/tau) * (k * control_signal[i] - y_plant)
            y_closed[i] = y_plant
            
        elif system_type == "Segundo Orden":
            k, wn, zeta = system_params['k'], system_params['wn'], system_params['zeta']
            dx1 = x2
            dx2 = -wn*wn*x1 - 2*zeta*wn*x2 + k*wn*wn*control_signal[i]
            x1 += dx1 * dt
            x2 += dx2 * dt
            y_closed[i] = x1
            
        else:  # Integrador
            k = system_params['k']
            integral_plant += k * control_signal[i] * dt
            y_closed[i] = integral_plant
        
        prev_error = error[i]
    
    return t, setpoint, y_open, y_closed, error, control_signal

def calculate_metrics(t, y_closed, setpoint, system_type):
    """Calcular métricas de desempeño"""
    final_value = setpoint[-1]
    
    # Para integradores, usar valor promedio de los últimos puntos
    if system_type == "Integrador":
        if len(y_closed) > 100:
            final_value_actual = np.mean(y_closed[-50:])
        else:
            final_value_actual = y_closed[-1]
        steady_state_error = abs(final_value - final_value_actual)
    else:
        # Tiempo de establecimiento (2%)
        settling_band = 0.02 * final_value
        settling_time = None
        
        for i in range(len(y_closed)-1, max(0, len(y_closed)-200), -1):
            if abs(y_closed[i] - final_value) > settling_band:
                settling_time = t[i] if i < len(t)-1 else t[-1]
                break
        
        if settling_time is None:
            settling_time = 0
        
        # Error en estado estacionario
        steady_state_error = abs(final_value - np.mean(y_closed[-100:]))
    
    # Sobrepaso máximo
    max_value = np.max(y_closed)
    overshoot = max_value - final_value
    overshoot_percent = (overshoot / final_value) * 100 if final_value != 0 else 0
    
    # Tiempo de subida (10% a 90%)
    target_10 = 0.1 * final_value
    target_90 = 0.9 * final_value
    
    rise_start = np.where(y_closed >= target_10)[0]
    rise_end = np.where(y_closed >= target_90)[0]
    
    rise_time = 0
    if len(rise_start) > 0 and len(rise_end) > 0:
        rise_time = t[rise_end[0]] - t[rise_start[0]]
    
    metrics = {
        'overshoot_percent': overshoot_percent,
        'rise_time': rise_time,
        'steady_state_error': steady_state_error
    }
    
    if system_type != "Integrador":
        metrics['settling_time'] = settling_time
    
    return metrics

# Sidebar para controles
st.sidebar.header("🏭 Configuración del Sistema")

# Selección del tipo de sistema
system_type = st.sidebar.selectbox(
    "Tipo de Sistema",
    ["Primer Orden", "Segundo Orden", "Integrador"],
    help="Selecciona el tipo de función de transferencia"
)

# Parámetros del sistema según el tipo
st.sidebar.subheader("⚙️ Parámetros del Sistema")

system_params = {}

if system_type == "Primer Orden":
    st.sidebar.markdown("**G(s) = K/(τs + 1)**")
    system_params['k'] = st.sidebar.slider(
        "Ganancia estática (K)", 
        min_value=0.1, max_value=5.0, value=1.0, step=0.1,
        help="Ganancia en estado estacionario"
    )
    system_params['tau'] = st.sidebar.slider(
        "Constante de tiempo (τ)", 
        min_value=0.1, max_value=5.0, value=1.0, step=0.1,
        help="Determina la velocidad de respuesta"
    )
    
elif system_type == "Segundo Orden":
    st.sidebar.markdown("**G(s) = K·ωₙ²/(s² + 2ζωₙs + ωₙ²)**")
    system_params['k'] = st.sidebar.slider(
        "Ganancia estática (K)", 
        min_value=0.1, max_value=5.0, value=1.0, step=0.1,
        help="Ganancia en estado estacionario"
    )
    system_params['wn'] = st.sidebar.slider(
        "Frecuencia natural (ωₙ)", 
        min_value=0.1, max_value=5.0, value=1.0, step=0.1,
        help="Frecuencia natural no amortiguada"
    )
    system_params['zeta'] = st.sidebar.slider(
        "Factor de amortiguamiento (ζ)", 
        min_value=0.1, max_value=2.0, value=0.5, step=0.1,
        help="ζ < 1: subamortiguado, ζ = 1: críticamente amortiguado, ζ > 1: sobreamortiguado"
    )
    
else:  # Integrador
    st.sidebar.markdown("**G(s) = K/s**")
    system_params['k'] = st.sidebar.slider(
        "Ganancia (K)", 
        min_value=0.1, max_value=5.0, value=1.0, step=0.1,
        help="Ganancia del integrador"
    )

# Separador
st.sidebar.markdown("---")
st.sidebar.header("🎛️ Controlador PID")

# Controles del PID
kp = st.sidebar.slider(
    "Ganancia Proporcional (Kp)", 
    min_value=0.0, max_value=10.0, value=1.0, step=0.1,
    help="Controla la respuesta proporcional al error actual"
)

ki = st.sidebar.slider(
    "Ganancia Integral (Ki)", 
    min_value=0.0, max_value=5.0, value=0.1, step=0.01,
    help="Elimina el error en estado estacionario"
)

kd = st.sidebar.slider(
    "Ganancia Derivativa (Kd)", 
    min_value=0.0, max_value=2.0, value=0.05, step=0.01,
    help="Reduce el sobrepaso y mejora la estabilidad"
)

# Parámetros de simulación
st.sidebar.markdown("---")
st.sidebar.subheader("📊 Simulación")
t_sim = st.sidebar.slider("Tiempo de simulación (s)", 5, 20, 10)
show_individual = st.sidebar.checkbox("Mostrar análisis detallado", value=True)

# Botones
col_btn1, col_btn2 = st.sidebar.columns(2)
with col_btn1:
    if st.button("🔄 Reset PID"):
        kp, ki, kd = 1.0, 0.1, 0.05
        st.rerun()

with col_btn2:
    if st.button("⚙️ Reset Sistema"):
        st.rerun()

# Realizar simulación
t, setpoint, y_open, y_closed, error, control_signal = pid_simulation(
    kp, ki, kd, system_type, system_params, t_sim
)
metrics = calculate_metrics(t, y_closed, setpoint, system_type)

# Mostrar función de transferencia actual
st.subheader("📋 Sistema Actual")
col_sys1, col_sys2 = st.columns([1, 1])

with col_sys1:
    if system_type == "Primer Orden":
        st.latex(f"G(s) = \\frac{{{system_params['k']}}}{{{system_params['tau']}s + 1}}")
    elif system_type == "Segundo Orden":
        st.latex(f"G(s) = \\frac{{{system_params['k']} \\cdot {system_params['wn']:.1f}^2}}{{s^2 + {2*system_params['zeta']*system_params['wn']:.1f}s + {system_params['wn']:.1f}^2}}")
    else:
        st.latex(f"G(s) = \\frac{{{system_params['k']}}}{{s}}")

with col_sys2:
    st.latex(f"PID(s) = {kp:.1f} + \\frac{{{ki:.2f}}}{{s}} + {kd:.2f}s")

# Layout principal
col1, col2 = st.columns([2, 1])

with col1:
    st.subheader("📈 Respuesta del Sistema")
    
    # Gráfica principal
    fig_main = go.Figure()
    
    fig_main.add_trace(go.Scatter(
        x=t, y=setpoint, 
        mode='lines', 
        name='Referencia', 
        line=dict(color='red', dash='dash', width=3)
    ))
    
    fig_main.add_trace(go.Scatter(
        x=t, y=y_open, 
        mode='lines', 
        name='Sin Control', 
        line=dict(color='blue', width=2),
        opacity=0.7
    ))
    
    fig_main.add_trace(go.Scatter(
        x=t, y=y_closed, 
        mode='lines', 
        name='Con PID', 
        line=dict(color='green', width=3)
    ))
    
    fig_main.update_layout(
        title=f"Sistema {system_type}: Sin Control vs Con PID",
        xaxis_title="Tiempo (s)",
        yaxis_title="Salida",
        legend=dict(x=0.7, y=0.1),
        height=400,
        template="plotly_white"
    )
    
    st.plotly_chart(fig_main, use_container_width=True)

# Métricas
with col2:
    st.subheader("📊 Métricas de Desempeño")
    
    if system_type != "Integrador":
        st.metric(
            label="⏱️ Tiempo de Establecimiento", 
            value=f"{metrics['settling_time']:.2f} s",
            help="Tiempo para alcanzar ±2% del valor final"
        )
    
    st.metric(
        label="📈 Sobrepaso Máximo", 
        value=f"{metrics['overshoot_percent']:.1f}%",
        delta=f"{metrics['overshoot_percent']-10:.1f}%" if metrics['overshoot_percent'] > 10 else None,
        delta_color="inverse"
    )
    
    st.metric(
        label="⚡ Tiempo de Subida", 
        value=f"{metrics['rise_time']:.2f} s",
        help="Tiempo de 10% a 90% del valor final"
    )
    
    st.metric(
        label="🎯 Error Estado Estacionario", 
        value=f"{metrics['steady_state_error']:.4f}",
        delta=f"{metrics['steady_state_error']:.4f}" if metrics['steady_state_error'] > 0.01 else None,
        delta_color="inverse"
    )

# Análisis detallado
if show_individual:
    st.markdown("---")
    st.subheader("📊 Análisis Detallado")
    
    fig_detailed = make_subplots(
        rows=2, cols=2,
        subplot_titles=('Error del Sistema', 'Señal de Control', 
                       'Respuesta Completa', 'Comparación Directa'),
        specs=[[{"secondary_y": False}, {"secondary_y": False}],
               [{"secondary_y": False}, {"secondary_y": False}]]
    )
    
    # Error
    fig_detailed.add_trace(
        go.Scatter(x=t, y=error, mode='lines', name='Error', 
                  line=dict(color='red', width=2)),
        row=1, col=1
    )
    
    # Señal de control
    fig_detailed.add_trace(
        go.Scatter(x=t, y=control_signal, mode='lines', name='Control', 
                  line=dict(color='purple', width=2)),
        row=1, col=2
    )
    
    # Respuesta completa
    fig_detailed.add_trace(
        go.Scatter(x=t, y=setpoint, mode='lines', name='Ref', 
                  line=dict(color='red', dash='dash')),
        row=2, col=1
    )
    fig_detailed.add_trace(
        go.Scatter(x=t, y=y_closed, mode='lines', name='PID', 
                  line=dict(color='green', width=2)),
        row=2, col=1
    )
    
    # Comparación
    fig_detailed.add_trace(
        go.Scatter(x=t, y=y_open, mode='lines', name='Sin Control', 
                  line=dict(color='blue', width=2), opacity=0.7),
        row=2, col=2
    )
    fig_detailed.add_trace(
        go.Scatter(x=t, y=y_closed, mode='lines', name='Con PID', 
                  line=dict(color='green', width=2)),
        row=2, col=2
    )
    
    fig_detailed.update_layout(height=600, showlegend=False, template="plotly_white")
    fig_detailed.update_xaxes(title_text="Tiempo (s)")
    fig_detailed.update_yaxes(title_text="Amplitud")
    
    st.plotly_chart(fig_detailed, use_container_width=True)

# Sección educativa
st.markdown("---")
st.subheader("🎓 Guía Educativa")

# Información del sistema actual
col_info1, col_info2 = st.columns(2)

with col_info1:
    st.markdown("**🏭 Características del Sistema:**")
    if system_type == "Primer Orden":
        st.markdown(f"""
        - **Tipo**: Sistema de primer orden
        - **Polos**: s = -1/τ = -{1/system_params['tau']:.2f}
        - **Ganancia DC**: {system_params['k']}
        - **Tiempo constante**: {system_params['tau']} s
        - **Comportamiento**: Respuesta exponencial
        """)
    elif system_type == "Segundo Orden":
        zeta = system_params['zeta']
        if zeta < 1:
            behavior = "Subamortiguado (oscilatorio)"
        elif zeta == 1:
            behavior = "Críticamente amortiguado"
        else:
            behavior = "Sobreamortiguado"
        
        st.markdown(f"""
        - **Tipo**: Sistema de segundo orden
        - **ωₙ**: {system_params['wn']:.2f} rad/s
        - **ζ**: {system_params['zeta']:.2f}
        - **Ganancia DC**: {system_params['k']}
        - **Comportamiento**: {behavior}
        """)
    else:
        st.markdown(f"""
        - **Tipo**: Integrador puro
        - **Polo**: s = 0
        - **Ganancia**: {system_params['k']}
        - **Tipo**: Sistema Tipo 1
        - **Comportamiento**: Error nulo a escalón
        """)

with col_info2:
    st.markdown("**🎛️ Efectos del Controlador PID:**")
    st.markdown(f"""
    - **Kp = {kp:.2f}**: {"✓ Adecuado" if 0.5 <= kp <= 3 else "⚠️ Revisar"}
    - **Ki = {ki:.2f}**: {"✓ Adecuado" if ki > 0 else "⚠️ Sin acción integral"}
    - **Kd = {kd:.2f}**: {"✓ Presente" if kd > 0 else "⚠️ Sin acción derivativa"}
    
    **Recomendaciones:**
    - Ajustar Kp para velocidad de respuesta
    - Ajustar Ki para eliminar error estacionario
    - Ajustar Kd para reducir oscilaciones
    """)

# Pie de página
st.markdown("---")
st.markdown("""
<div style='text-align: center; color: #666;'>
    <p>🎛️ <strong>Simulador Avanzado de Control PID</strong> | Desarrollado para Ingeniería - Regulación Automática</p>
    <p><em>Experimente con diferentes sistemas y observe los efectos del control PID</em></p>
</div>
""", unsafe_allow_html=True)