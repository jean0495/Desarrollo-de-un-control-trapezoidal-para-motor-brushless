# -*- coding: utf-8 -*-
"""
Script principal de simulación: Control trapezoidal (6 pasos) de un
motor BLDC, con lazo de velocidad (PI) y lazo de corriente (histéresis).
Incluye filtro pasa bajas en la referencia de velocidad para suavizar transitorios.
"""
import os
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from actuador.bldc import MotorBLDC
from control.corriente import ControladorCorriente
from control.velocidad import ControladorVelocidad
from electronica.inversor import Inversor

# ============================================================
# PARÁMETROS DEL MOTOR
# ============================================================
R = 0.2            # Resistencia de fase [Ohm]
L = 8.5e-3         # Inductancia de fase [H]
J = 0.089          # Inercia del rotor [kg*m^2]
B = 0.005          # Fricción viscosa [N*m*s]
P = 4              # Pares de polos
LAMBDA_M = 0.175   # Flujo magnético [V*s]

VDC = 24.0

# ============================================================
# PARÁMETROS DEL CONTROLADOR DE VELOCIDAD
# ============================================================
KP_VEL = 1.0
KI_VEL = 2.0
TE_MIN, TE_MAX = -17.8, 17.8   # Límites de torque [N*m]
TS_VEL = 7 * 20e-6              # 140 us

# ============================================================
# PARÁMETROS DEL CONTROLADOR DE CORRIENTE Y HARDWARE
# ============================================================
BANDA_HISTERESIS = 0.01   # A
TS_CURRENT = 20e-6        # s
DEAD_TIME = 1e-6          # s

# ============================================================
# PARÁMETROS DE SIMULACIÓN Y FILTRO
# ============================================================
DT_PLANT = 1e-6          # CORRECCIÓN 1: Paso a 1 us para resolver el Dead Time
TIEMPO_SIM = 10           # Duración total de la simulación [s]
T_CARGA = 0.0            # Par de carga aplicado [N*m]

TAU_FILTRO_REF = 0.05    # Constante de tiempo del Filtro Pasa Bajas [s] 

# Función para la referencia
def omega_ref_perfil(t):
    """Referencia de velocidad cruda (escalón de 0 a 5 rad/s en t=0.05s)."""
    return 0.0 if t < 0.05 else 5.0


def main():
    # Se instancia el objeto motor
    motor = MotorBLDC(R, L, J, B, P, DT_PLANT, LAMBDA_M)
    
    # CORRECCIÓN 2: Inversor muestreado a DT_PLANT (simula el hardware MCPWM)
    inversor = Inversor(dt_sim=DT_PLANT, dead_time=DEAD_TIME) 
    
    ctrl_corriente = ControladorCorriente(TS_CURRENT, BANDA_HISTERESIS)
    ctrl_velocidad = ControladorVelocidad(
        TS_VEL, KP_VEL, KI_VEL, TE_MIN, TE_MAX, P, LAMBDA_M
    )

    n_steps = int(TIEMPO_SIM / DT_PLANT)
    k_current = max(1, round(TS_CURRENT / DT_PLANT))
    k_speed = max(1, round(TS_VEL / DT_PLANT))

    # Factor alpha para la discretización del filtro pasa bajas
    alpha_lpf = 1.0 - np.exp(-DT_PLANT / TAU_FILTRO_REF)
    wref_filtrada = 0.0

    I_ref = 0.0
    Va = Vb = Vc = 0.0
    Ah = Al = Bh = Bl = Ch = Cl = 0

    # Comandos ideales del controlador
    cmd_A = cmd_B = cmd_C = 0

    # Preasignación de arreglos para historiales
    hist_t = np.zeros(n_steps)
    hist_omega = np.zeros(n_steps)
    hist_ia = np.zeros(n_steps)
    hist_ib = np.zeros(n_steps)
    hist_ic = np.zeros(n_steps)
    hist_ea = np.zeros(n_steps)
    hist_Te = np.zeros(n_steps)
    hist_wref = np.zeros(n_steps)
    hist_Iref = np.zeros(n_steps)
    hist_Ah = np.zeros(n_steps)
    hist_Al = np.zeros(n_steps)
    hist_Bh = np.zeros(n_steps)
    hist_Bl = np.zeros(n_steps)

    # CORRECCIÓN 3: Reestructuración del bucle multitasa
    for k in range(n_steps):
        t = k * DT_PLANT
        
        # --- Filtro Pasa Bajas de la referencia (Cada 1 us) ---
        wref_raw = omega_ref_perfil(t)
        wref_filtrada += alpha_lpf * (wref_raw - wref_filtrada)

        # 1. Lazo Lento: Control de Velocidad (Cada 140 us)
        if k % k_speed == 0:
            I_ref = ctrl_velocidad.calcular(wref_filtrada, motor.omega_m)

        # 2. Lazo Rápido: Control de Corriente e Histéresis (Cada 20 us)
        if k % k_current == 0:
            _, _, _, sector = motor.get_sensores_hall()
            Sah, Sal, Sbh, Sbl, Sch, Scl = ctrl_corriente.calcular(
                I_ref, motor.ia, motor.ib, motor.ic, sector
            )
            # Guardamos la intención de conmutación ideal
            cmd_A = 1 if Sah else 0
            cmd_B = 1 if Sbh else 0
            cmd_C = 1 if Sch else 0

        # 3. Hardware MCPWM del Inversor y Planta Física (SIEMPRE - Cada 1 us)
        Ah, Al, Bh, Bl, Ch, Cl = inversor.actualizar(cmd_A, cmd_B, cmd_C)
        Va, Vb, Vc = inversor.voltajes_de_fase(VDC)

        motor.actualizar(Va, Vb, Vc, T_CARGA)

        # Guardado de historiales para telemetría
        hist_t[k] = t
        hist_omega[k] = motor.omega_m
        hist_ia[k] = motor.ia
        hist_ib[k] = motor.ib
        hist_ic[k] = motor.ic
        hist_ea[k] = getattr(motor, 'ea', getattr(motor, 'e_a', LAMBDA_M * motor.omega_m))
        hist_Te[k] = motor.Te
        hist_wref[k] = wref_filtrada
        hist_Iref[k] = I_ref
        hist_Ah[k] = Ah
        hist_Al[k] = Al
        hist_Bh[k] = Bh
        hist_Bl[k] = Bl

    # ---------------- Gráficas Optimizadas ----------------
    plt.close('all')
    STEP = 10  # Submuestreo para agilizar dibujado

    fig, axs = plt.subplots(4, 1, figsize=(11, 11), layout='constrained')

    # 1. Velocidad
    axs[0].plot(hist_t[::STEP], hist_wref[::STEP], "--", label="omega_ref (Filtrada)")
    axs[0].plot(hist_t[::STEP], hist_omega[::STEP], label="omega_m")
    axs[0].set_ylabel("Velocidad [rad/s]")
    axs[0].set_xlabel("Tiempo [s]")
    axs[0].legend(loc="lower right")
    axs[0].grid(True)

    # 2. FEM e_a e i_a
    ax_fem = axs[1]
    ax_curr_a = ax_fem.twinx()

    l1 = ax_fem.plot(hist_t[::STEP], hist_ea[::STEP], color="tab:red", label="FEM Fase A (e_a)")
    l2 = ax_curr_a.plot(hist_t[::STEP], hist_ia[::STEP], color="tab:blue", alpha=0.7, label="Corriente i_a")
    
    ax_fem.set_xlim(0, 0.6)
    ax_fem.set_ylabel("FEM e_a [V]", color="tab:red")
    ax_curr_a.set_ylabel("Corriente i_a [A]", color="tab:blue")
    ax_fem.set_xlabel("Tiempo [s]")
    
    lines = l1 + l2
    labels = [l.get_label() for l in lines]
    ax_fem.legend(lines, labels, loc="upper right")
    ax_fem.grid(True)

    # 3. Corrientes de las 3 Fases
    axs[2].plot(hist_t[::STEP], hist_ia[::STEP], label="i_a")
    axs[2].plot(hist_t[::STEP], hist_ib[::STEP], label="i_b")
    axs[2].plot(hist_t[::STEP], hist_ic[::STEP], label="i_c")
    axs[2].plot(hist_t[::STEP], hist_Iref[::STEP], "k--", label="I_ref", alpha=0.5)
    axs[2].set_xlim(0, 0.6)
    axs[2].set_ylabel("Corrientes [A]")
    axs[2].set_xlabel("Tiempo [s]")
    axs[2].legend(loc="upper right")
    axs[2].grid(True)

    # 4. Señales de Compuerta Crudas (Microsegundos)
    ZOOM_START = 0.10
    ZOOM_DURACION = 30 * TS_CURRENT  # 600 µs
    zoom_mask = (hist_t >= ZOOM_START) & (hist_t < ZOOM_START + ZOOM_DURACION)

    axs[3].step(hist_t[zoom_mask], hist_Ah[zoom_mask], where="post", label="A_high")
    axs[3].step(hist_t[zoom_mask], hist_Al[zoom_mask], where="post", label="A_low")
    axs[3].step(hist_t[zoom_mask], hist_Bh[zoom_mask] + 2.2, where="post", label="B_high (+2.2 offset)")
    axs[3].step(hist_t[zoom_mask], hist_Bl[zoom_mask] + 2.2, where="post", label="B_low (+2.2 offset)")
    axs[3].set_ylabel("Compuertas (0/1)")
    axs[3].set_xlabel("Tiempo [s]")
    axs[3].set_title(f"Señales de compuerta crudas ({ZOOM_DURACION*1e6:.0f} µs)")
    axs[3].legend(loc="upper right", fontsize=8)
    axs[3].grid(True)

    # Guardar en alta resolución
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "outputs")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "simulacion_bldc.png")
    plt.savefig(out_path, dpi=200, bbox_inches='tight')
    
    print(f"Gráfica guardada en: {out_path}")
    print(f"Velocidad final: {hist_omega[-1]:.2f} rad/s (referencia: {hist_wref[-1]:.2f} rad/s)")

    # ---------------- Medidor por Clics ----------------
    puntos_t = []

    def medir_tiempo(event):
        if event.xdata is not None and event.button == 1:
            puntos_t.append(event.xdata)
            print(f"📍 Clic {len(puntos_t)}: t = {event.xdata:.7f} s ({event.xdata * 1e6:.1f} µs)")
            
            if len(puntos_t) == 2:
                t1, t2 = puntos_t[0], puntos_t[1]
                dt = abs(t2 - t1)
                frecuencia_khz = (1 / dt) / 1000 if dt > 0 else 0
                
                print("=" * 45)
                print("MEDICIÓN DE TIEMPO (Δt):")
                print(f"   • Δt = {dt * 1e6:.2f} µs  ({dt * 1e3:.4f} ms)")
                print(f"   • Frecuencia (1/Δt) = {frecuencia_khz:.2f} kHz")
                print("=" * 45 + "\n")
                
                puntos_t.clear()

    fig.canvas.mpl_connect('button_press_event', medir_tiempo)

    plt.show(block=False)


if __name__ == "__main__":
    main()