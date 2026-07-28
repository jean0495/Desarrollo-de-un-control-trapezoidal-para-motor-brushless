# -*- coding: utf-8 -*-
"""
Script principal de simulación: Control trapezoidal (6 pasos) de un
motor BLDC, con lazo de velocidad (PI) y lazo de corriente conmutable
entre histéresis y PWM de frecuencia fija.
"""
import os
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from actuador.bldc import MotorBLDC
from control.corriente import ControladorCorriente
from control.pwm import ControladorPWM
from control.velocidad import ControladorVelocidad
from electronica.inversor import Inversor

# ============================================================
# SELECCIÓN DE CONTROLADOR DE CORRIENTE
# ============================================================
# Opciones: "HISTERESIS" o "PWM"
CONTROLADOR_CORRIENTE = "PWM"

# ============================================================
# PARÁMETROS DEL MOTOR
# ============================================================
R = 0.2            # Resistencia de fase [Ohm]
L = 8.5e-3         # Inductancia de fase [H]
J = 0.089          # Inercia del rotor [kg*m^2]
B = 0.005          # Fricción viscosa [N*m*s]
P = 4              # Pares de polos
LAMBDA_M = 0.175   # Flujo magnético [V*s]

VDC = 48.0

# ============================================================
# PARÁMETROS DEL CONTROLADOR DE VELOCIDAD
# ============================================================
KP_VEL = 1.0
KI_VEL = 2.0
TE_MIN, TE_MAX = -17.8, 17.8   # Límites de torque [N*m]
TS_VEL = 7 * 20e-6              # 140 us

# ============================================================
# PARÁMETROS DEL CONTROLADOR DE CORRIENTE (HISTÉRESIS)
# ============================================================
BANDA_HISTERESIS = 0.01   # A
TS_CURRENT = 20e-6        # s

# ============================================================
# PARÁMETROS DEL CONTROLADOR DE CORRIENTE (PWM FRECUENCIA FIJA)
# ============================================================
FREQ_PWM = 20e3            # Hz
T_PWM = 1.0 / FREQ_PWM
T_SIGMA = T_PWM / 2.0       # retardo promedio de conmutación
KP_PWM = L / (2 * T_SIGMA)  # V/A  -> ~170
KI_PWM = R / (2 * T_SIGMA)  # V/(A*s) -> ~4000

# ============================================================
# PARÁMETROS DE HARDWARE
# ============================================================
DEAD_TIME = 1e-6          # s

# ============================================================
# PARÁMETROS DE LA RAMPA Y CONDICIÓN INICIAL
# ============================================================
T_INICIO_RAMPA = 2.0      # Tiempo en que inicia la rampa [s]
T_DURACION_RAMPA = 1.0    # Tiempo que tarda en alcanzar la velocidad objetivo [s]
OMEGA_TARGET = 5.0
MODO_CONDICION_INICIAL = "DOS_FASES"

# ============================================================
# PARÁMETROS DE SIMULACIÓN Y FILTRO
# ============================================================
DT_PLANT = 1e-6          # Paso de integración a 1 us
TIEMPO_SIM = 10           # Duración total de la simulación [s]
T_CARGA = 1.0            # Par de carga aplicado [N*m]

TAU_FILTRO_REF = 0.05    # Constante de tiempo del Filtro Pasa Bajas [s]


def crear_controlador_corriente():
    """
    Instancia el controlador de corriente según CONTROLADOR_CORRIENTE.
    Ambas clases comparten la misma interfaz (.Ts y .calcular(...)),
    así que el resto del programa no necesita saber cuál está activo.
    """
    if CONTROLADOR_CORRIENTE == "HISTERESIS":
        return ControladorCorriente(TS_CURRENT, BANDA_HISTERESIS)
    elif CONTROLADOR_CORRIENTE == "PWM":
        return ControladorPWM(
            DT_PLANT, FREQ_PWM, KP_PWM, KI_PWM, VDC,
            duty_min=0.0, duty_max=1.0
        )
    else:
        raise ValueError(f"CONTROLADOR_CORRIENTE desconocido: {CONTROLADOR_CORRIENTE}")


def omega_ref_perfil(t):
    """
    Referencia de velocidad en rampa:
    - 0 a 2s: 0.0 rad/s
    - 2s a 3s: Rampa lineal hasta OMEGA_TARGET
    - > 3s: OMEGA_TARGET constante
    """
    if t < T_INICIO_RAMPA:
        return 0.0
    elif t < T_INICIO_RAMPA + T_DURACION_RAMPA:
        progreso = (t - T_INICIO_RAMPA) / T_DURACION_RAMPA
        return progreso * OMEGA_TARGET
    else:
        return OMEGA_TARGET


def main():
    motor = MotorBLDC(R, L, J, B, P, DT_PLANT, LAMBDA_M)
    inversor = Inversor(dt_sim=DT_PLANT, dead_time=DEAD_TIME)

    ctrl_corriente = crear_controlador_corriente()
    ctrl_velocidad = ControladorVelocidad(
        TS_VEL, KP_VEL, KI_VEL, TE_MIN, TE_MAX, P, LAMBDA_M
    )

    n_steps = int(TIEMPO_SIM / DT_PLANT)
    # k_current se deriva del Ts propio del controlador activo,
    # así que funciona igual sin importar cuál esté seleccionado
    k_current = max(1, round(ctrl_corriente.Ts / DT_PLANT))
    k_speed = max(1, round(TS_VEL / DT_PLANT))

    alpha_lpf = 1.0 - np.exp(-DT_PLANT / TAU_FILTRO_REF)
    wref_filtrada = 0.0

    I_ref = 0.0
    Va = Vb = Vc = 0.0
    Ah = Al = Bh = Bl = Ch = Cl = 0
    cmd_A = cmd_B = cmd_C = 0

    hist_t = np.zeros(n_steps)
    hist_omega = np.zeros(n_steps)
    hist_ia = np.zeros(n_steps)
    hist_ib = np.zeros(n_steps)
    hist_ic = np.zeros(n_steps)
    hist_ea = np.zeros(n_steps)
    hist_Te = np.zeros(n_steps)
    hist_wref = np.zeros(n_steps)
    hist_Iref = np.zeros(n_steps)
    hist_Ch = np.zeros(n_steps)
    hist_Cl = np.zeros(n_steps)
    hist_Bh = np.zeros(n_steps)
    hist_Bl = np.zeros(n_steps)

    for k in range(n_steps):
        t = k * DT_PLANT

        wref_raw = omega_ref_perfil(t)
        wref_filtrada += alpha_lpf * (wref_raw - wref_filtrada)

        if k % k_speed == 0:
            I_ref = ctrl_velocidad.calcular(wref_filtrada, motor.omega_m)

        if k % k_current == 0:  # Debo cambiar esto, poner otra lógica que me ayude a disminuir los picos de corriente
            if t < T_INICIO_RAMPA:
                if MODO_CONDICION_INICIAL == "DOS_FASES":
                    cmd_A = 0; cmd_B = 0; cmd_C = 0
                elif MODO_CONDICION_INICIAL == "UNA_FASE":
                    cmd_A = 1; cmd_B = 0; cmd_C = 0
                else:
                    cmd_A = cmd_B = cmd_C = 0
            else:
                _, _, _, sector = motor.get_sensores_hall()
                Sah, Sal, Sbh, Sbl, Sch, Scl = ctrl_corriente.calcular(
                    I_ref, motor.ia, motor.ib, motor.ic, sector
                )
                cmd_A = 1 if Sah else 0
                cmd_B = 1 if Sbh else 0
                cmd_C = 1 if Sch else 0

        Ah, Al, Bh, Bl, Ch, Cl = inversor.actualizar(cmd_A, cmd_B, cmd_C)
        Va, Vb, Vc = inversor.voltajes_de_fase(VDC)

        motor.actualizar(Va, Vb, Vc, T_CARGA)

        hist_t[k] = t
        hist_omega[k] = motor.omega_m
        hist_ia[k] = motor.ia
        hist_ib[k] = motor.ib
        hist_ic[k] = motor.ic
        hist_ea[k] = getattr(motor, 'ea', getattr(motor, 'e_a', LAMBDA_M * motor.omega_m))
        hist_Te[k] = motor.Te
        hist_wref[k] = wref_filtrada
        hist_Iref[k] = I_ref
        hist_Ch[k] = Ch
        hist_Cl[k] = Cl
        hist_Bh[k] = Bh
        hist_Bl[k] = Bl

    plt.close('all')
    STEP = 10

    fig, axs = plt.subplots(4, 1, figsize=(11, 11), layout='constrained')

    axs[0].plot(hist_t[::STEP], hist_wref[::STEP], "--", label="omega_ref (Filtrada)")
    axs[0].plot(hist_t[::STEP], hist_omega[::STEP], label="omega_m")
    axs[0].set_ylabel("Velocidad [rad/s]")
    axs[0].set_xlabel("Tiempo [s]")
    axs[0].legend(loc="lower right")
    axs[0].grid(True)

    ax_fem = axs[1]
    ax_curr_a = ax_fem.twinx()
    l1 = ax_fem.plot(hist_t[::STEP], hist_ea[::STEP], color="tab:red", label="FEM Fase A (e_a)")
    l2 = ax_curr_a.plot(hist_t[::STEP], hist_ia[::STEP], color="tab:blue", alpha=0.7, label="Corriente i_a")
    ax_fem.set_xlim(0, 3.5)
    ax_fem.set_ylabel("FEM e_a [V]", color="tab:red")
    ax_curr_a.set_ylabel("Corriente i_a [A]", color="tab:blue")
    ax_fem.set_xlabel("Tiempo [s]")
    lines = l1 + l2
    labels = [l.get_label() for l in lines]
    ax_fem.legend(lines, labels, loc="upper right")
    ax_fem.grid(True)

    axs[2].plot(hist_t[::STEP], hist_ia[::STEP], label="i_a")
    axs[2].plot(hist_t[::STEP], hist_ib[::STEP], label="i_b")
    axs[2].plot(hist_t[::STEP], hist_ic[::STEP], label="i_c")
    axs[2].plot(hist_t[::STEP], hist_Iref[::STEP], "k--", label="I_ref", alpha=0.5)
    axs[2].set_xlim(0, 3.5)
    axs[2].set_ylabel("Corrientes [A]")
    axs[2].set_xlabel("Tiempo [s]")
    axs[2].legend(loc="upper right")
    axs[2].grid(True)

    # ------------------------------------------------------------
    # Ventana de zoom para señales de compuerta:
    # - PWM: se dimensiona con el PERIODO REAL de conmutación (Ts_pwm),
    #   no con el Ts de invocación (que en PWM es DT_PLANT, la resolución
    #   de la portadora). Usar DT_PLANT aquí daría una ventana más corta
    #   que un solo ciclo de PWM y no se vería el patrón real.
    # - Histéresis: se sigue usando su Ts de evaluación, como antes.
    # ------------------------------------------------------------
    if CONTROLADOR_CORRIENTE == "PWM":
        periodo_conmutacion = ctrl_corriente.Ts_pwm
        N_CICLOS_ZOOM = 10   # cuántos periodos de PWM completos mostrar
    else:
        periodo_conmutacion = ctrl_corriente.Ts
        N_CICLOS_ZOOM = 30  # histéresis: mantiene el criterio original

    ZOOM_START = T_INICIO_RAMPA + 0.5
    ZOOM_DURACION = N_CICLOS_ZOOM * periodo_conmutacion
    zoom_mask = (hist_t >= ZOOM_START) & (hist_t < ZOOM_START + ZOOM_DURACION)

    axs[3].step(hist_t[zoom_mask], hist_Ch[zoom_mask], where="post", label="C_high")
    axs[3].step(hist_t[zoom_mask], hist_Cl[zoom_mask], where="post", label="C_low")
    axs[3].step(hist_t[zoom_mask], hist_Bh[zoom_mask] + 2.2, where="post", label="B_high (+2.2 offset)")
    axs[3].step(hist_t[zoom_mask], hist_Bl[zoom_mask] + 2.2, where="post", label="B_low (+2.2 offset)")
    axs[3].set_ylabel("Compuertas (0/1)")
    axs[3].set_xlabel("Tiempo [s]")
    axs[3].set_title(f"Señales de compuerta crudas ({ZOOM_DURACION*1e6:.0f} µs) — {CONTROLADOR_CORRIENTE}")
    axs[3].legend(loc="upper right", fontsize=8)
    axs[3].grid(True)

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "outputs")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"simulacion_bldc_{CONTROLADOR_CORRIENTE.lower()}.png")
    plt.savefig(out_path, dpi=200, bbox_inches='tight')

    print(f"Gráfica guardada en: {out_path}")
    print(f"Controlador de corriente: {CONTROLADOR_CORRIENTE}")
    print(f"Velocidad final: {hist_omega[-1]:.2f} rad/s (referencia: {hist_wref[-1]:.2f} rad/s)")

    # ---------------- Medidor por Clics ----------------
    # Haz clic en 2 puntos de la gráfica (por ejemplo, dos flancos
    # de subida consecutivos de A_high) para medir Δt directamente
    # y confirmar que el periodo/frecuencia real coincide con FREQ_PWM.
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