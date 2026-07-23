# -*- coding: utf-8 -*-
"""
Script principal de simulación: Control trapezoidal (6 pasos) de un
motor BLDC, con lazo de velocidad (PI) y lazo de corriente (histéresis).

Reemplaza al diagrama de Simulink; el puente de diodos rectificador NO se
modela porque en la implementación real se usa una batería directa.
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
# PARÁMETROS DEL MOTOR (extraídos del bloque "Brushless DC Motor
# Drive" -> pestaña "Permanent Magnet Synchronous Machine" de MATLAB)
# ============================================================
R = 0.2            # Resistencia de fase [Ohm]
L = 8.5e-3         # Inductancia de fase [H]
J = 0.089          # Inercia del rotor [kg*m^2]
B = 0.005          # Fricción viscosa [N*m*s]
P = 4              # Pares de polos
LAMBDA_M = 0.175   # Flujo magnético [V*s]

VDC = 24.0  # <-- AJUSTAR: no visible en las capturas (pestaña "Converters and DC bus")

# ============================================================
# PARÁMETROS DEL CONTROLADOR DE VELOCIDAD (pestaña "Controller")
# ============================================================
KP_VEL = 1.0
KI_VEL = 2.0
TE_MIN, TE_MAX = -17.8, 17.8   # Límites de torque [N*m]
TS_VEL = 7 * 20e-6              # 140 us

# ============================================================
# PARÁMETROS DEL CONTROLADOR DE CORRIENTE
# ============================================================
BANDA_HISTERESIS = 0.01   # A
TS_CURRENT = 20e-6        # s
DEAD_TIME = 1e-6          # s  <-- AJUSTAR si tienes el valor real

# ============================================================
# PARÁMETROS DE SIMULACIÓN
# ============================================================
DT_PLANT = 2e-6      # Paso de integración de la planta [s]
TIEMPO_SIM = 3
      # Duración total de la simulación [s]
T_CARGA = 0.0          # Par de carga aplicado [N*m]


def omega_ref_perfil(t):
    """Referencia de velocidad: escalón de 0 a 100 rad/s en t=0.05s."""
    return 0.0 if t < 0.05 else 5.0


def main():
    motor = MotorBLDC(R, L, J, B, P, DT_PLANT, LAMBDA_M)
    inversor = Inversor(dt_sim=TS_CURRENT, dead_time=DEAD_TIME)
    ctrl_corriente = ControladorCorriente(TS_CURRENT, BANDA_HISTERESIS)
    ctrl_velocidad = ControladorVelocidad(
        TS_VEL, KP_VEL, KI_VEL, TE_MIN, TE_MAX, P, LAMBDA_M
    )

    n_steps = int(TIEMPO_SIM / DT_PLANT)
    k_current = max(1, round(TS_CURRENT / DT_PLANT))
    k_speed = max(1, round(TS_VEL / DT_PLANT))

    I_ref = 0.0
    Va = Vb = Vc = 0.0
    Ah = Al = Bh = Bl = Ch = Cl = 0

    hist_t = np.zeros(n_steps)
    hist_omega = np.zeros(n_steps)
    hist_ia = np.zeros(n_steps)
    hist_ib = np.zeros(n_steps)
    hist_ic = np.zeros(n_steps)
    hist_Te = np.zeros(n_steps)
    hist_wref = np.zeros(n_steps)
    hist_Iref = np.zeros(n_steps)
    hist_Ah = np.zeros(n_steps)
    hist_Al = np.zeros(n_steps)
    hist_Bh = np.zeros(n_steps)
    hist_Bl = np.zeros(n_steps)

    for k in range(n_steps):
        t = k * DT_PLANT
        wref = omega_ref_perfil(t)

        if k % k_speed == 0:
            I_ref = ctrl_velocidad.calcular(wref, motor.omega_m)

        if k % k_current == 0:
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
        hist_Te[k] = motor.Te
        hist_wref[k] = wref
        hist_Iref[k] = I_ref
        hist_Ah[k] = Ah
        hist_Al[k] = Al
        hist_Bh[k] = Bh
        hist_Bl[k] = Bl

    # ---------------- Gráficas ----------------
    plt.close('all')
    fig, axs = plt.subplots(5, 1, figsize=(10, 12), sharex=False)

    axs[0].plot(hist_t, hist_wref, "--", label="omega_ref")
    axs[0].plot(hist_t, hist_omega, label="omega_m")
    axs[0].set_ylabel("Velocidad [rad/s]")
    axs[0].set_xlabel("Tiempo [s]")
    axs[0].legend(loc="lower right")
    axs[0].grid(True)

    axs[1].plot(hist_t, hist_ia, label="ia")
    axs[1].plot(hist_t, hist_ib, label="ib")
    axs[1].plot(hist_t, hist_ic, label="ic")
    axs[1].plot(hist_t, hist_Iref, "k--", label="I_ref", alpha=0.6)
    axs[1].set_ylabel("Corriente [A]")
    axs[1].set_xlabel("Tiempo [s]")
    axs[1].legend(loc="lower right")
    axs[1].grid(True)

    axs[2].plot(hist_t, hist_Te)
    axs[2].set_ylabel("Torque [N*m]")
    axs[2].set_xlabel("Tiempo [s]")
    axs[2].grid(True)

    # ---- Zoom real: pocas decenas de ciclos de TS_CURRENT ----
    ZOOM_START = 0.10
    ZOOM_DURACION = 30 * TS_CURRENT   # ~30 ciclos de conmutación -> pulsos distinguibles
    zoom_mask = (hist_t >= ZOOM_START) & (hist_t < ZOOM_START + ZOOM_DURACION)

    axs[3].step(hist_t[zoom_mask], hist_ia[zoom_mask], where="post", label="ia")
    axs[3].step(hist_t[zoom_mask], hist_ib[zoom_mask], where="post", label="ib")
    axs[3].set_ylabel("Corriente [A]")
    axs[3].set_xlabel("Tiempo [s]")
    axs[3].set_title(f"Zoom corriente ({ZOOM_DURACION*1e6:.0f} us) - ")
    axs[3].legend(loc="upper right")
    axs[3].grid(True)

    # ---- Señales de compuerta crudas: aquí SÍ se ve el dead-time real ----
    # (con Va/Vb no se distingue "low-side ON" de "dead-time", ambos dan 0V)
    axs[4].step(hist_t[zoom_mask], hist_Ah[zoom_mask], where="post", label="A_high")
    axs[4].step(hist_t[zoom_mask], hist_Al[zoom_mask], where="post", label="A_low")
    axs[4].step(hist_t[zoom_mask], hist_Bh[zoom_mask] + 2.2, where="post", label="B_high (+2.2 offset)")
    axs[4].step(hist_t[zoom_mask], hist_Bl[zoom_mask] + 2.2, where="post", label="B_low (+2.2 offset)")
    axs[4].set_ylabel("Compuertas (0/1)")
    axs[4].set_xlabel("Tiempo [s]")
    axs[4].set_title("Señales de compuerta crudas ")
    axs[4].legend(loc="upper right", fontsize=8)
    axs[4].grid(True)

    plt.tight_layout()

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "outputs")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "simulacion_bldc.png")
    plt.savefig(out_path, dpi=150)
    print(f"Gráfica guardada en: {out_path}")
    print(f"Velocidad final: {hist_omega[-1]:.2f} rad/s (referencia: {hist_wref[-1]:.2f} rad/s)")
    # ---------------- Medidor tipo Osciloscopio por clics ----------------
    puntos_t = []

    def medir_tiempo(event):
        if event.xdata is not None and event.button == 1:  # Clic izquierdo
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
                
                puntos_t.clear()  # Reiniciar para la siguiente medición

    # Conectar el evento de clic de ratón a la figura
    fig.canvas.mpl_connect('button_press_event', medir_tiempo)

    # Mostrar la gráfica
    plt.show(block=False)


if __name__ == "__main__":
    main()
