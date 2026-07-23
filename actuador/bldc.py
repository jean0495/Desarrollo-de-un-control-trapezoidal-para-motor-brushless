# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 16:43:06 2026

@author: lmintegral
"""

# -*- coding: utf-8 -*-
"""
Modelo de Planta: Motor BLDC Trapezoidal
"""
from actuador.base import MotorBase
from actuador.sensores import SensoresHall
import math
from modelos_matematicos.electrica import actualizar_corriente_2fases, calcular_fem, perfil_trapezoidal
from modelos_matematicos.mecanica import calcular_torque, actualizar_movimiento

# Mapeo de sector (1-6) -> (fase que conduce positivo, fase que conduce
# negativo, fase en circuito abierto). Debe coincidir exactamente con la
# tabla de conmutación de ControladorCorriente.
_MAPA_SECTOR = {
    1: ('A', 'B', 'C'),
    2: ('A', 'C', 'B'),
    3: ('B', 'C', 'A'),
    4: ('B', 'A', 'C'),
    5: ('C', 'A', 'B'),
    6: ('C', 'B', 'A'),
}
class MotorBLDC(MotorBase):
    def __init__(self, R, L, J, B, P, Ts, lambda_m):
        # Heredamos los parámetros físicos genéricos de la clase padre
        super().__init__(R, L, J, B, P, Ts)
        
        # Parámetro exclusivo del BLDC (Flujo magnético pico o Constante de voltaje Ke)
        # En MATLAB suele ser "Flux linkage established by magnets (V.s)"
        self.lambda_m = float(lambda_m) 
        
        #Correspondiente a los sensores
        self.sensores = SensoresHall()
        # ==========================================
        # VARIABLES DE ESTADO (Condiciones Iniciales)
        # ==========================================
        
        # 1. Estados Eléctricos (Corrientes de fase en Amperios)
        self.ia = 0.0
        self.ib = 0.0
        self.ic = 0.0
        
        # 2. Estados Mecánicos
        self.omega_m = 0.0   # Velocidad mecánica del rotor [rad/s]
        self.theta_m = 0.0   # Posición mecánica del rotor [rad]
        self.theta_e = 0.0   # Posición eléctrica [rad]
        
        # 3. Variables de cálculo instantáneo (No son estados, pero se actualizan)
        self.ea = 0.0        # Fuerza electromotriz (Back-EMF) fase A [V]
        self.eb = 0.0        # Fuerza electromotriz (Back-EMF) fase B [V]
        self.ec = 0.0        # Fuerza electromotriz (Back-EMF) fase C [V]
        
        self.Te = 0.0        # Torque electromagnético generado [N*m]
        
    def get_estados(self):
        """
        Retorna los sensores del motor para el controlador y graficación.
        Equivale al puerto de salida 'm' (Measurement) en Simulink.
        """
        return {
            'ia': self.ia,
            'ib': self.ib,
            'ic': self.ic,
            'omega_m': self.omega_m,
            'theta_e': self.theta_e,
            'Te': self.Te
        }
    
    def actualizar(self, Va, Vb, Vc, T_carga):
        """
        Ejecuta un paso de simulación del motor. Recibe los voltajes que el
        inversor aplica a las fases activas y el par de carga, y actualiza
        los estados internos.

        Nota de modelado: en control trapezoidal de 6 pasos solo 2 fases
        conducen a la vez; la tercera queda en circuito abierto real
        (corriente = 0), no atada a 0V. Por eso el sector (posición real
        del rotor, vía los sensores Hall virtuales) determina aquí qué
        fase es la flotante, independientemente del voltaje que el
        inversor le esté aplicando a ese terminal.
        """
        # 0. Determinar sector actual (posición real del rotor)
        _, _, _, sector = self.get_sensores_hall()
        fase_pos, fase_neg, fase_flotante = _MAPA_SECTOR[sector]

        # 1. Obtener valores actuales del perfil trapezoidal
        fa = perfil_trapezoidal(self.theta_e)
        fb = perfil_trapezoidal(self.theta_e - 2.0 * math.pi / 3.0)
        fc = perfil_trapezoidal(self.theta_e - 4.0 * math.pi / 3.0)

        # 2. Calcular la FEM actual con la velocidad de la iteración anterior
        self.ea, self.eb, self.ec = calcular_fem(self.theta_e, self.omega_m, self.lambda_m, self.P)

        # 3. Calcular el Torque actual (con las corrientes del paso anterior)
        self.Te = calcular_torque(self.ia, self.ib, self.ic, fa, fb, fc, self.lambda_m, self.P)

        # 4. Actualizar corrientes (Dinámica Eléctrica) - modelo de 2 fases
        V = {'A': Va, 'B': Vb, 'C': Vc}
        e = {'A': self.ea, 'B': self.eb, 'C': self.ec}
        i_actual = {'A': self.ia, 'B': self.ib, 'C': self.ic}

        I_nueva = actualizar_corriente_2fases(
            i_actual[fase_pos],
            V[fase_pos], V[fase_neg],
            e[fase_pos], e[fase_neg],
            self.R, self.L, self.Ts
        )

        nuevas = {fase_pos: I_nueva, fase_neg: -I_nueva, fase_flotante: 0.0}
        self.ia, self.ib, self.ic = nuevas['A'], nuevas['B'], nuevas['C']

        # 5. Actualizar velocidad y posición (Dinámica Mecánica)
        self.omega_m, self.theta_m, self.theta_e = actualizar_movimiento(
            self.omega_m, self.theta_m, self.Te, T_carga, 
            self.J, self.B, self.P, self.Ts
        )
        
    def get_sensores_hall(self):
        """Retorna el código Ha, Hb, Hc y el sector actual"""
        Ha, Hb, Hc = self.sensores.leer_estados(self.theta_e)
        sector = self.sensores.obtener_sector(Ha, Hb, Hc)
        return Ha, Hb, Hc, sector
    
        