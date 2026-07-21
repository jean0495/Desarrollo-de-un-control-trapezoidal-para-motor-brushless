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
from math.electrica import actualizar_corrientes, calcular_fem, perfil_trapezoidal
from math.mecanica import calcular_torque, actualizar_movimiento
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
        Ejecuta un paso de simulación del motor. Recibe los voltajes del 
        inversor y el par de carga aplicado, y actualiza los estados internos.
        """
        # 1. Obtener valores actuales del perfil trapezoidal
        fa = perfil_trapezoidal(self.theta_e)
        fb = perfil_trapezoidal(self.theta_e - 2.0 * math.pi / 3.0)
        fc = perfil_trapezoidal(self.theta_e - 4.0 * math.pi / 3.0)
        
        # 2. Calcular la FEM actual con la velocidad de la iteración anterior
        self.ea, self.eb, self.ec = calcular_fem(self.theta_e, self.omega_m, self.lambda_m, self.P)
        
        # 3. Calcular el Torque actual
        self.Te = calcular_torque(self.ia, self.ib, self.ic, fa, fb, fc, self.lambda_m, self.P)
        
        # 4. Actualizar corrientes (Dinámica Eléctrica)
        self.ia, self.ib, self.ic = actualizar_corrientes(
            self.ia, self.ib, self.ic, 
            Va, Vb, Vc, 
            self.ea, self.eb, self.ec, 
            self.R, self.L, self.Ts
        )
        
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
    
        