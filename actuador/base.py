# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 16:47:13 2026

@author: lmintegral
"""

# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 16:50:30 2026

@author: lmintegral
"""

# -*- coding: utf-8 -*-
"""
Clase base para motores eléctricos
"""

class MotorBase:
    def __init__(self, R, L, J, B, P, Ts):
        """
        Parámetros físicos extraídos del bloque de MATLAB:
        :param R: Resistencia del estator por fase [Ohms]
        :param L: Inductancia del estator por fase [Henrios]
        :param J: Inercia del rotor [kg*m^2]
        :param B: Coeficiente de fricción viscosa [N*m*s]
        :param P: Número de PARES de polos (Pole pairs)
        :param Ts: Tiempo de muestreo para el solucionador de Euler [s]
        """
        self.R = float(R)
        self.L = float(L)
        self.J = float(J)
        self.B = float(B)
        self.P = int(P)
        self.Ts = float(Ts)