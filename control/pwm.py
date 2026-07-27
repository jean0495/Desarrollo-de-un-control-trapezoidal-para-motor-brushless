# -*- coding: utf-8 -*-
"""
Created on Mon Jul 27 08:48:53 2026

@author: lmintegral
"""

from control.controller import Controlador

class pwm_control(Controlador):
    def __init__(self, feq_pwm, pulse_width):
        super().__init__(Ts)
        self.frecuencia = float(feq_pwm)
        self.pulso = float(pulse_width)
    
    
    def aplicar_pwm():
        pass