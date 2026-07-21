# -*- coding: utf-8 -*-
"""
Clase Base para los controladores del sistema
"""

class Controlador:
    def __init__(self, Ts):
        self.Ts = float(Ts)
        
    def calcular(self):
        # Método que deberá ser sobreescrito por las clases hijas
        pass