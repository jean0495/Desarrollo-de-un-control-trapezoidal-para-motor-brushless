# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 17:03:53 2026

@author: lmintegral
"""

"""
Módulo de Electrónica de Potencia: Inversor Trifásico de 6 pulsos
"""

class Inversor:
    def __init__(self, Vdc=48.0):
        """
        :param Vdc: Voltaje del bus DC de entrada [Voltios]
        """
        self.Vdc = float(Vdc)
        
    def calcular_voltaje_brazo(self, Sh, Sl, i_fase, e_fase):
        """
        Calcula el voltaje de una pierna del inversor respecto a tierra (GND).
        
        :param Sh: Estado de la compuerta superior (1 = ON, 0 = OFF)
        :param Sl: Estado de la compuerta inferior (1 = ON, 0 = OFF)
        :param i_fase: Corriente actual de la fase (para los diodos)
        :param e_fase: Fuerza electromotriz actual (para fase flotante)
        """
        if Sh == 1 and Sl == 0:
            # Interruptor superior cerrado, inferior abierto
            return self.Vdc
            
        elif Sh == 0 and Sl == 1:
            # Interruptor inferior cerrado, superior abierto
            return 0.0
            
        else:
            # Ambos interruptores abiertos (Fase inactiva o Tiempo Muerto)
            # La inductancia del motor fuerza a la corriente a pasar por los diodos
            tolerancia_cero = 0.001 # Amperios
            
            if i_fase > tolerancia_cero:
                # La corriente sale hacia el motor: conduce el diodo inferior
                return 0.0
            elif i_fase < -tolerancia_cero:
                # La corriente regresa del motor: conduce el diodo superior
                return self.Vdc
            else:
                # Corriente cero (Fase verdaderamente flotante).
                # El voltaje medido en la terminal es la mitad del bus más su propia FEM
                return (self.Vdc / 2.0) + e_fase

    def obtener_voltajes_fase(self, pulsos, ia, ib, ic, ea, eb, ec):
        """
        Convierte los 6 pulsos de control en los 3 voltajes aplicados al motor.
        
        :param pulsos: Tupla o lista de 6 estados (Sah, Sal, Sbh, Sbl, Sch, Scl)
        """
        Sah, Sal, Sbh, Sbl, Sch, Scl = pulsos
        
        Va = self.calcular_voltaje_brazo(Sah, Sal, ia, ea)
        Vb = self.calcular_voltaje_brazo(Sbh, Sbl, ib, eb)
        Vc = self.calcular_voltaje_brazo(Sch, Scl, ic, ec)
        
        return Va, Vb, Vc