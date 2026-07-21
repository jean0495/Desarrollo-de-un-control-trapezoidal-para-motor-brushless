# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 17:05:36 2026

@author: lmintegral
"""


"""
Módulo de instrumentación: Sensores de Efecto Hall virtuales
"""
import math

class SensoresHall:
    def __init__(self):
        # En una simulación ideal, los sensores no necesitan parámetros de 
        # inicialización, pero dejamos el constructor listo por si en el 
        # futuro queremos agregarle "ruido" o "retrasos" de lectura.
        pass

    def leer_estados(self, theta_e):
        """
        Convierte la posición eléctrica pura en los 3 estados lógicos (0 o 1) 
        de los sensores Hall (Ha, Hb, Hc) desfasados 120 grados eléctricos.
        """
        # Normalizamos a un ciclo de 0 a 2*pi
        theta = theta_e % (2.0 * math.pi)
        
        # Sensor A: Activo durante la primera mitad del ciclo eléctrico
        Ha = 1 if 0 <= theta < math.pi else 0
        
        # Sensor B: Desfasado 120 grados (2*pi/3)
        theta_b = (theta - (2.0 * math.pi / 3.0)) % (2.0 * math.pi)
        Hb = 1 if 0 <= theta_b < math.pi else 0
        
        # Sensor C: Desfasado 240 grados (4*pi/3)
        theta_c = (theta - (4.0 * math.pi / 3.0)) % (2.0 * math.pi)
        Hc = 1 if 0 <= theta_c < math.pi else 0
        
        return Ha, Hb, Hc

    def obtener_sector(self, Ha, Hb, Hc):
        """
        Toma el código de 3 bits de los sensores (001 a 110) y determina 
        en cuál de los 6 sectores (60 grados cada uno) está el rotor.
        """
        # Convertimos los 3 bits separados en un solo número entero
        codigo = (Ha << 2) | (Hb << 1) | Hc
        
        # Diccionario de decodificación estándar para motores BLDC
        # Nota: Los códigos 000 (0) y 111 (7) son físicamente imposibles 
        # en sensores desfasados a 120° (indicarían un cable roto).
        mapeo_sectores = {
            0b101: 1,
            0b100: 2,
            0b110: 3,
            0b010: 4,
            0b011: 5,
            0b001: 6
        }
        
        # Retorna el sector (1 a 6). Si hay un error, por seguridad devuelve 1.
        return mapeo_sectores.get(codigo, 1)