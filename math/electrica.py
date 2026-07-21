# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 16:50:30 2026

@author: lmintegral
"""

"""
Módulo de cálculo eléctrico para el Motor BLDC
"""
import math

def calcular_voltaje_neutro(Va, Vb, Vc, ea, eb, ec):
    """
    Calcula el voltaje del punto neutro asumiendo un estator 
    conectado en estrella sin cable neutro.
    """
    Vn = (Va + Vb + Vc - ea - eb - ec) / 3.0
    return Vn

def actualizar_corrientes(ia, ib, ic, Va, Vb, Vc, ea, eb, ec, R, L, Ts):
    """
    Resuelve el circuito RL serie usando el método de Euler hacia adelante.
    Retorna las corrientes de fase en el instante [k+1].
    """
    # 1. Obtenemos el voltaje de neutro
    Vn = calcular_voltaje_neutro(Va, Vb, Vc, ea, eb, ec)
    
    # 2. Calculamos las derivadas actuales (di/dt)
    dia_dt = (Va - Vn - R * ia - ea) / L
    dib_dt = (Vb - Vn - R * ib - eb) / L
    dic_dt = (Vc - Vn - R * ic - ec) / L
    
    # 3. Aplicamos el método de Euler para predecir el siguiente estado
    ia_nueva = ia + Ts * dia_dt
    ib_nueva = ib + Ts * dib_dt
    ic_nueva = ic + Ts * dic_dt
    
    return ia_nueva, ib_nueva, ic_nueva 

def perfil_trapezoidal(theta_e):
    """
    Genera el valor normalizado (-1 a 1) de la forma de onda trapezoidal 
    para un ángulo eléctrico dado. Replicando la lógica interna de MATLAB.
    """
    # 1. Normalizamos el ángulo para que siempre esté en el ciclo de 0 a 2*pi
    theta = theta_e % (2.0 * math.pi)
    
    # 2. Función a trozos (Piecewise) del trapecio ideal con mesetas de 120°
    if 0 <= theta < (math.pi / 6.0):
        # Rampa de subida (0 a 30 grados)
        return (6.0 / math.pi) * theta
        
    elif (math.pi / 6.0) <= theta < (5.0 * math.pi / 6.0):
        # Meseta superior plana (30 a 150 grados)
        return 1.0
        
    elif (5.0 * math.pi / 6.0) <= theta < (7.0 * math.pi / 6.0):
        # Rampa de bajada (150 a 210 grados)
        return 1.0 - (6.0 / math.pi) * (theta - 5.0 * math.pi / 6.0)
        
    elif (7.0 * math.pi / 6.0) <= theta < (11.0 * math.pi / 6.0):
        # Meseta inferior plana (210 a 330 grados)
        return -1.0
        
    else:
        # Rampa de subida final para cerrar el ciclo (330 a 360 grados)
        return -1.0 + (6.0 / math.pi) * (theta - 11.0 * math.pi / 6.0)

def calcular_fem(theta_e, omega_m, lambda_m, P):
    """
    Calcula las fuerzas electromotrices (Back-EMF) de las tres fases (ea, eb, ec)
    basado en el perfil trapezoidal, la posición eléctrica y la velocidad del rotor.
    """
    # Calculamos la amplitud máxima del voltaje inducido
    e_peak = P * omega_m * lambda_m
    
    # Aplicamos el perfil a cada fase, desfasadas 120 grados eléctricos (2*pi/3)
    ea = e_peak * perfil_trapezoidal(theta_e)
    eb = e_peak * perfil_trapezoidal(theta_e - 2.0 * math.pi / 3.0)
    ec = e_peak * perfil_trapezoidal(theta_e - 4.0 * math.pi / 3.0)
    
    return ea, eb, ec