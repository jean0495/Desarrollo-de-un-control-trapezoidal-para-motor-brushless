# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 16:50:07 2026

@author: lmintegral
"""

"""
Módulo de cálculo mecánico para el Motor BLDC
"""

def calcular_torque(ia, ib, ic, fa, fb, fc, lambda_m, P):
    """
    Calcula el par electromagnético (Te) generado por el motor.
    fa, fb, fc son los valores (-1 a 1) del perfil trapezoidal en el instante actual.
    """
    Te = P * lambda_m * (fa * ia + fb * ib + fc * ic)
    return Te

def actualizar_movimiento(omega_m, theta_m, Te, T_carga, J, B, P, Ts):
    """
    Resuelve la ecuación de movimiento mecánico usando el método de Euler.
    Retorna la nueva velocidad, posición mecánica y posición eléctrica.
    """
    # 1. Calculamos la aceleración actual (d(omega)/dt)
    domega_dt = (Te - T_carga - B * omega_m) / J
    
    # 2. Euler para predecir la nueva velocidad y posición mecánica
    omega_nueva = omega_m + Ts * domega_dt
    theta_m_nuevo = theta_m + Ts * omega_m
    
    # 3. Calculamos la nueva posición eléctrica (se normaliza en el cálculo de FEM)
    theta_e_nuevo = P * theta_m_nuevo
    
    return omega_nueva, theta_m_nuevo, theta_e_nuevo