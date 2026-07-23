# -*- coding: utf-8 -*-
"""
Controlador de Lazo Exterior: Regulador de Velocidad (PI)

Ganancias y límites extraídos del bloque "Brushless DC Motor Drive"
(pestaña Controller) de MATLAB/Simscape.
"""
from control.controller import Controlador


class ControladorVelocidad(Controlador):
    def __init__(self, Ts, Kp, Ki, Te_min, Te_max, P, lambda_m):
        """
        :param Ts: Tiempo de muestreo del lazo de velocidad [s]
        :param Kp: Ganancia proporcional (MATLAB: Proportional gain)
        :param Ki: Ganancia integral (MATLAB: Integral gain)
        :param Te_min: Límite inferior de torque de salida [N*m]
        :param Te_max: Límite superior de torque de salida [N*m]
        :param P: Número de pares de polos (para convertir Torque -> I_ref)
        :param lambda_m: Constante de flujo magnético [V*s]
        """
        super().__init__(Ts)
        self.Kp = float(Kp)
        self.Ki = float(Ki)
        self.Te_min = float(Te_min)
        self.Te_max = float(Te_max)

        # Constante de par para control trapezoidal de 6 pasos (2 fases
        # conduciendo simultáneamente en la zona plana del perfil):
        # Te = 2 * P * lambda_m * I  =>  I_ref = Te_ref / Kt
        self.Kt = 2.0 * float(P) * float(lambda_m)

        # Memoria del integrador
        self._integral = 0.0

    def calcular(self, omega_ref, omega_m):
        """
        Ejecuta un paso del PI de velocidad.
        Retorna I_ref: corriente de referencia para el lazo interno de corriente.
        """
        error = omega_ref - omega_m

        p_term = self.Kp * error
        integral_candidata = self._integral + self.Ki * error * self.Ts
        Te_candidato = p_term + integral_candidata

        # Anti-windup tipo "clamping": solo se integra si el resultado no
        # está saturado (o si integrar ayuda a salir de la saturación).
        if self.Te_min <= Te_candidato <= self.Te_max:
            self._integral = integral_candidata

        Te_ref = p_term + self._integral
        Te_ref = max(self.Te_min, min(self.Te_max, Te_ref))

        I_ref = Te_ref / self.Kt
        return I_ref
