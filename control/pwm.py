# -*- coding: utf-8 -*-
"""
Controlador de Lazo Interior: PWM de Frecuencia Fija (PI + portadora)
"""
from control.controller import Controlador


class ControladorPWM(Controlador):
    def __init__(self, Ts, freq_pwm, Kp, Ki, Vdc, duty_min=0.0, duty_max=1.0):
        """
        Ts        : periodo al que se INVOCA calcular(). Debe ser igual (o muy
                    cercano) al paso de planta, para resolver bien la portadora
                    (idealmente Ts = DT_PLANT).
        freq_pwm  : frecuencia de conmutación fija deseada [Hz]
        Kp, Ki    : ganancias del PI, en V/A y V/(A*s) respectivamente
        Vdc       : bus DC usado para convertir el comando de voltaje del PI a duty
        duty_min/duty_max : límites de saturación del ciclo útil (anti-windup)
        """
        super().__init__(Ts)
        self.freq_pwm = float(freq_pwm)
        self.Ts_pwm = 1.0 / self.freq_pwm
        self.Kp = float(Kp)
        self.Ki = float(Ki)
        self.Vdc = float(Vdc)
        self.duty_min = float(duty_min)
        self.duty_max = float(duty_max)

        # Estados de los dos PI independientes (fase "+" y fase "-" del sector)
        self.integ_pos = 0.0
        self.integ_neg = 0.0

        # Acumulador de tiempo para la portadora tipo diente de sierra
        self.t_carrier = 0.0

        self.pulsos_actuales = [0, 0, 0, 0, 0, 0]

    def _pi(self, error, integ_estado):
        """PI (salida en voltios) con anti-windup por saturación del duty cycle."""
        p_term = self.Kp * error
        integ_tentativo = integ_estado + self.Ki * error * self.Ts
        u_tentativo = p_term + integ_tentativo
        duty_tentativo = u_tentativo / self.Vdc

        if duty_tentativo > self.duty_max:
            duty = self.duty_max
            integ_final = integ_estado          # se congela el integrador
        elif duty_tentativo < self.duty_min:
            duty = self.duty_min
            integ_final = integ_estado
        else:
            duty = duty_tentativo
            integ_final = integ_tentativo

        return duty, integ_final

    def _actualizar_portadora(self):
        """Diente de sierra normalizado en [0, 1), de periodo Ts_pwm."""
        self.t_carrier += self.Ts
        if self.t_carrier >= self.Ts_pwm:
            self.t_carrier -= self.Ts_pwm
        return self.t_carrier / self.Ts_pwm

    def calcular(self, I_ref, ia, ib, ic, sector):
        """
        Misma firma e interfaz que ControladorCorriente.calcular(), para
        poder alternar entre histéresis y PWM en el main sin cambiar nada
        más que la instancia del controlador.
        """
        Sah = Sal = Sbh = Sbl = Sch = Scl = 0
        carrier = self._actualizar_portadora()

        def chop(duty):
            return 1 if duty > carrier else 0

        if sector == 1:
            duty_pos, self.integ_pos = self._pi(I_ref - ia, self.integ_pos)
            duty_neg, self.integ_neg = self._pi(I_ref - (-ib), self.integ_neg)
            Sah = chop(duty_pos); Sal = 1 - Sah
            Sbl = chop(duty_neg); Sbh = 1 - Sbl

        elif sector == 2:
            duty_pos, self.integ_pos = self._pi(I_ref - ia, self.integ_pos)
            duty_neg, self.integ_neg = self._pi(I_ref - (-ic), self.integ_neg)
            Sah = chop(duty_pos); Sal = 1 - Sah
            Scl = chop(duty_neg); Sch = 1 - Scl

        elif sector == 3:
            duty_pos, self.integ_pos = self._pi(I_ref - ib, self.integ_pos)
            duty_neg, self.integ_neg = self._pi(I_ref - (-ic), self.integ_neg)
            Sbh = chop(duty_pos); Sbl = 1 - Sbh
            Scl = chop(duty_neg); Sch = 1 - Scl

        elif sector == 4:
            duty_pos, self.integ_pos = self._pi(I_ref - ib, self.integ_pos)
            duty_neg, self.integ_neg = self._pi(I_ref - (-ia), self.integ_neg)
            Sbh = chop(duty_pos); Sbl = 1 - Sbh
            Sal = chop(duty_neg); Sah = 1 - Sal

        elif sector == 5:
            duty_pos, self.integ_pos = self._pi(I_ref - ic, self.integ_pos)
            duty_neg, self.integ_neg = self._pi(I_ref - (-ia), self.integ_neg)
            Sch = chop(duty_pos); Scl = 1 - Sch
            Sal = chop(duty_neg); Sah = 1 - Sal

        elif sector == 6:
            duty_pos, self.integ_pos = self._pi(I_ref - ic, self.integ_pos)
            duty_neg, self.integ_neg = self._pi(I_ref - (-ib), self.integ_neg)
            Sch = chop(duty_pos); Scl = 1 - Sch
            Sbl = chop(duty_neg); Sbh = 1 - Sbl

        self.pulsos_actuales = [Sah, Sal, Sbh, Sbl, Sch, Scl]
        return tuple(self.pulsos_actuales)