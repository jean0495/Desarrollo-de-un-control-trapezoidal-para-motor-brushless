# -*- coding: utf-8 -*-
"""
Created on Fri Jul 17 16:41:43 2026

@author: lmintegral
"""
# -*- coding: utf-8 -*-
"""
Controlador de Lazo Interior: Histéresis y Tabla de Conmutación
"""
from control.controller import Controlador

class ControladorCorriente(Controlador):
    def __init__(self, Ts, banda_histeresis):
        super().__init__(Ts)
        self.banda = float(banda_histeresis)
        
        # Estado anterior de los pulsos (memoria de la histéresis)
        # Formato: [Sah, Sal, Sbh, Sbl, Sch, Scl]
        self.pulsos_actuales = [0, 0, 0, 0, 0, 0]

    def _aplicar_histeresis(self, i_medida, i_referencia, estado_actual):
        """
        Lógica Bang-Bang. 
        Retorna 1 (encender) o 0 (apagar) manteniendo el estado si está dentro de la banda.
        """
        error = i_referencia - i_medida
        
        if error > self.banda:
            return 1  # La corriente está muy baja, encendemos
        elif error < -self.banda:
            return 0  # La corriente está muy alta, apagamos
        else:
            return estado_actual  # Dentro de la banda, mantenemos el estado anterior

    def calcular(self, I_ref, ia, ib, ic, sector):
        """
        Determina qué compuertas encender basado en el sector y la histéresis.
        Retorna la tupla de 6 pulsos: (Sah, Sal, Sbh, Sbl, Sch, Scl)
        """
        # Inicializamos todas las compuertas en OFF por defecto
        Sah = Sal = Sbh = Sbl = Sch = Scl = 0
        
        # Recuperamos el estado anterior para la lógica de histéresis
        Sah_old, Sal_old, Sbh_old, Sbl_old, Sch_old, Scl_old = self.pulsos_actuales
        
        # =======================================================
        # TABLA DE CONMUTACIÓN DE 6 PASOS Y CONTROL DE HISTÉRESIS
        # =======================================================
        
        if sector == 1:
            # Fase A positiva (+I_ref), Fase B negativa (-I_ref), Fase C apagada
            Sah = self._aplicar_histeresis(ia, I_ref, Sah_old)
            Sal = 1 if Sah == 0 else 0  # Complementario para el brazo A
            
            Sbl = self._aplicar_histeresis(-ib, I_ref, Sbl_old)
            Sbh = 1 if Sbl == 0 else 0  # Complementario para el brazo B
            
        elif sector == 2:
            # Fase A positiva, Fase C negativa, Fase B apagada
            Sah = self._aplicar_histeresis(ia, I_ref, Sah_old)
            Sal = 1 if Sah == 0 else 0
            
            Scl = self._aplicar_histeresis(-ic, I_ref, Scl_old)
            Sch = 1 if Scl == 0 else 0
            
        elif sector == 3:
            # Fase B positiva, Fase C negativa, Fase A apagada
            Sbh = self._aplicar_histeresis(ib, I_ref, Sbh_old)
            Sbl = 1 if Sbh == 0 else 0
            
            Scl = self._aplicar_histeresis(-ic, I_ref, Scl_old)
            Sch = 1 if Scl == 0 else 0
            
        elif sector == 4:
            # Fase B positiva, Fase A negativa, Fase C apagada
            Sbh = self._aplicar_histeresis(ib, I_ref, Sbh_old)
            Sbl = 1 if Sbh == 0 else 0
            
            Sal = self._aplicar_histeresis(-ia, I_ref, Sal_old)
            Sah = 1 if Sal == 0 else 0
            
        elif sector == 5:
            # Fase C positiva, Fase A negativa, Fase B apagada
            Sch = self._aplicar_histeresis(ic, I_ref, Sch_old)
            Scl = 1 if Sch == 0 else 0
            
            Sal = self._aplicar_histeresis(-ia, I_ref, Sal_old)
            Sah = 1 if Sal == 0 else 0
            
        elif sector == 6:
            # Fase C positiva, Fase B negativa, Fase A apagada
            Sch = self._aplicar_histeresis(ic, I_ref, Sch_old)
            Scl = 1 if Sch == 0 else 0
            
            Sbl = self._aplicar_histeresis(-ib, I_ref, Sbl_old)
            Sbh = 1 if Sbl == 0 else 0

        # Guardamos el estado para el siguiente ciclo
        self.pulsos_actuales = [Sah, Sal, Sbh, Sbl, Sch, Scl]
        
        return tuple(self.pulsos_actuales)