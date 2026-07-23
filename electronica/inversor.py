import numpy as np

class Inversor:
    """
    Clase que representa un Inversor Trifásico de 6 pulsos con
    generación de tiempo muerto (Dead Time) por software.
    """
    def __init__(self, dt_sim: float, dead_time: float = 1e-6):
        """
        Parameters
        ----------
        dt_sim : float
            Paso de tiempo de la simulación o ciclo de ejecución (segundos).
        dead_time : float
            Tiempo muerto deseado entre conmutaciones (segundos).
        """
        self.dt_sim = dt_sim
        self.dead_time = dead_time
        
        # Cantidad de pasos de simulación que dura el tiempo muerto
        self.dead_time_ticks = int(np.ceil(dead_time / dt_sim)) if dt_sim > 0 else 0
        
        # Estado interno de las 3 ramas del inversor (A, B, C)
        self.fases = {
            'A': {'prev_cmd': 0, 'counter': self.dead_time_ticks, 'out_high': 0, 'out_low': 0},
            'B': {'prev_cmd': 0, 'counter': self.dead_time_ticks, 'out_high': 0, 'out_low': 0},
            'C': {'prev_cmd': 0, 'counter': self.dead_time_ticks, 'out_high': 0, 'out_low': 0},
        }

    def _procesar_rama(self, nombre_fase: str, cmd_deseado: int):
        """Procesa la lógica de retardo para una única rama."""
        fase = self.fases[nombre_fase]

        # 1. Detección de flanco (cambio de estado): Apagado inmediato de ambos MOSFETs
        if cmd_deseado != fase['prev_cmd']:
            fase['counter'] = 0
            fase['out_high'] = 0
            fase['out_low'] = 0
            fase['prev_cmd'] = cmd_deseado
            return

        # 2. Ventana de Tiempo Muerto activa: Mantener ambos apagados
        if fase['counter'] < self.dead_time_ticks:
            fase['counter'] += 1
            fase['out_high'] = 0
            fase['out_low'] = 0
        # 3. Retardo finalizado: Encendido seguro de la salida correspondiente
        else:
            if cmd_deseado == 1:
                fase['out_high'] = 1
                fase['out_low'] = 0
            else:
                fase['out_high'] = 0
                fase['out_low'] = 1

    def actualizar(self, cmd_A: int, cmd_B: int, cmd_C: int):
        """
        Procesa los 3 comandos de entrada y retorna las 6 salidas digitales.

        Returns
        -------
        tuple: (A_high, A_low, B_high, B_low, C_high, C_low)
        """
        self._procesar_rama('A', int(cmd_A))
        self._procesar_rama('B', int(cmd_B))
        self._procesar_rama('C', int(cmd_C))

        return (
            self.fases['A']['out_high'], self.fases['A']['out_low'],
            self.fases['B']['out_high'], self.fases['B']['out_low'],
            self.fases['C']['out_high'], self.fases['C']['out_low']
        )

    def voltajes_de_fase(self, Vdc: float):
        """
        Convierte el estado actual de las 6 compuertas en los voltajes de
        fase (referenciados al riel negativo del bus DC) que necesita el
        modelo de planta (MotorBLDC.actualizar).

        Simplificación asumida: si una rama no está siendo forzada por el
        controlador de corriente (ninguno de los dos MOSFETs de esa rama
        encendido, es decir la fase "no conmutada" del control trapezoidal),
        se trata como si el lado bajo estuviera activo (fase atada a 0V).
        Esta es la aproximación estándar usada en modelos simplificados de
        6 pasos; no reproduce el "floating" ideal de la fase inactiva, pero
        es suficiente para validar el comportamiento del controlador.
        """
        def _voltaje_rama(fase):
            if self.fases[fase]['out_high']:
                return float(Vdc)
            # out_low activo, o rama sin forzar (dead-time / no conmutada)
            return 0.0

        Va = _voltaje_rama('A')
        Vb = _voltaje_rama('B')
        Vc = _voltaje_rama('C')
        return Va, Vb, Vc