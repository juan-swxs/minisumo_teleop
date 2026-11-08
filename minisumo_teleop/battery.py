"""Estado de la alimentacion del sistema (INA219 en el carro).

Bateria: 2 x Beston BST-9V (Ni-MH recargable de 250 mAh) en paralelo.
Las "9 V" de Ni-MH son 7 celdas de 1.2 V (8.4 V nominal): ~9.8 V recien
cargada, ~7.0 V descargada (1.0 V por celda). En paralelo la tension es
la misma, la capacidad se suma (500 mAh) y la resistencia interna se
reduce a la mitad.

La tension de una Ni-MH es casi plana en la mayor parte de la descarga,
asi que el porcentaje se estima combinando dos fuentes:
  - Conteo de coulombs (mAh consumidos que integra el ESP32 con el
    INA219): preciso a corto plazo, pero acumula error y no sabe con que
    carga arranco la bateria.
  - Tension en circuito abierto estimada, V + I * R_interna (compensa la
    caida cuando los motores consumen): absoluta pero ruidosa.
El porcentaje avanza con los mAh consumidos y se corrige despacio hacia
el de la tension (mas rapido en reposo, cuando la tension es fiable).
"""

import math
import time
from collections import deque

# Tension de una celda Ni-MH en reposo vs carga restante (%), descarga a
# ~0.2C. Por debajo de 1.0 V la celda se considera vacia.
NIMH_CURVE = [
    (1.00, 0.0), (1.10, 4.0), (1.15, 9.0), (1.18, 18.0), (1.21, 32.0),
    (1.23, 45.0), (1.25, 58.0), (1.27, 72.0), (1.30, 86.0), (1.33, 94.0), (1.38, 100.0),
]

DEFAULTS = {
    'battery_cells': 7,             # celdas en serie (8.4 V nominal)
    'battery_packs': 2,             # baterias iguales en paralelo
    'battery_capacity_mah': 500.0,  # 2 x 250 mAh
    'battery_r_int_ohm': 0.7,       # una Ni-MH 9 V tiene ~1.4 ohm; dos en paralelo, la mitad
    'battery_low_pct': 20.0,
    'battery_critical_pct': 10.0,
}


def soc_from_cell_voltage(v_cell):
    pts = NIMH_CURVE
    if v_cell <= pts[0][0]:
        return 0.0
    if v_cell >= pts[-1][0]:
        return 100.0
    for (v0, s0), (v1, s1) in zip(pts, pts[1:]):
        if v_cell <= v1:
            return s0 + (s1 - s0) * (v_cell - v0) / (v1 - v0)
    return 100.0


class BatteryMonitor:
    HISTORY_S = 15 * 60      # historial que se le entrega al navegador al abrir la vista
    HISTORY_DT = 0.25        # s entre puntos del historial

    def __init__(self, cfg=None):
        self.cfg = dict(DEFAULTS)
        if cfg:
            self.cfg.update(cfg)
        self.reset()

    def reset(self):
        self.t_last = None           # ultimo mensaje (time.monotonic)
        self.t_start = None
        self.v = self.i = self.p = 0.0           # instantaneos (V, mA, W)
        self.v_f = self.i_f = self.p_f = None    # suavizados para mostrar
        self.i_avg = None                        # corriente media ~30 s (autonomia)
        self.used_mah = 0.0                      # del ESP32 (conteo de coulombs)
        self.used0 = None
        self.soc = None
        self.energy_wh = 0.0
        self.peak_ma = 0.0
        self.v_min = None
        self.v_max = None
        self.present = False
        self.cells = int(self.cfg['battery_cells'])
        self.hist = deque(maxlen=int(self.HISTORY_S / self.HISTORY_DT))
        self._t_hist = 0.0

    # ------------------------------------------------------------------
    def update(self, voltage_v, current_a, charge_ah, capacity_ah, present):
        """current_a con la convencion de sensor_msgs/BatteryState: < 0 descargando."""
        now = time.monotonic()
        dt = 0.0 if self.t_last is None else min(now - self.t_last, 2.0)
        self.t_last = now
        cap = capacity_ah * 1000.0 if capacity_ah and capacity_ah > 0 else float(self.cfg['battery_capacity_mah'])
        self.cfg['battery_capacity_mah'] = cap

        v = float(voltage_v)
        i_ma = -float(current_a) * 1000.0           # > 0 descargando
        if math.isnan(i_ma):
            i_ma = 0.0
        self.present = bool(present) and v > 1.0
        if not self.present:
            self.v, self.i, self.p = v, 0.0, 0.0
            return
        if self.t_start is None:
            self.t_start = now
        # Una "9 V" de 8 celdas (9.6 V) recien cargada pasa de 10.3 V
        if v > 10.3 and self.cells < 8:
            self.cells = 8

        self.v, self.i, self.p = v, i_ma, v * i_ma / 1000.0
        # Suavizado corto (0.3 s): el INA219 ya promedia 64 muestras, asi
        # que basta para limpiar la medida sin retrasar los cambios
        a = 1.0 if self.v_f is None else min(1.0, dt / 0.3)
        self.v_f = v if self.v_f is None else self.v_f + a * (v - self.v_f)
        self.i_f = i_ma if self.i_f is None else self.i_f + a * (i_ma - self.i_f)
        self.p_f = self.p if self.p_f is None else self.p_f + a * (self.p - self.p_f)
        b = 1.0 if self.i_avg is None else min(1.0, dt / 30.0)
        self.i_avg = max(i_ma, 0.0) if self.i_avg is None else self.i_avg + b * (max(i_ma, 0.0) - self.i_avg)

        self.peak_ma = max(self.peak_ma, i_ma)
        self.v_min = v if self.v_min is None else min(self.v_min, v)
        self.v_max = v if self.v_max is None else max(self.v_max, v)
        self.energy_wh += max(self.p, 0.0) * dt / 3600.0

        # Carga consumida desde que el ESP32 arranco (charge = capacidad - consumido)
        used = cap - charge_ah * 1000.0 if charge_ah is not None and not math.isnan(charge_ah) else self.used_mah
        if self.used0 is None or used < self.used_mah - 1.0:
            self.used0 = used                         # el carro se reinicio: nueva cuenta
        d_used = max(used - self.used_mah, 0.0) if self.soc is not None else 0.0
        self.used_mah = used

        # SOC por tension (con la caida interna compensada)
        v_ocv = v + max(i_ma, 0.0) / 1000.0 * float(self.cfg['battery_r_int_ohm'])
        soc_v = soc_from_cell_voltage(v_ocv / self.cells)
        if self.soc is None:
            self.soc = soc_v
        else:
            self.soc -= d_used / cap * 100.0
            k = (1.0 / 60.0) if i_ma < 80.0 else (1.0 / 600.0)   # en reposo la tension es fiable
            self.soc += min(1.0, k * dt) * (soc_v - self.soc)
        self.soc = max(0.0, min(100.0, self.soc))

        if now - self._t_hist >= self.HISTORY_DT:
            self._t_hist = now
            # el historial guarda la medida suavizada (la misma que se grafica en vivo)
            self.hist.append((round(time.time(), 2), round(self.v_f, 4), round(self.i_f, 2), round(self.p_f, 4), round(self.soc, 1)))

    # ------------------------------------------------------------------
    def state(self):
        now = time.monotonic()
        age = None if self.t_last is None else now - self.t_last
        cap = float(self.cfg['battery_capacity_mah'])
        if age is None or age > 3.0:
            status = 'sin_datos'
        elif not self.present:
            status = 'desconectada'
        elif self.soc is not None and self.soc <= self.cfg['battery_critical_pct']:
            status = 'critica'
        elif self.soc is not None and self.soc <= self.cfg['battery_low_pct']:
            status = 'baja'
        elif (self.i_f or 0.0) < 15.0:
            status = 'reposo'
        else:
            status = 'descargando'
        remaining_mah = None if self.soc is None else self.soc / 100.0 * cap
        runtime_min = None
        if remaining_mah is not None and self.i_avg is not None and self.i_avg > 15.0:
            runtime_min = remaining_mah / self.i_avg * 60.0
        session_used = None if self.used0 is None else max(self.used_mah - self.used0, 0.0)
        return {
            'status': status,
            'age': age,
            'present': self.present,
            'v': self.v_f if self.v_f is not None else self.v,
            'v_raw': self.v,
            'v_cell': (self.v_f or self.v) / self.cells if self.cells else None,
            'i_ma': self.i_f if self.i_f is not None else self.i,
            'i_raw': self.i,
            'i_avg_ma': self.i_avg,
            'p_w': self.p_f if self.p_f is not None else self.p,
            'soc': self.soc,
            'remaining_mah': remaining_mah,
            'runtime_min': runtime_min,
            'used_mah': session_used,
            'energy_wh': self.energy_wh,
            'peak_ma': self.peak_ma,
            'v_min': self.v_min,
            'v_max': self.v_max,
            'session_s': None if self.t_start is None else now - self.t_start,
            'cells': self.cells,
            'capacity_mah': cap,
            'low_pct': self.cfg['battery_low_pct'],
            'critical_pct': self.cfg['battery_critical_pct'],
            'r_int': self.cfg['battery_r_int_ohm'],
            'packs': int(self.cfg['battery_packs']),
        }

    def history(self):
        return list(self.hist)
