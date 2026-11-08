"""Filtros de Kalman genericos: extendido (EKF) y sin esencia (UKF).

Ambos comparten la misma interfaz para poder correrlos en paralelo con
los mismos datos y compararlos (precision, tiempo de computo, memoria):

    kf.predict(f, dt, u, F_jac=None)          # F_jac solo lo usa el EKF
    kf.update(z, h, R, H_jac=None, z_angles=())

- f(x, u, dt) -> x'          modelo de proceso (puede ser no lineal)
- h(x) -> z                  modelo de medicion (puede ser no lineal)
- Las componentes angulares del estado (angle_idx) y de la medicion
  (z_angles) se manejan con resta envuelta en [-pi, pi] y, en el UKF,
  con media circular de los puntos sigma.
"""

import math

import numpy as np


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _residual(a, b, angles):
    d = np.asarray(a, dtype=float) - np.asarray(b, dtype=float)
    for i in angles:
        d[i] = wrap_pi(d[i])
    return d


class _Base:
    def __init__(self, x0, P0, Q_fn, angle_idx=()):
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.n = self.x.size
        self.Q_fn = Q_fn
        self.angle_idx = tuple(angle_idx)
        self.last_nis = 0.0

    def _normalize(self):
        for i in self.angle_idx:
            self.x[i] = wrap_pi(self.x[i])
        # Simetria numerica de P
        self.P = 0.5 * (self.P + self.P.T)

    def reset(self, x0, P0):
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.last_nis = 0.0

    def nbytes(self):
        """Memoria de los arreglos propios del filtro (bytes)."""
        return self.x.nbytes + self.P.nbytes


class EKF(_Base):
    """Filtro de Kalman extendido: linealiza f y h con sus jacobianos."""

    name = 'EKF'

    def predict(self, f, dt, u, F_jac=None):
        F = F_jac(self.x, u, dt)
        self.x = np.asarray(f(self.x, u, dt), dtype=float)
        self.P = F @ self.P @ F.T + self.Q_fn(dt)
        self._normalize()

    def update(self, z, h, R, H_jac=None, z_angles=()):
        H = H_jac(self.x)
        y = _residual(z, h(self.x), z_angles)
        S = H @ self.P @ H.T + R
        S_inv = np.linalg.inv(S)
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ y
        # Forma de Joseph: mantiene P simetrica y definida positiva
        I_KH = np.eye(self.n) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self._normalize()
        self.last_nis = float(y @ S_inv @ y)
        return self.last_nis


class UKF(_Base):
    """Filtro de Kalman sin esencia (puntos sigma escalados de Van der Merwe).

    No usa jacobianos: propaga 2n+1 puntos sigma por los modelos no
    lineales y recupera media y covarianza de los puntos transformados.
    """

    name = 'UKF'

    def __init__(self, x0, P0, Q_fn, angle_idx=(), alpha=0.3, beta=2.0, kappa=0.0):
        super().__init__(x0, P0, Q_fn, angle_idx)
        n = self.n
        self.lam = alpha * alpha * (n + kappa) - n
        c = n + self.lam
        self.Wm = np.full(2 * n + 1, 0.5 / c)
        self.Wc = self.Wm.copy()
        self.Wm[0] = self.lam / c
        self.Wc[0] = self.lam / c + (1.0 - alpha * alpha + beta)
        self._c = c
        self._sigmas = np.zeros((2 * n + 1, n))

    def nbytes(self):
        return super().nbytes() + self._sigmas.nbytes + self.Wm.nbytes + self.Wc.nbytes

    def _sigma_points(self):
        n = self.n
        P = self.P
        try:
            L = np.linalg.cholesky(self._c * P)
        except np.linalg.LinAlgError:
            # P perdio la definicion positiva por redondeo: se repara
            w, V = np.linalg.eigh(0.5 * (P + P.T))
            P = V @ np.diag(np.maximum(w, 1e-9)) @ V.T
            self.P = P
            L = np.linalg.cholesky(self._c * P)
        s = self._sigmas
        s[0] = self.x
        for i in range(n):
            s[i + 1] = self.x + L[:, i]
            s[n + i + 1] = self.x - L[:, i]
        return s

    def _mean(self, pts, angles):
        m = self.Wm @ pts
        for i in angles:
            m[i] = math.atan2(self.Wm @ np.sin(pts[:, i]), self.Wm @ np.cos(pts[:, i]))
        return m

    def predict(self, f, dt, u, F_jac=None):
        sig = self._sigma_points()
        Y = np.array([f(s, u, dt) for s in sig], dtype=float)
        x = self._mean(Y, self.angle_idx)
        P = self.Q_fn(dt).copy()
        for k in range(Y.shape[0]):
            d = _residual(Y[k], x, self.angle_idx)
            P += self.Wc[k] * np.outer(d, d)
        self.x, self.P = x, P
        self._normalize()

    def update(self, z, h, R, H_jac=None, z_angles=()):
        sig = self._sigma_points()
        Z = np.array([h(s) for s in sig], dtype=float)
        if Z.ndim == 1:
            Z = Z[:, None]
        z_mean = self._mean(Z, z_angles)
        m = Z.shape[1]
        S = np.array(R, dtype=float).reshape(m, m).copy()
        Pxz = np.zeros((self.n, m))
        for k in range(Z.shape[0]):
            dz = _residual(Z[k], z_mean, z_angles)
            dx = _residual(sig[k], self.x, self.angle_idx)
            S += self.Wc[k] * np.outer(dz, dz)
            Pxz += self.Wc[k] * np.outer(dx, dz)
        S_inv = np.linalg.inv(S)
        K = Pxz @ S_inv
        y = _residual(z, z_mean, z_angles)
        self.x = self.x + K @ y
        self.P = self.P - K @ S @ K.T
        self._normalize()
        self.last_nis = float(y @ S_inv @ y)
        return self.last_nis
