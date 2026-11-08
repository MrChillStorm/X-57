"""A discrete lifting line for the built-up model: horseshoes on strips of a span that may leave the plane (dihedral, winglets).

A strip runs from A to B across the span, listed left to right, in (y, z). Its bound vortex lies at the quarter chord and its legs run straight aft.
Velocities are in the crossflow plane: at the bound vortex each leg gives half of what it gives far behind.
"""
import numpy as np


def normals(rows):
    """each strip's upward normal in (y, z)"""
    t = np.array([[r['yb'] - r['ya'], r['zb'] - r['za']] for r in rows], float)
    t /= np.linalg.norm(t, axis=1)[:, None]
    return np.c_[-t[:, 1], t[:, 0]]


def influence(rows):
    """F[k, j]: the flow down the normal at strip k, per unit circulation of strip j: an induced angle times speed"""
    n = len(rows)
    P = np.array([[0.5 * (r['ya'] + r['yb']), 0.5 * (r['za'] + r['zb'])] for r in rows])
    N = normals(rows)
    F = np.zeros((n, n))
    for j, r in enumerate(rows):
        for ye, ze, s in ((r['yb'], r['zb'], 1.0), (r['ya'], r['za'], -1.0)):
            d = P - (ye, ze)
            r2 = (d ** 2).sum(axis=1)
            vel = s / (4 * np.pi * r2)[:, None] * np.c_[-d[:, 1], d[:, 0]]
            F[:, j] -= (vel * N).sum(axis=1)
    return F


def operator(rows, a_sec=6.2, k_ind=1.0):
    """the pre-solved line. With section lift cl = cl_g - a alpha_i, the induced angle is Kf (cl_g V) / V_k, where cl_g is the lift at the geometric angle"""
    c = np.array([r['c'] for r in rows])
    F = influence(rows) * (0.5 * c)[None, :] * k_ind
    return np.linalg.solve(np.eye(len(rows)) + a_sec * F, F)


def solve(rows, alpha, a_sec=6.2, k_ind=1.0):
    """section lift, induced angle and the coefficients of a geometric angle per strip (rad)"""
    cl_g = a_sec * np.asarray(alpha, float)
    ai = operator(rows, a_sec, k_ind) @ cl_g
    cl = cl_g - a_sec * ai
    area = np.array([r['area'] for r in rows])
    N = normals(rows)
    L = cl * area * N[:, 1]
    return cl, ai, L.sum(), (cl * area * ai).sum()


def field(rows, gam, x, y, z, dz=0.0):
    """the air's upward speed at a point from circulations gam, the wake and the bound vortices lifted by dz. Legs run aft along x from the quarter chord"""
    w = 0.0
    for r, g in zip(rows, gam):
        xb = r['x4']
        for ye, ze, s in ((r['yb'], r['zb'], 1.0), (r['ya'], r['za'], -1.0)):
            ay, az, ax = y - ye, z - ze - dz, x - xb
            rr = ay * ay + az * az
            if rr < 1e-6:
                continue
            w += s * g / (4 * np.pi * rr) * (1 + ax / np.sqrt(ax * ax + rr)) * ay
        ax, az = x - xb, z - 0.5 * (r['za'] + r['zb']) - dz
        rho = np.hypot(ax, az)
        if rho > 1e-6:
            c1 = (r['yb'] - y) / np.hypot(r['yb'] - y, rho)
            c2 = (r['ya'] - y) / np.hypot(r['ya'] - y, rho)
            w -= g / (4 * np.pi * rho) * (c1 - c2) * ax / rho
    return w
