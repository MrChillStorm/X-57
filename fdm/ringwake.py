"""The axial speed a tube of ring vortices gives a wing strip behind a propeller. The rings as the V-22's
rotorwake.py (Heyson and Katzoff, uniform loading), straight behind the disc, contracting to the far jet's radius. In units of the speed
added at the disc: 1 at its centre, 2 far behind"""
import math
from functools import lru_cache
import numpy as np
from scipy.special import ellipk, ellipe
from scipy.integrate import quad

LAMBDA_C = 5.0


def ring_uz(r, z):
    a = (1 + r) ** 2 + z * z
    m = min(4 * r / a, 1 - 1e-12)
    d = (1 - r) ** 2 + z * z
    return (ellipk(m) + (1 - r * r - z * z) / max(d, 1e-9) * ellipe(m)) / (2 * math.pi * math.sqrt(a))


@lru_cache(maxsize=None)
def at(x, r, k):
    """x behind the disc, r off its axis, in radii; k the far radius"""
    def f(s):
        Rs = 1.0 - (1.0 - k) * (1.0 - math.exp(-LAMBDA_C * s))
        return ring_uz(r / Rs, (x - s) / Rs) / Rs
    pts = [x] if x > 0 else None
    v = quad(f, 0.0, 30.0, points=pts, limit=200)[0] + quad(f, 30.0, 2000.0, limit=100)[0]
    return 2.0 * v


def mean_axial(x, dy, dz, R, k):
    r = np.hypot(np.asarray(dy), dz) / R
    return float(np.mean([at(round(x / R, 3), round(float(q), 2), k) for q in r]))
