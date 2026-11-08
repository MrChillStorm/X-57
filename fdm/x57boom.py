"""The X-57's nose boom: "A flight-test nose boom of the National Advisory Committee for Aeronautics (NACA) style was also added to
the airplane to measure airspeed by capturing total pressure; static pressure; angle of attack (alpha); and sideslip angle (beta)"
[20230004174]; "installation of a large air data boom in the nose" [20240010807]. NASA's rendering [20205010303 fig. 2] and the
photograph of N857NA show where: out of the nose's right side at its white and blue paint line, parallel to the fuselage, and well
ahead of it, the two vanes near its tip; polished metal. No document gives a dimension, so these are mine, from the rendering: its pitot tip 1.2 m
ahead of the nose, a tube 38 mm across at the nose tapering to 25 mm, a 16 mm probe its last 0.36 m; the angle-of-attack vane on a
stub outboard, to the right, the sideslip vane on a stub below, each a flag 60 x 50 mm, trailing into the airflow in flight (JSBSim's alpha
and beta, above 30 kt). Writes Models/boom.ac and boom.xml, in the fuselage's frame, and includes them in fuselage.xml.
"""
import math, os, re
import numpy as np
import mkmodel as M

NOSE = (-4.346, 0.0, -0.664)          # the fuselage's nose tip (fuselage.ac, x aft, y right, z up)
ROOT_Y, ROOT_X = 0.115, 0.07          # the boom's axis: this far right of the centreline, its root this far aft of the nose tip
AHEAD = 1.20                          # its pitot tip ahead of the nose tip
VANE_AT = 0.48                        # the vanes' pivots behind the tip
STUB, FLAG = 0.06, (0.060, 0.050, 0.002)


def ring(x, y, z, r, n=14):
    return [(x, y + r * math.cos(2 * math.pi * k / n), z + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def lathe(stations, y, z, n=14):
    """a tube along x: stations (x, r) from the root forward, its ends closed"""
    V, F = [], []
    for x, r in stations:
        V += ring(x, y, z, r, n)
    for i in range(len(stations) - 1):
        for k in range(n):
            a, b = i * n + k, i * n + (k + 1) % n
            F.append((a, a + n, b + n, b))
    F.append(tuple(range(n))[::-1])
    F.append(tuple(range((len(stations) - 1) * n, len(stations) * n)))
    return V, F


def box(c, size):
    """a box at c: size along x, y, z"""
    cx, cy, cz = c
    hx, hy, hz = (s / 2 for s in size)
    V = [(cx + sx * hx, cy + sy * hy, cz + sz * hz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    F = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return V, F


def obj(name, V, F, axis=None):
    """an AC3D object, model (x, y, z) to AC's (x, z, -y), every face outward: from its own centre, or for a tube from its axis (y, z)"""
    V = np.array(V, float)
    out = ['OBJECT poly', 'name "%s"' % name, 'numvert %d' % len(V)]
    out += ['%.5f %.5f %.5f' % (x, z, -y) for x, y, z in V]
    out.append('numsurf %d' % len(F))
    c = V.mean(0)
    for f in F:
        p = V[list(f)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        d = p.mean(0) - c
        if axis is not None:
            r = p.mean(0) - np.array((p.mean(0)[0], axis[0], axis[1]))
            d = r if np.linalg.norm(r) > 1e-4 else d
        if np.dot(n, d) < 0:
            f = tuple(reversed(f))
        out += ['SURF 0x10', 'mat 0', 'refs %d' % len(f)] + ['%d 0 0' % i for i in f]
    out.append('kids 0')
    return '\n'.join(out)


def install(out):
    m = os.path.join(out, 'Models')
    x0, _, z = NOSE
    root, tip = x0 + ROOT_X, x0 - AHEAD
    stations = [(root, 0.019), (x0 - 0.04, 0.019), (tip + 0.40, 0.0125), (tip + 0.39, 0.0085), (tip + 0.36, 0.008), (tip + 0.03, 0.008),
                (tip + 0.01, 0.006), (tip, 0.0035)]
    objs = [obj('boom', *lathe(stations, ROOT_Y, z), axis=(ROOT_Y, z))]
    xv = tip + VANE_AT
    r = 0.0125 + (0.019 - 0.0125) * (xv - (tip + 0.40)) / ((x0 - 0.04) - (tip + 0.40))
    # the angle-of-attack vane: a stub outboard (right), a horizontal flag trailing from its end; the sideslip vane: a stub down, a vertical flag
    pa = (xv, ROOT_Y + r + STUB, z)
    pb = (xv, ROOT_Y, z - r - STUB)
    objs.append(obj('boom-stub-a', *box((xv, ROOT_Y + r + STUB / 2, z), (0.006, STUB, 0.006))))
    objs.append(obj('boom-stub-b', *box((xv, ROOT_Y, z - r - STUB / 2), (0.006, 0.006, STUB))))
    objs.append(obj('boom-vane-a', *box((xv + FLAG[0] / 2, pa[1] + FLAG[1] / 2 - 0.004, z), (FLAG[0], FLAG[1], FLAG[2]))))
    objs.append(obj('boom-vane-b', *box((xv + FLAG[0] / 2, ROOT_Y, pb[2] - FLAG[1] / 2 + 0.004), (FLAG[0], FLAG[2], FLAG[1]))))
    ac = ['AC3Db', 'MATERIAL "boom" rgb 0.70 0.71 0.73  amb 0.4 0.4 0.4  emis 0 0 0  spec 0.6 0.6 0.6  shi 64  trans 0',
          'OBJECT world', 'kids %d' % len(objs)] + objs
    open(os.path.join(m, 'boom.ac'), 'w').write('\n'.join(ac) + '\n')
    gate = '<condition><greater-than><property>velocities/airspeed-kt</property><value>30</value></greater-than></condition>'
    vane = lambda n, prop, axis, c: ('<animation><type>rotate</type><object-name>%s</object-name>%s<property>%s</property>'
                                     '<factor>-1</factor><min-deg>-30</min-deg><max-deg>30</max-deg>'
                                     '<center><x-m>%.4f</x-m><y-m>%.4f</y-m><z-m>%.4f</z-m></center><axis><x>%d</x><y>%d</y><z>%d</z></axis></animation>'
                                     % (n, gate, prop, c[0], c[1], c[2], *axis))
    xml = ['<?xml version="1.0"?>', '<!-- the X-57\'s air-data nose boom: written by fdm/x57boom.py -->', '<PropertyList>', '<path>boom.ac</path>',
           # its flags trail into the airflow: with the wind from below (alpha up) the alpha flag's tail rises, from the right (beta) the beta flag's swings left
           vane('boom-vane-a', 'fdm/jsbsim/aero/alpha-deg', (0, 1, 0), pa),
           vane('boom-vane-b', 'fdm/jsbsim/aero/beta-deg', (0, 0, 1), pb),
           M.effect('x57-polished', ('boom', 'boom-stub-a', 'boom-stub-b', 'boom-vane-a', 'boom-vane-b')),
           '</PropertyList>']
    open(os.path.join(m, 'boom.xml'), 'w').write('\n'.join(xml) + '\n')
    p = os.path.join(m, 'fuselage.xml')
    t = open(p, errors='replace').read()
    if 'boom.xml' not in t:
        t = t.replace('</PropertyList>', '<model><path>boom.xml</path></model>\n</PropertyList>', 1)
        open(p, 'w', errors='replace').write(t)
