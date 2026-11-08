"""Read NASA's OpenVSP model of the X-57 (refs/geometry/X-57.vsp3) without OpenVSP: it is XML.

Every component carries its absolute position and rotation, in inches on the airplane's stations (x aft, y right,
z up). A component with planar symmetry about y = 0 is stored once, for the right side, so six high-lift propellers
and one cruise propeller stand for twelve and two.

    m = Model(path)
    m.named('HL-Prop')            # the components of that name
    wing_sections(m.first('CleanWing'))
    blade(m.first('Cruise-Prop'))
"""
import re
import numpy as np
import xml.etree.ElementTree as ET


class Comp:
    """one OpenVSP component"""

    def __init__(s, g):
        s.g = g
        pc = g.find('ParmContainer')
        s.pc = pc
        s.name = pc.find('Name').text
        s.id = pc.find('ID').text if pc.find('ID') is not None else None
        gb = g.find('GeomBase')
        s.type = gb.find('TypeName').text
        s.parent = gb.find('ParentID').text if gb.find('ParentID') is not None else None
        x = s.group('XForm')
        s.loc = np.array([x['X_Location'], x['Y_Location'], x['Z_Location']])
        s.rot = np.array([x['X_Rotation'], x['Y_Rotation'], x['Z_Rotation']])
        s.mirrored = int(s.group('Sym').get('Sym_Planar_Flag', 0)) == 2       # XZ plane

    def group(s, name):
        e = s.pc.find(name)
        return {} if e is None else {c.tag: float(c.attrib['Value']) for c in e if 'Value' in c.attrib}


class Model:
    def __init__(s, path):
        s.root = ET.parse(path).getroot()
        s.comps = []
        for g in s.root.iter('Geom'):
            pc = g.find('ParmContainer')
            if pc is not None and pc.find('Name') is not None and g.find('GeomBase') is not None:
                s.comps.append(Comp(g))

    def named(s, name):
        return [c for c in s.comps if c.name == name]

    def first(s, name):
        return s.named(name)[0]


def vec(text):
    return np.array([float(x) for x in text.replace('\n', ' ').split(',') if x.strip()])


def airfoil(fa):
    """a FileAirfoil element: its name and its (x, z) upper and lower surfaces, leading edge first, chord 1"""
    up, lo = vec(fa.find('UpperPnts').text).reshape(-1, 3), vec(fa.find('LowerPnts').text).reshape(-1, 3)
    return fa.find('AirfoilName').text, up[:, :2], lo[:, :2]


def thickness(up, lo):
    """largest thickness over chord, with both surfaces on one x grid"""
    u, l = up[np.argsort(up[:, 0])], lo[np.argsort(lo[:, 0])]      # a flap's nose can run back on itself
    x0, x1 = max(u[0, 0], l[0, 0]), min(u[-1, 0], l[-1, 0])
    x = np.linspace(x0, x1, 401)
    return float((np.interp(x, u[:, 0], u[:, 1]) - np.interp(x, l[:, 0], l[:, 1])).max() / (x1 - x0))


def wing_sections(c):
    """the panels of a Wing component, root to tip: span, chords, sweep, twist, dihedral (in, deg) and the airfoil at
    the panel's outer end. OpenVSP's first section only holds the root airfoil, with a zero-length panel"""
    out = []
    for xs in c.g.find('WingGeom').iter('XSec'):
        pc = xs.find('ParmContainer')
        if pc is None or pc.find('XSec') is None:
            continue
        p = {e.tag: float(e.attrib['Value']) for e in pc.find('XSec') if 'Value' in e.attrib}
        fa = xs.find('.//FileAirfoil')
        out.append(dict(p, airfoil=None if fa is None else airfoil(fa)))
    return out


def body_sections(c):
    """a Fuselage component's cross-sections: station, height of the centre, full height and width (in). The first and last are points"""
    L = c.group('Design')['Length']
    out = []
    for x in c.g.find('FuselageGeom').iter('XSec'):
        pc = x.find('ParmContainer')
        if pc is None or pc.find('XSec') is None:
            continue
        d = {e.tag: float(e.attrib['Value']) for e in pc.find('XSec') if 'Value' in e.attrib}
        q = x.find('.//XSecCurve/ParmContainer')
        h = {e.tag: float(e.attrib['Value']) for g in (q if q is not None else []) for e in g if 'Value' in e.attrib}
        out.append(dict(x=c.loc[0] + d['XLocPercent'] * L, z=c.loc[2] + d['ZLocPercent'] * L, h=h.get('Height', 0.0), w=h.get('Width', 0.0)))
    return out


def fin_panels(c):
    """a Wing component standing on end: its panels bottom to top, as height, leading edge and chord at each panel's ends (in). A panel's
    sweep is at its leading edge, or at its trailing edge where Sweep_Location says so"""
    z, le = c.loc[2], c.loc[0]
    out = []
    for p in wing_sections(c):
        b, c0, c1, sw = p.get('Span', 0.0), p.get('Root_Chord', 0.0), p.get('Tip_Chord', 0.0), p.get('Sweep', 0.0)
        if b <= 1.01:
            continue
        run = b * np.tan(np.radians(sw))
        le1 = le + run if p.get('Sweep_Location', 0.0) < 0.5 else le + c0 + run - c1
        out.append(dict(z0=z, z1=z + b, x0=le, x1=le1, c0=c0, c1=c1))
        z, le = z + b, le1
    return out


def blade(c):
    """a Propeller component's blade: the stations r/R with chord/R, twist (deg), rake/R, skew/R and thickness/chord
    as OpenVSP's curves hold them, and the propeller's own parameters"""
    p = c.g.find('PropellerGeom')
    out = {}
    for tag in ('Chord', 'Twist', 'Rake', 'Skew', 'Thick'):
        curve = p.find(tag)
        vals = {}
        for e in curve.iter():
            m = re.match(r'([a-zA-Z]+)_(\d+)$', e.tag)
            if m and 'Value' in e.attrib:
                vals.setdefault(m.group(1), {})[int(m.group(2))] = float(e.attrib['Value'])
        rk = vals.pop('r')
        vk = next(iter(vals.values()))
        i = sorted(rk)
        out[tag.lower()] = (np.array([rk[k] for k in i]), np.array([vk[k] for k in i]))
    d = c.group('Design')
    foils = []
    for fa in c.g.iter('FileAirfoil'):
        n, up, lo = airfoil(fa)
        if n not in [f[0] for f in foils]:
            foils.append((n, up, lo))
    return dict(curves=out, diameter_in=d['Diameter'], blades=int(d['NumBlade']), beta34=d['Beta34'],
                feather=d['Feather'], use_beta34=bool(d['UseBeta34Flag']), foils=foils)
