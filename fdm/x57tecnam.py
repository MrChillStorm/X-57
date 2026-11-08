#!/usr/bin/env python3
"""The Tecnam's cockpit lights, which the X-57 doesn't have: fitted only from the X-57 menu, and until then nothing of them is
in the scene, neither their lamps nor their controls.

The X-57 has no cockpit lighting. NASA's power drawings (ICD-CEPT-006 slide 44, 20240009560 fig. 5) keep the P2006T's INSTR
LIGHT circuit, 5 A on DC Converter Bus B, collared out; its photograph of the cockpit (slide 11) has blanks where the P2006T has
its dimmers, and NASA's traction power panel where the P2006T's overhead console was; its flights were to start within an hour
of sunrise [20240007413]. No NASA document plans any.

What a P2006T has [AFM 7.6, fig. 9; 7.17 fig. 17]:
- Instrument lights, on a switch on the right instrument panel, the fifth rocker of its LIGHTS row (AFM fig. 11, 20: LAND, TAXI,
  NAV, STROBE, INSTR; the X-57 has the first four and a blank, which the rocker replaces), in three parts, each on its own dimmer: the switches' built-in lights, the
  avionics' lights, and two cockpit lights in the overhead console that light the left and right panels. The circuit is NASA's:
  Bus B through the INSTR LIGHT breaker (a P2006T's left breaker rack has an INSTR breaker too, in a photograph of one).
- A cabin light, in the ceiling at the console's aft end, with its switch. The P2006T feeds it from the battery bus: here the
  Essential Bus, the X-57's nearest.
- An emergency light, five LEDs in the console's front, straight from the battery, its switch "on the LH breakers rack". NASA's
  traction power panel fills that front: the light sits just aft of it, on the Essential battery. No photograph shows that
  switch, and one of a P2006T's left rack and NASA's of the X-57's show breakers only: here it is on the lamp.

Places: the console's lamps from the FGAddon P2006T model's overhead console (its texture on its mesh), in the panel's frame,
under the cabin's roof. The dimmers are that model's three knobs and plate, at its own place and size: NASA's EXPERIMENTAL
placard holds that place on the X-57 (its photograph has blanks there), so fitting the lights takes the placard away; which knob is which isn't marked, so they go in
the flight manual's order, switches, avionics, cockpit. Levels are estimates, as the Bo 105's cockpit lights: the
cockpit lights give each panel half a mean 6 lux at full, built-in and key lights glow as a face so lit would look (6/pi cd/m2
for white), the cabin light 12 cd, the emergency light 5 cd; colour a warm white. Lit faces and lenses are swapped in only
while lit, the HDR lights only while on: off, they cost the renderer nothing.

    install(out)    out: the package (../X-57): writes Models/Interior/Panel/X57/tecnam*, the dimmer panel, the INSTR rocker,
                    lit copies of the rockers and avionics faces, Nasal/tecnam.nas and gui/dialogs/x57-tecnam-lights.xml
"""
import math, os, re
import numpy as np
from PIL import Image, ImageDraw

import mkmodel as M
import x57panel as P

FIT = 'sim/model/x57/tecnam-lights'
LV = '/instrumentation/x57/tecnam/'
CTL = '/controls/lighting/'
HDR_CD = 635.0
LIGHT_LUX = 6.0                         # an instrument face's mean illuminance at full (the Bo 105's)
INTEGRAL = LIGHT_LUX / math.pi          # cd/m2 of a white face so lit: the built-in and key lights at full
WARM = (1.0, 0.95, 0.85)
CABIN_CD, EMERG_CD = 12.0, 5.0
ROCKERS = ('landing', 'taxi', 'nav', 'strobe', 'instr')
AVIONICS = ('gtn', 'gtx', 'audio', 'uhf-face')
INSTR_Y = P.BLANK_Y                      # the fifth rocker, in the blank's place
LOWER = (0.0135, -0.285, -21)            # lower.xml's offsets: x, z, pitch
DIMMERS = (-0.00036, -0.38338, -0.21477)     # the Tecnam's own place (its panel.xml), where NASA's placard is: the placard goes when these are fitted
DIM_SCALE = 1.0


def console(p):
    """a point of the Tecnam's overhead console (its AC frame) in the panel's: all-overheads.xml puts it at
    (0.48876, 0, 0.42971), pitched 81.71 deg"""
    x, y, z = p[0], -p[2], p[1]
    t = math.radians(81.71)
    return np.array((0.48876 + x * math.cos(t) + z * math.sin(t), y, 0.42971 - x * math.sin(t) + z * math.cos(t)))


COCKPIT = {'l': console((0.0169, 0.0092, 0.0352)), 'r': console((0.0169, 0.0092, -0.0352))}
CABIN = console((0.0169, 0.1144, 0.0))


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def roof(out, x, y=0.0):
    """the cabin roof's underside above (x, y) of the panel's frame, from the interior's mesh"""
    off = re.search(r'<x-m>\s*([-\d.]+)\s*</x-m>\s*<y-m>[^<]*</y-m>\s*<z-m>\s*([-\d.]+)', open(os.path.join(out, 'Models', 'Interior', 'Panel', 'all-panels.xml')).read())
    o = np.array((x + float(off.group(1)), y, 0.25 + float(off.group(2))))
    best = None
    for name, V, polys in M_objects(os.path.join(out, 'Models', 'Interior', 'interior.ac')):
        Q = np.c_[V[:, 0], -V[:, 2], V[:, 1]]
        for p in polys:
            for k in range(1, len(p) - 1):
                a, b, c = Q[p[0]], Q[p[k]], Q[p[k + 1]]
                e1, e2 = b - a, c - a
                h = np.cross((0, 0, 1.0), e2)
                det = e1 @ h
                if abs(det) < 1e-12:
                    continue
                s = o - a
                u = (s @ h) / det
                q = np.cross(s, e1)
                v = q[2] / det
                t = (e2 @ q) / det
                if u >= 0 and v >= 0 and u + v <= 1 and t > 0 and (best is None or t < best):
                    best = t
    return 0.25 + best


def M_objects(path):
    out = []
    for o in re.split(r'(?=^OBJECT )', open(path, errors='replace').read(), flags=re.M):
        n, c = re.search(r'^name "([^"]+)"', o, re.M), re.search(r'^numvert (\d+)', o, re.M)
        if not (n and c):
            continue
        loc = re.search(r'^loc (.*)', o, re.M)
        L = o[c.end():].lstrip('\n').split('\n')
        V = np.array([[float(q) for q in l.split()[:3]] for l in L[:int(c.group(1))]])
        V = V + (np.array([float(q) for q in loc.group(1).split()]) if loc else 0.0)
        polys = [[int(l.split()[0]) for l in m.group(2).strip().split('\n')] for m in re.finditer(r'^refs (\d+)\n((?:\d+ \S+ \S+\n?)+)', o, re.M)]
        out.append((n.group(1), V, polys))
    return out


def frame(axis):
    a = unit(axis)
    u = unit(np.cross(a, (0, 0, 1.0)) if abs(a[2]) < 0.95 else np.cross(a, (1.0, 0, 0)))
    return a, u, np.cross(a, u)


def axis_lathe(name, base, axis, prof, tex=None, n=24):
    """round about `axis` from `base`: (radius, distance along it), closed at the far end"""
    a, u, w = frame(axis)
    v, s = [], []
    for r, d in prof:
        v += [tuple(np.asarray(base) + d * a + r * (math.cos(2 * math.pi * k / n) * u + math.sin(2 * math.pi * k / n) * w)) for k in range(n)]
    for i in range(len(prof) - 1):
        for k in range(n):
            j = (k + 1) % n
            s.append(P.surf((i * n + k, i * n + j, (i + 1) * n + j, (i + 1) * n + k), flag='0x30'))
    s.append(P.surf([(len(prof) - 1) * n + k for k in range(n)], [(0.5 + 0.5 * math.cos(2 * math.pi * k / n), 0.5 + 0.5 * math.sin(2 * math.pi * k / n)) for k in range(n)], flag='0x30'))
    return P.obj(name, v, s, tex)


def slab(name, c, half, tex=None):
    """a block on the panel's axes: centre c, half sizes (x, y, z); its underside takes the picture"""
    x0, y0, z0 = np.asarray(c) - half
    x1, y1, z1 = np.asarray(c) + half
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    s = [P.surf((0, 1, 2, 3), ((1, 0), (0, 0), (0, 1), (1, 1)), flag='0x30')]
    for q in ((4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
        s.append(P.surf(q, ((0.02, 0.5),) * 4, flag='0x30'))
    return P.obj(name, v, s, tex)


def select(objs, cond):
    return '<animation><type>select</type>%s<condition>%s</condition></animation>' % (''.join('<object-name>%s</object-name>' % o for o in objs), cond)


def lit_cond(prop, on=True):
    c = '<greater-than><property>%s</property><value>0</value></greater-than>' % prop
    return c if on else '<not>%s</not>' % c


def spot(name, pos, d, cd, exp, cut, rng, level):
    """an HDR spot light at `level` (0-1) x cd, there only while on"""
    pos, d = np.asarray(pos), unit(d)
    return ('<light><name>%s</name><type>spot</type><position><x-m>%.4f</x-m><y-m>%.4f</y-m><z-m>%.4f</z-m></position>'
            '<direction><lookat-x-m>%.4f</lookat-x-m><lookat-y-m>%.4f</lookat-y-m><lookat-z-m>%.4f</lookat-z-m></direction>'
            '<color><r>%g</r><g>%g</g><b>%g</b></color><intensity><property>%s</property><scale>%.6g</scale></intensity>'
            '<spot-exponent>%g</spot-exponent><spot-cutoff>%.1f</spot-cutoff><range-m>%.3f</range-m></light>'
            '<animation><type>noshadow</type><object-name>%s</object-name></animation>'
            '<animation><type>select</type><object-name>%s</object-name><condition><and><property>/sim/rendering/hdr/hdr-enabled</property>'
            '<greater-than><property>%s</property><value>0</value></greater-than></and></condition></animation>\n'
            % ((name,) + tuple(pos) + tuple(pos + d) + WARM + (level, cd / HDR_CD, exp, cut, rng, name, name, level)))


def flood(lamp, area, e=2.0):
    """a cockpit light on a panel half: aimed at its middle; the intensity (cd) that gives it a mean LIGHT_LUX, its cutoff
    and range. The panel's face is the plane x = 0, lit from behind the pilot's head"""
    (y0, y1), (z0, z1) = area
    g = np.linspace(0.02, 0.98, 25)
    Y, Z = np.meshgrid(y0 + g * (y1 - y0), z0 + g * (z1 - z0))
    F = np.stack([np.zeros(Y.size), Y.ravel(), Z.ravel()], 1)
    aim = np.array((0.0, (y0 + y1) / 2, (z0 + z1) / 2))
    axis = unit(aim - lamp)
    v = F - lamp
    dist = np.linalg.norm(v, axis=1)
    c = (v / dist[:, None]) @ axis
    cut = min(60.0, math.degrees(math.acos(c.min())) + 2)
    rng = 1.16 * dist.max()
    E = c ** e * (-(v[:, 0]) / dist) * np.clip(1 - (dist / rng) ** 4, 0, None) ** 2 / dist ** 2
    return axis, LIGHT_LUX / E.mean(), cut, rng


def duplicate(ac_path, names, suffix='-lit'):
    """a copy of each named object, beside it, for the lit face that is swapped in"""
    t = open(ac_path, errors='replace').read()
    blocks = re.split(r'(?=^OBJECT poly)', t, flags=re.M)
    add = []
    for b in blocks:
        n = re.search(r'^name "([^"]+)"', b, re.M)
        if n and n.group(1) in names and b.rstrip().endswith('kids 0'):
            add.append(re.sub(r'^name "([^"]+)"', 'name "\\1%s"' % suffix, b, count=1, flags=re.M).rstrip('\n') + '\n')
    if len(add) != len(names):
        raise SystemExit('x57tecnam: not all of %s found in %s' % (names, ac_path))
    k = int(re.search(r'^OBJECT world\nkids (\d+)', t, re.M).group(1))
    t = re.sub(r'^(OBJECT world\nkids )\d+', r'\g<1>%d' % (k + len(add)), t, count=1, flags=re.M)
    open(ac_path, 'w', errors='replace').write(t.rstrip('\n') + '\n' + ''.join(add))


def label(path, words, size=(256, 64), bg=(176, 179, 183), ink=(40, 42, 46)):
    im = Image.new('RGB', size, bg)
    P.centred(ImageDraw.Draw(im), size[0] / 2, size[1] / 2 + 6, words, 34, ink)
    im.save(path)


def lamps(out, d):
    """the console's lamps under the roof, the emergency light, their HDR lights: tecnam.xml in the panel's frame"""
    objs, xml = [], []
    housings, lenses = [], []
    for s, area in (('l', ((-0.50, -0.02), (-0.26, 0.156))), ('r', ((0.02, 0.50), (-0.26, 0.156)))):
        lamp = COCKPIT[s]
        axis, cd, cut, rng = flood(lamp, area)
        top = lamp - axis * 0.034                       # above the roof: the housing grows out of it
        objs.append(axis_lathe('tl-cockpit-%s' % s, top, axis, ((0.017, 0.0), (0.017, 0.030), (0.0135, 0.034)), 'cream.png'))
        objs.append(axis_lathe('tl-cockpit-%s-lens' % s, lamp, axis, ((0.0115, 0.0), (0.0115, 0.0008))))
        housings.append('tl-cockpit-%s' % s)
        lenses.append(('tl-cockpit-%s-lens' % s, LV + 'cockpit-nits'))
        xml.append(spot('tl-cockpit-%s-light' % s, lamp + axis * 0.003, axis, cd, 2, cut, rng, LV + 'cockpit'))
        COCKPIT_CD[s] = cd
    # the cabin light: a wide lens in the ceiling, its switch on the housing's aft edge
    z = roof(out, CABIN[0]) - 0.010
    c = np.array((CABIN[0], 0.0, z))
    objs.append(slab('tl-cabin', c, np.array((0.028, 0.052, 0.016)), 'cream.png'))
    objs.append(slab('tl-cabin-lens', c - (0, 0, 0.0165), np.array((0.021, 0.044, 0.0008))))
    housings.append('tl-cabin')
    lenses.append(('tl-cabin-lens', LV + 'cabin-nits'))
    sw = c + (0.029, 0.034, -0.004)
    objs.append(axis_lathe('tl-cabin-sw', sw, (0, 0, -1), ((0.0020, 0.0), (0.0024, 0.010), (0.0018, 0.012))))
    xml.append('<animation><type>rotate</type><object-name>tl-cabin-sw</object-name><property>%scabin-light</property><interpolation><entry><ind>0</ind><dep>-24</dep></entry><entry><ind>1</ind><dep>24</dep></entry></interpolation>'
               '<axis><x>0</x><y>1</y><z>0</z></axis><center><x-m>%.4f</x-m><y-m>%.4f</y-m><z-m>%.4f</z-m></center></animation>' % ((CTL,) + tuple(sw)))
    xml.append('<animation><type>pick</type><object-name>tl-cabin-sw</object-name><action><button>0</button><repeatable type="bool">false</repeatable>'
               '<binding><command>property-toggle</command><property>/sim/model/x57/click/rocker</property></binding>'
               '<binding><command>property-toggle</command><property>%scabin-light</property></binding></action></animation>' % CTL)
    xml.append(spot('tl-cabin-light', c - (0, 0, 0.018), (0, 0, -1), CABIN_CD, 1, 75, 1.5, LV + 'cabin'))
    # the emergency light: five LEDs aft of NASA's panel, aimed at the panel's middle
    ex = 0.462
    z = roof(out, ex) - 0.006
    e = np.array((ex, 0.0, z))
    objs.append(slab('tl-emerg', e, np.array((0.010, 0.032, 0.006)), 'cream.png'))
    leds = []
    for i in range(5):
        p = e + (-0.002, -0.020 + 0.010 * i, -0.0062)
        leds.append(axis_lathe('tl-emerg-led-%d' % i, p, (0, 0, -1), ((0.0024, 0.0), (0.0024, 0.0006)), n=12))
    objs += leds
    housings.append('tl-emerg')
    lenses += [('tl-emerg-led-%d' % i, LV + 'emergency-nits') for i in range(5)]
    sw = e + (0.011, 0.024, -0.002)
    objs.append(axis_lathe('tl-emerg-sw', sw, (0, 0, -1), ((0.0016, 0.0), (0.0019, 0.008), (0.0014, 0.0095))))
    xml.append('<animation><type>rotate</type><object-name>tl-emerg-sw</object-name><property>%semergency-light</property><interpolation><entry><ind>0</ind><dep>-24</dep></entry><entry><ind>1</ind><dep>24</dep></entry></interpolation>'
               '<axis><x>0</x><y>1</y><z>0</z></axis><center><x-m>%.4f</x-m><y-m>%.4f</y-m><z-m>%.4f</z-m></center></animation>' % ((CTL,) + tuple(sw)))
    xml.append('<animation><type>pick</type><object-name>tl-emerg-sw</object-name><action><button>0</button><repeatable type="bool">false</repeatable>'
               '<binding><command>property-toggle</command><property>/sim/model/x57/click/rocker</property></binding>'
               '<binding><command>property-toggle</command><property>%semergency-light</property></binding></action></animation>' % CTL)
    xml.append(spot('tl-emerg-light', e - (0, 0, 0.008), np.array((0.0, 0.0, -0.05)) - e, EMERG_CD, 1, 60, 1.2, LV + 'emergency'))
    with open(os.path.join(d, 'tecnam.ac'), 'w') as f:
        f.write(M.ac(objs))
    every = housings + [n for n, _ in lenses] + ['tl-cabin-sw', 'tl-emerg-sw']
    xml.insert(0, select(every, '<property>%s</property>' % FIT))
    xml.append(M.effect('x57-face', housings))                    # cream, as NASA's overhead panel's housing
    xml.append(M.effect('x57-case', ['tl-cabin-sw', 'tl-emerg-sw']))
    by = {}
    for n, lv in lenses:
        by.setdefault(lv, []).append(n)
    for lv, ns in by.items():
        xml.append(P.lit(ns, lv, '%g %g %g' % WARM, 0.8))
    with open(os.path.join(d, 'tecnam.xml'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<!-- the Tecnam\'s cockpit lights, written by fdm/x57tecnam.py (see there): not on the X-57, fitted from its menu -->\n<PropertyList>\n<path>tecnam.ac</path>\n%s\n</PropertyList>\n' % '\n'.join(xml))
    return '<model><path>X57/tecnam.xml</path></model>'


COCKPIT_CD = {}


def scale_ac(src, dst, k):
    """an AC model scaled about its origin"""
    out, n = [], 0
    for line in open(src, errors='replace').read().split('\n'):
        if n > 0:
            out.append(' '.join('%.6f' % (float(q) * k) for q in line.split()[:3]))
            n -= 1
            continue
        if line.startswith('loc '):
            line = 'loc ' + ' '.join('%.6f' % (float(q) * k) for q in line.split()[1:4])
        elif line.startswith('numvert '):
            n = int(line.split()[1])
        out.append(line)
    open(dst, 'w').write('\n'.join(out))


def dimmers(base):
    """the Tecnam's three dimmer knobs, its light panel, back in their place: switches, avionics, cockpit"""
    src = os.path.join(base, 'Instruments', 'lights')
    scale_ac(os.path.join(src, 'lightpanel.ac'), os.path.join(src, 'lightpanel-x57.ac'), DIM_SCALE)
    knobs = (('btn-panel', -0.03, 'switches', 'Switch lights'), ('btn-instr', 0.0, 'avionics', 'Avionics lights'), ('btn-other', 0.03, 'cockpit', 'Cockpit lights'))
    x = ['<path>lightpanel-x57.ac</path>', select(['base'] + [k[0] for k in knobs], '<property>%s</property>' % FIT)]
    for obj, y, n, words in knobs:
        p = CTL + n + '-norm'
        x.append('<animation><type>knob</type><drag-direction>vertical</drag-direction><object-name>%s</object-name><action><binding><command>property-adjust</command>'
                 '<property>%s</property><min>0</min><max>1</max><factor>0.02</factor></binding></action>'
                 '<hovered><binding><command>set-tooltip</command><tooltip-id>tecnam-%s</tooltip-id><label>%s: %%3d%%%%</label><property>%s</property><mapping>percent</mapping></binding></hovered></animation>'
                 % (obj, p, n, words, p))
        x.append('<animation><type>rotate</type><object-name>%s</object-name><property>%s</property><factor>270</factor><center><x-m>%.5f</x-m><y-m>%.5f</y-m><z-m>0</z-m></center>'
                 '<axis><x>-1</x><y>0</y><z>0</z></axis></animation>' % (obj, p, 0.00052 * DIM_SCALE, y * DIM_SCALE))
    with open(os.path.join(src, 'lightpanel.xml'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<!-- the Tecnam\'s instrument light dimmers [AFM 7.17 fig. 17, 13], written by fdm/x57tecnam.py: fitted from the X-57 menu -->\n<PropertyList>\n%s\n</PropertyList>\n' % '\n'.join(x))
    return '<model><path>Instruments/lights/lightpanel.xml</path><offsets><x-m>%.5f</x-m><y-m>%.5f</y-m><z-m>%.5f</z-m></offsets></model>' % DIMMERS


def rockers(base):
    """the INSTR rocker in the LIGHTS row, and a lit face for each rocker, swapped in while its built-in light is on"""
    sw = os.path.join(base, 'Instruments', 'switch')
    duplicate(os.path.join(sw, 'on-off.ac'), ['on-off'])
    t = open(os.path.join(sw, 'landing-lights.xml'), errors='replace').read()
    open(os.path.join(sw, 'instr-lights.xml'), 'w', errors='replace').write(t.replace('controls/lighting/landing-lights', CTL.strip('/') + '/instr-light'))
    nits = LV + 'switches-nits'
    for n in ROCKERS:
        p = os.path.join(sw, n + '-lights.xml')
        t = open(p, errors='replace').read()
        if 'on-off-lit' in t:
            continue
        t = t.replace('<object-name>on-off</object-name>', '<object-name>on-off</object-name><object-name>on-off-lit</object-name>')
        t = re.sub(r'(<effect><inherits-from>[^<]*x57-face</inherits-from>.*?)<object-name>on-off-lit</object-name>', r'\1', t, flags=re.S)
        extra = select(['on-off'], lit_cond(nits, False)) + select(['on-off-lit'], lit_cond(nits)) + P.lit(['on-off-lit'], nits, '%g %g %g' % WARM, 1.0)
        if n == 'instr':
            extra += select(['base', 'on-off', 'on-off-lit'], '<property>%s</property>' % FIT)
        t = t.replace('</PropertyList>', extra + '\n</PropertyList>')
        open(p, 'w', errors='replace').write(t)
    return '<model><path>Instruments/switch/instr-lights.xml</path><offsets><x-m>0.019</x-m><y-m>%.3f</y-m><z-m>-0.290</z-m><pitch-deg>-21</pitch-deg></offsets></model>' % INSTR_Y


def avionics(d):
    """the avionics' key lights: a lit copy of each face, swapped in while on"""
    ac, xp = os.path.join(d, 'x57-panel.ac'), os.path.join(d, 'x57-panel.xml')
    t = open(xp, errors='replace').read()
    if 'gtn-lit' in t:
        return
    duplicate(ac, list(AVIONICS))
    nits = LV + 'avionics-nits'
    extra = select(list(AVIONICS), lit_cond(nits, False)) + select([n + '-lit' for n in AVIONICS], lit_cond(nits)) + P.lit([n + '-lit' for n in AVIONICS], nits, '%g %g %g' % WARM, 0.5)
    open(xp, 'w', errors='replace').write(t.replace('</PropertyList>', extra + '\n</PropertyList>'))


def cb_instr():
    """the INSTR LIGHT breaker's property: unwired breakers pull out at 1"""
    for r, row in enumerate(P.RIGHT_CB):
        for c, (amps, word, prop) in enumerate(row):
            if word == 'INSTR|LIGHT':
                return '/sim/model/x57/cb/r-%d-%d' % (r, c)


def nasal(out):
    lens = lambda cd, area: cd / area
    cockpit_nits = lens(max(COCKPIT_CD.values()), math.pi * 0.0115 ** 2)
    t = '''# The Tecnam's cockpit lights, written by fdm/x57tecnam.py (see there). The X-57 has none: X-57 menu > Tecnam cockpit lights
# fits them. Unfitted, every level stays 0 and nothing of them is drawn. Instrument lights: the INSTR rocker, on DC Converter
# Bus B through the INSTR LIGHT breaker; each part on its dimmer. Cabin light: its switch, the Essential Bus. Emergency light:
# its switch, on its lamp, the Essential battery itself. Levels 0-1 in LV for the HDR lights, cd/m2 for what glows.
var LV = "%s";
var C = "%s";
var FIT = "/%s";
var CB = "%s";
var INTEGRAL = %.4f;
var COCKPIT_NITS = %.1f;
var CABIN_NITS = %.1f;
var EMERG_NITS = %.1f;
var clamp01 = func(v) { v == nil ? 0 : (v < 0 ? 0 : (v > 1 ? 1 : v)); };
var volts = func(n) { (getprop("/fdm/jsbsim/elec/" ~ n) or 0) > 10; };

var update = func {
    var fit = getprop(FIT) or 0;
    var instr = fit and volts("dc-b-v") and !(getprop(CB) or 0) and getprop(C ~ "instr-light");
    var dim = func(n) { instr ? clamp01(getprop(C ~ n ~ "-norm")) : 0; };
    setprop(LV ~ "switches-nits", dim("switches") * INTEGRAL);
    setprop(LV ~ "avionics-nits", dim("avionics") * INTEGRAL);
    var c = dim("cockpit");
    setprop(LV ~ "cockpit", c);
    setprop(LV ~ "cockpit-nits", c * COCKPIT_NITS);
    var cabin = (fit and volts("ess-bus-v") and getprop(C ~ "cabin-light")) ? 1 : 0;
    setprop(LV ~ "cabin", cabin);
    setprop(LV ~ "cabin-nits", cabin * CABIN_NITS);
    var em = (fit and volts("ess-batt-v") and getprop(C ~ "emergency-light")) ? 1 : 0;
    setprop(LV ~ "emergency", em);
    setprop(LV ~ "emergency-nits", em * EMERG_NITS);
};
foreach (var p; [FIT, CB, C ~ "instr-light", C ~ "switches-norm", C ~ "avionics-norm", C ~ "cockpit-norm", C ~ "cabin-light", C ~ "emergency-light"])
    setlistener(p, func { update(); }, 0, 0);
var timer = maketimer(0.5, update);      # the buses
timer.start();
''' % (LV, CTL, FIT, cb_instr(), INTEGRAL, cockpit_nits, CABIN_CD / (0.042 * 0.088), EMERG_CD / (5 * math.pi * 0.0024 ** 2))
    with open(os.path.join(out, 'Nasal', 'tecnam.nas'), 'w') as f:
        f.write(t)


def dialog(out):
    """X-57 menu > Tecnam cockpit lights: fit them; the same switches and dimmers as in the cockpit"""
    en = '<enable><property>/%s</property></enable>' % FIT
    def check(r, words, p, extra=en):
        return ('<checkbox><row>%d</row><col>0</col><colspan>3</colspan><halign>left</halign><label>%s</label><property>%s</property><live>true</live>%s'
                '<binding><command>dialog-apply</command></binding></checkbox>' % (r, words, p, extra))
    def slider(r, words, n):
        return ('<text><row>%d</row><col>0</col><halign>left</halign><label>%s</label></text><slider><row>%d</row><col>1</col><pref-width>160</pref-width><min>0</min><max>1</max>'
                '<step>0.05</step><live>true</live><property>%s%s-norm</property>%s<binding><command>dialog-apply</command></binding></slider>' % (r, words, r, CTL, n, en))
    rows = [check(0, "Fit the Tecnam's cockpit lights (the X-57 has none)", '/' + FIT, ''),
            check(1, 'INSTR rocker (LIGHTS row)', CTL + 'instr-light'),
            slider(2, 'Switch lights', 'switches'), slider(3, 'Avionics lights', 'avionics'), slider(4, 'Cockpit lights', 'cockpit'),
            check(5, 'Cabin light', CTL + 'cabin-light'), check(6, 'Emergency light', CTL + 'emergency-light')]
    d = os.path.join(out, 'gui', 'dialogs')
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, 'x57-tecnam-lights.xml'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<!-- written by fdm/x57tecnam.py -->\n<PropertyList><name>x57-tecnam-lights</name><layout>vbox</layout>'
                "<text><label>The Tecnam's cockpit lights</label></text><hrule/>"
                '<text><label>NASA flew the X-57 by day and collared its instrument lights out.</label></text>'
                '<text><label>Fitted: dimmers low on the left panel, INSTR in the LIGHTS row,</label></text>'
                '<text><label>the cabin and emergency lights\' switches on their lamps in the ceiling.</label></text><hrule/>'
                '<group><layout>table</layout>%s</group><hrule/>'
                '<button><legend>Close</legend><default>true</default><key>Esc</key><binding><command>dialog-close</command></binding></button></PropertyList>\n' % ''.join(rows))


def install(out):
    base = os.path.join(out, 'Models', 'Interior', 'Panel')
    d = os.path.join(base, 'X57')
    p = os.path.join(base, 'panel.xml')
    t = open(p, errors='replace').read()
    if 'X57/tecnam.xml' not in t:
        add = [lamps(out, d), dimmers(base), rockers(base)]
        t = t.replace('</PropertyList>', '\n'.join(add) + '\n</PropertyList>')
        open(p, 'w', errors='replace').write(t)
    avionics(d)
    nasal(out)
    dialog(out)
    print('  Tecnam cockpit lights: %.2f and %.2f cd floods, fitted from the menu' % (COCKPIT_CD['l'], COCKPIT_CD['r']))
