#!/usr/bin/env python3
"""The FlightGear aircraft: the flight models, the set files, and the 3-D model.

    python3 mkfg.py            # writes ../X-57

The wing, nacelles and propellers are lofted here from NASA's numbers, in pieces that ride on the bending mode; the rest is the P2006T's (mkmodel.py).
Its frame is JSBSim's stations, x aft, y right, z up, from the VRP; AC3D keeps y up, and its z is the model's -y.
"""
import json, math, os, subprocess, sys
import numpy as np
import vsp
import mkmodel as MM
import x57lights as LT
import x57tecnam
import x57cabin
import x57pilot
import x57boom
IN_ = 0.0254
import x57flex as FX

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'X-57')
GEO = json.load(open(os.path.join(HERE, 'data', 'geometry.json')))
IN = 0.0254


class Mesh:
    def __init__(s, name, mat=0):
        s.name, s.mat, s.v, s.f = name, mat, [], []

    def loft(s, rings, closed=True, caps=True):
        """rings: sections of the same number of points, in inches. Quads between them, and an end cap on each"""
        n = len(rings[0])
        base = len(s.v)
        for r in rings:
            s.v.extend(tuple(p) for p in r)
        for i in range(len(rings) - 1):
            for j in range(n if closed else n - 1):
                a, b = base + i * n + j, base + i * n + (j + 1) % n
                s.f.append((a, b, b + n, a + n))
        if caps:
            s.f.append(tuple(base + j for j in range(n - 1, -1, -1)))
            s.f.append(tuple(base + (len(rings) - 1) * n + j for j in range(n)))

    def ac(s):
        """AC3D object text, in metres"""
        v = ['%.4f %.4f %.4f' % (p[0] * IN, p[2] * IN, -p[1] * IN) for p in s.v]
        out = ['OBJECT poly', 'name "%s"' % s.name, 'numvert %d' % len(v)] + v + ['numsurf %d' % len(s.f)]
        for f in s.f:
            out += ['SURF 0x10', 'mat %d' % s.mat, 'refs %d' % len(f)] + ['%d 0 0' % i for i in f]
        return '\n'.join(out + ['kids 0'])


def section(path):
    a = np.loadtxt(path, skiprows=1)
    i = np.linspace(0, len(a) - 1, 26).astype(int)
    return a[i]


def wing(m, name, sect, y0, y1, c0, c1, x0, x1, z, twist0=0.0, twist1=0.0):
    """a tapered panel from y0 to y1 at leading edges x0, x1"""
    rings = []
    for y, c, x, tw in ((y0, c0, x0, twist0), (y1, c1, x1, twist1)):
        t = math.radians(tw)
        rings.append([(x + c * (p[0] * math.cos(t) + p[1] * math.sin(t)), y, z + c * (p[1] * math.cos(t) - p[0] * math.sin(t))) for p in sect])
    m.loft(rings[::-1] if y1 > y0 else rings)        # always tipward to the left: mirrored, the faces would turn inward and FlightGear wouldn't draw them from outside


def body(m, stations, n=18):
    """a body of ellipses: stations (x, z centre, half width, half height), inches"""
    rings = []
    for x, z, w, h in stations:
        rings.append([(x, w * math.cos(2 * math.pi * k / n), z + h * math.sin(2 * math.pi * k / n)) for k in range(n)])
    m.loft(rings)


def prop(m, hub, d, blades, tbl, sense, r_hub, spinner, band=None):
    """a propeller on NASA's blade: chord and pitch along the radius, a lens section, and the spinner it grows from, on a mesh of its own, as the
    blades' painted band if there is one. Hub at (x, y, z), x aft"""
    R = d / 2
    rr = np.array(tbl['r_R'], float)
    cc = np.array(tbl['chord_R'], float)
    bb = np.array(tbl['beta_deg'], float)
    sec = ((-0.5, 0.0), (-0.28, 0.07), (0.05, 0.085), (0.5, 0.0), (0.05, -0.03), (-0.28, -0.04))
    st = np.r_[0.7 * r_hub / R, np.linspace(max(rr[0], 1.2 * r_hub / R), 0.985, 9)]
    cut = (0.86, 0.95)
    if band is not None:
        st = np.unique(np.r_[st, cut])
    for k in range(blades):
        a = 2 * math.pi * k / blades
        ey, ez = math.cos(a), math.sin(a)                   # along the blade
        ty, tz = -sense * ez, sense * ey                    # the way it turns
        rings = []
        for s in st:
            c_, be = float(np.interp(s, rr, cc)) * R, math.radians(float(np.interp(s, rr, bb)))
            if s < rr[0]:
                c_ = 0.6 * c_
            ring = []
            for u, t in sec:
                du, dt = u * c_ * math.cos(be) - t * c_ * math.sin(be), u * c_ * math.sin(be) + t * c_ * math.cos(be)
                ring.append((hub[0] - dt + 0.0, hub[1] + s * R * ey + du * ty, hub[2] + s * R * ez + du * tz))
            rings.append(ring)
        if band is None:
            m.loft(rings)
        else:
            a, b = (int(np.searchsorted(st, c_)) for c_ in cut)
            m.loft(rings[:a + 1])
            band.loft(rings[a:b + 1])
            m.loft(rings[b:])
    i = len(spinner.v)
    body(spinner, [(hub[0] - 3.0 * r_hub, hub[2], 0.08 * r_hub, 0.08 * r_hub), (hub[0] - 2.4 * r_hub, hub[2], 0.55 * r_hub, 0.55 * r_hub), (hub[0] - 1.4 * r_hub, hub[2], 0.86 * r_hub, 0.86 * r_hub), (hub[0] - 0.4 * r_hub, hub[2], r_hub, r_hub), (hub[0] + 0.6 * r_hub, hub[2], r_hub, r_hub)], 16)
    spinner.v[i:] = [(q[0], q[1] + hub[1], q[2]) for q in spinner.v[i:]]


def fuselage_stations(v):
    """OpenVSP's own eight cross sections, by position, width and height"""
    c = v.first('Fuselage')
    L = c.group('Design')['Length']
    out = []
    for xs in c.g.iter('XSec'):
        pc = xs.find('ParmContainer')
        if pc is None or pc.find('XSec') is None:
            continue
        g = {e.tag: float(e.attrib['Value']) for e in pc.find('XSec') if 'Value' in e.attrib}
        size = {}
        for e in xs.iter():
            if 'Value' in e.attrib and (e.tag.endswith('Width') or e.tag.endswith('Height')):
                size.setdefault(e.tag.split('_')[-1], float(e.attrib['Value']))
        out.append((c.loc[0] + g['XLocPercent'] * L, c.loc[2] + g['ZLocPercent'] * L, size.get('Width', 0.0) / 2, size.get('Height', 0.0) / 2))
    return out


def model():
    v = vsp.Model(os.path.join(HERE, '..', 'refs', 'geometry', 'X-57.vsp3'))
    sec = section(os.path.join(HERE, 'data', 'sections', 'wing_root.dat'))
    w = GEO['wing']
    p = w['panels'][0]
    x0, z0 = GEO['reference']['wing_le_in'][0], GEO['reference']['wing_le_in'][2]
    x1 = x0 + p['y1'] * math.tan(math.radians(p['sweep'])) + p['loc'] * (p['c0'] - p['c1'])
    parts = []
    wp = MM.wing_parts(Mesh, wing, sec, p, x0, x1, z0)
    parts += [q[0] for q in wp]
    hub = GEO['cruise']['hub_in']
    nac = []
    for sgn, sd in ((1, 'r'), (-1, 'l')):
        m = Mesh('nacelle-' + sd)
        body(m, [(hub[0] + 2.4, hub[2], 4.0, 4.0), (hub[0] + 12, hub[2], 8, 8), (hub[0] + 40, hub[2], 7, 7), (hub[0] + 62, hub[2], 1, 1)], 16)
        m.v = [(q[0], q[1] + sgn * hub[1], q[2]) for q in m.v]
        parts.append(m)
        nac.append((m.name, sgn * hub[1], hub[2]))
    for j, hp in enumerate(GEO['high_lift']['hubs_in']):
        for sgn, sd in ((1, 'r'), (-1, 'l')):
            m = Mesh('hl-nacelle-%s-%d' % (sd, j))
            body(m, [(hp[0] + 1.2, hp[2], 1.9, 1.9), (hp[0] + 5, hp[2], 3.2, 3.2), (hp[0] + 22, hp[2], 2.6, 2.6), (hp[0] + 34, hp[2], 0.6, 0.6)], 12)
            m.v = [(q[0], q[1] + sgn * hp[1], q[2]) for q in m.v]
            parts.append(m)
            nac.append((m.name, sgn * hp[1], hp[2]))
    props = []
    for s, sgn in (('left', -1), ('right', 1)):
        m, sp, bd, rg = (Mesh('cruise-%s-%s' % (k, s), 1) for k in ('prop', 'spin', 'band', 'ring'))
        prop(m, (hub[0], sgn * hub[1], hub[2]), 60.0, 3, GEO['cruise']['blade'], -sgn, 4.0, sp, bd)     # sense: +1 the disc's right side rises, as a positive spin about +x
        body(rg, [(hub[0] + 2.2, hub[2], 4.12, 4.12), (hub[0] + 4.4, hub[2], 4.9, 4.9)], 16)     # the red ring round the spinner
        rg.v = [(q[0], q[1] + sgn * hub[1], q[2]) for q in rg.v]
        props += [m, sp, bd, rg]
        for k, x_, r_ in (('nav', LT.TIP_X, 0.75), ('strobe', LT.TIP_X - 0.16, 0.75)):       # the lenses on the nacelle's outboard side
            ln = Mesh('lens-%s-%s' % (k, s), 1)
            body(ln, [(x_ / IN_ - r_, hub[2], 0.05, 0.05), (x_ / IN_ - 0.55 * r_, hub[2], 0.7 * r_, 0.7 * r_), (x_ / IN_ + 0.55 * r_, hub[2], 0.7 * r_, 0.7 * r_), (x_ / IN_ + r_, hub[2], 0.05, 0.05)], 10)
            ln.v = [(q[0], q[1] + sgn * (LT.TIP_Y / IN_ - 0.5), q[2]) for q in ln.v]
            props.append(ln)
    for nm_, (x_, y_, z_), r_ in (('landing', LT.LAMPS['landing'], 0.75), ('taxi', LT.LAMPS['taxi'], 0.75), ('tail', (8.95, 0.0, 1.76), 0.7), ('fin', (8.95, 0.0, 3.41), 0.7)):
        ln = Mesh('lens-' + nm_, 1)
        body(ln, [(x_ / IN_ - 0.5 * r_, z_ / IN_, 0.05, 0.05), (x_ / IN_ - 0.3 * r_, z_ / IN_, 0.7 * r_, 0.7 * r_), (x_ / IN_ + 0.3 * r_, z_ / IN_, r_, r_), (x_ / IN_ + 0.5 * r_, z_ / IN_, r_, r_)], 12)
        ln.v = [(q[0], q[1] + y_ / IN_, q[2]) for q in ln.v]
        props.append(ln)
    # high-lift discs 1 to 6 outboard to inboard on the left, 7 to 12 inboard to outboard on the right
    hubs = GEO['high_lift']['hubs_in']
    order = [(-q[1], q) for q in reversed(hubs)] + [(q[1], q) for q in hubs]
    for i, (y, q) in enumerate(order, 1):
        m, sp = Mesh('hl-prop-%d' % i, 1), Mesh('hl-spin-%d' % i, 1)
        prop(m, (q[0], y, q[2]), 22.674, 5, GEO['high_lift']['blade'], 1 if i <= 6 else -1, 1.9, sp)
        props += [m, sp]
        m, sp = Mesh('hl-fold-%d' % i, 1), Mesh('hl-foldspin-%d' % i, 1)       # stopped, the blades lie back along the nacelle, on the spinner
        body(sp, [(q[0] - 5.7, q[2], 0.15, 0.15), (q[0] - 4.5, q[2], 1.05, 1.05), (q[0] - 2.6, q[2], 1.65, 1.65), (q[0] - 0.8, q[2], 1.9, 1.9), (q[0] + 1.2, q[2], 1.9, 1.9)], 16)
        sp.v = [(p_[0], p_[1] + y, p_[2]) for p_ in sp.v]
        props.append(sp)
        for k in range(5):
            a = 2 * math.pi * k / 5
            ey, ez = math.cos(a), math.sin(a)
            ty, tz = -ez, ey
            rings = []
            for x_, rad, w_, t_ in ((q[0] - 0.5, 2.0, 0.55, 0.20), (q[0] + 3.5, 3.45, 0.85, 0.18), (q[0] + 8.0, 3.5, 0.72, 0.15), (q[0] + 11.6, 3.35, 0.32, 0.08)):
                rings.append([(x_, y + rad * ey + du * w_ * ty + dt * t_ * ey, q[2] + rad * ez + du * w_ * tz + dt * t_ * ez) for du, dt in ((-1, 0), (0, 1), (1, 0), (0, -1))])
            m.loft(rings)
        props.append(m)
    parts += props
    text = ['AC3Db', 'MATERIAL "skin" rgb 0.9 0.91 0.93 amb 0.5 0.5 0.5 emis 0 0 0 spec 0.3 0.3 0.3 shi 20 trans 0',
            'MATERIAL "blade" rgb 0.15 0.15 0.17 amb 0.2 0.2 0.2 emis 0 0 0 spec 0.1 0.1 0.1 shi 10 trans 0',
            'OBJECT world', 'kids %d' % len(parts)] + [q.ac() for q in parts]
    return '\n'.join(text) + '\n', hub, order, wp, nac, z0


def model_xml(hub, order, wp, nac, z0):
    """the wing rides on the bending mode; propellers on the properties the sound reads. Outer moves first: a later animation works inside an earlier one"""
    D = FX.modal(False)
    M = IN
    an = []

    def spin(obj, y, x, z, prop, sign):
        names = ''.join('<object-name>%s</object-name>' % o for o in obj)
        an.append('''<animation><type>spin</type>%s<property>%s</property><factor>%g</factor>
  <center><x-m>%.3f</x-m><y-m>%.3f</y-m><z-m>%.3f</z-m></center><axis><x>1</x><y>0</y><z>0</z></axis></animation>''' % (names, prop, sign, x * M, y * M, z * M))

    for q in wp:
        if q[2] != 'fixed':
            MM.ride(an, q[0].name, q[1], z0, D)
    for n, y, z in nac:
        MM.ride(an, n, y, z, D)
    for s, y in (('left', -hub[1]), ('right', hub[1])):
        MM.ride(an, ['cruise-%s-%s' % (k, s) for k in ('prop', 'spin', 'band', 'ring')], y, hub[2], D)
    for i, (y, q) in enumerate(order, 1):
        MM.ride(an, ['hl-%s-%d' % (k, i) for k in ('prop', 'spin', 'fold', 'foldspin')], y, q[2], D)
    for q in wp:
        if q[2] == 'flap':
            MM.surface(an, q[0].name, 'fdm/jsbsim/ctl/flap-deg', q[3][0], q[3][1], 1)
        elif q[2] == 'ail':
            MM.surface(an, q[0].name, 'fdm/jsbsim/ctl/ail-%s-deg' % ('left' if q[1] < 0 else 'right'), q[3][0], q[3][1], 1)
    for side, sd, y in ((-1, 'left', -hub[1]), (1, 'right', hub[1])):        # the wing tips' lights go with the tips
        MM.ride(an, ['lens-nav-' + sd, 'lens-strobe-' + sd] + ['light-' + l[0] for l in LT.LIGHTS if l[-1] == side], y, hub[2], D)
    for nm_, p_ in LT.LAMPS.items():                                         # and the leading edge's with the wing at their own station
        MM.ride(an, ['lens-' + nm_, 'light-' + nm_], p_[1] / IN_, p_[2] / IN_, D)
    spin(['cruise-%s-left' % k for k in ('prop', 'spin', 'band')], -hub[1], hub[0], hub[2], 'fdm/jsbsim/x57/cruise[0]/rpm', 0.2)
    spin(['cruise-%s-right' % k for k in ('prop', 'spin', 'band')], hub[1], hub[0], hub[2], 'fdm/jsbsim/x57/cruise[1]/rpm', -0.2)
    four = '<equals><property>sim/aero</property><value>x57-mod4</value></equals>'
    for i, (y, q) in enumerate(order, 1):
        p_ = 'fdm/jsbsim/x57/hl[%d]/rpm' % i
        an.append('<animation><type>select</type><object-name>hl-prop-%d</object-name><object-name>hl-spin-%d</object-name><condition><and>%s<greater-than><property>%s</property><value>250</value></greater-than></and></condition></animation>' % (i, i, four, p_))
        an.append('<animation><type>select</type><object-name>hl-fold-%d</object-name><object-name>hl-foldspin-%d</object-name><condition><and>%s<less-than-equals><property>%s</property><value>250</value></less-than-equals></and></condition></animation>' % (i, i, four, p_))
        spin(['hl-prop-%d' % i, 'hl-spin-%d' % i], y, q[0], q[2], p_, 0.2 if i <= 6 else -0.2)
    an.append(MM.effect('x57-paint', [q[0].name for q in wp] + [n for n, _, _ in nac]))
    # colours from NASA's render of Mod IV and the Mod II footage: pale cruise blades with a red band, a red ring, navy spinners, dark high-lift blades
    an.append(MM.effect('x57-cblade', ['cruise-prop-left', 'cruise-prop-right']))
    an.append(MM.effect('x57-blade', ['hl-prop-%d' % i for i in range(1, 13)] + ['hl-fold-%d' % i for i in range(1, 13)]))
    an.append(MM.effect('x57-red', ['cruise-%s-%s' % (k, s) for k in ('band', 'ring') for s in ('left', 'right')] + ['lens-nav-left']))
    an.append(MM.effect('x57-green', ['lens-nav-right']))
    an.append(MM.effect('x57-lens', ['lens-strobe-left', 'lens-strobe-right', 'lens-landing', 'lens-taxi', 'lens-tail', 'lens-fin']))
    an.append(MM.effect('x57-spinner', ['cruise-spin-left', 'cruise-spin-right'] + ['hl-spin-%d' % i for i in range(1, 13)] + ['hl-foldspin-%d' % i for i in range(1, 13)]))
    fus = '<model><path>fuselage.xml</path><offsets><x-m>%.3f</x-m><z-m>%.3f</z-m></offsets></model>' % (MM.X0, MM.Z0)
    return '<?xml version="1.0"?>\n<PropertyList>\n<path>x57.ac</path>\n%s\n%s%s\n</PropertyList>\n' % (fus, LT.models(), '\n'.join(an))


def set_file(mod):
    name = 'Mod IV' if mod == 'mod4' else 'Mod III'
    hl = '''
  <key n="72"><name>H</name><desc>High-lift system: arm or disarm</desc>
    <binding><command>property-toggle</command><property>/sim/model/x57/hl-arm</property></binding></key>
  <key n="89"><name>Y</name><desc>High-lift quick disarm, the yoke switch</desc>
    <binding><command>property-assign</command><property>/sim/model/x57/hl-yoke</property><value>1</value></binding></key>''' if mod == 'mod4' else ''
    regen = ''.join('''
  <key n="%d"><name>%s</name><desc>Regeneration: the throttles run from idle at the top to full regeneration at the bottom, or back to driving the motors</desc>
    <binding><command>nasal</command><script>x57cockpit.regen_toggle();</script></binding></key>''' % k for k in ((127, 'Delete'), (8, 'Backspace, a Mac\'s delete key')))
    return '''<?xml version="1.0"?>
<!-- generated by fdm/mkfg.py -->
<PropertyList>
 <sim>
  <description>X-57 Maxwell %s (JSBSim)</description>
  <long-description>NASA's battery-electric demonstrator, modelled from NASA's public data; not made or endorsed by NASA. A built-up model on NASA's CFD, each propeller with its own slipstream; the P2006T's cabin as NASA stripped it, with the X-57's panel, the instrumentation pallet and the batteries. The X-57 menu holds its systems and the failures NASA studied.</long-description>
  <author>JSBSim model from NASA's public data</author>
  <aircraft-version>0.1</aircraft-version>
  <status>production</status>
  <minimum-fg-version>2024.2.0</minimum-fg-version>
  <compatibility><pbr-model type="bool">true</pbr-model></compatibility>
  <tags><tag>compositor</tag><tag>HDR</tag><tag>electric</tag><tag>experimental</tag></tags>
  <flight-model>jsb</flight-model>
  <aero>x57-%s</aero>
  <model><path>Aircraft/X-57/Models/x57.xml</path><x57><click><rocker type="int">-1</rocker><push type="int">-1</push></click><d175-page type="int">1</d175-page><show-yoke type="bool">true</show-yoke><hl-arm type="bool">false</hl-arm><hl-airspeed type="bool">false</hl-airspeed><hl-yoke type="bool">false</hl-yoke><crew-display type="bool">true</crew-display><dau-enable type="bool">true</dau-enable><rf-enable type="bool">true</rf-enable><record-enable type="bool">true</record-enable><tecnam-lights type="bool">false</tecnam-lights></x57></model>
  <rating><FDM type="int">5</FDM><systems type="int">4</systems><cockpit type="int">4</cockpit><model type="int">4</model></rating>
  <view n="0"><internal archive="y">true</internal><config><x-offset-m archive="y">-0.271</x-offset-m><y-offset-m archive="y">2.085</y-offset-m><z-offset-m archive="y">2.689</z-offset-m><pitch-offset-deg>-14</pitch-offset-deg><default-field-of-view-deg type="double">90</default-field-of-view-deg></config></view>
  <!-- the aft cabin's batteries, their strings, control modules and vent, from above the front seats looking aft (fdm/x57cabin.py); V cycles to it -->
  <view n="100"><name>Battery View</name><enabled type="bool">true</enabled><type>lookfrom</type><internal archive="y">true</internal><config><from-model type="bool">true</from-model><from-model-idx type="int">0</from-model-idx><x-offset-m archive="y">0.0</x-offset-m><y-offset-m archive="y">2.23</y-offset-m><z-offset-m archive="y">3.03</z-offset-m><heading-offset-deg type="double">180</heading-offset-deg><default-heading-deg type="double">180</default-heading-deg><pitch-offset-deg type="double">-37</pitch-offset-deg><default-pitch-deg type="double">-37</default-pitch-deg><default-field-of-view-deg type="double">88</default-field-of-view-deg><ground-level-nearplane-m type="double">0.05</ground-level-nearplane-m><dynamic-view type="bool">false</dynamic-view><limits><enabled type="bool">false</enabled></limits></config></view>
  <!-- the pilot's face, headset and sunglasses, from over the yoke looking back at him (fdm/mkpilot.py); V cycles to it -->
  <view n="101"><name>Pilot Face View</name><enabled type="bool">true</enabled><type>lookfrom</type><internal archive="y">true</internal><config><from-model type="bool">true</from-model><from-model-idx type="int">0</from-model-idx><x-offset-m archive="y">-0.20</x-offset-m><y-offset-m archive="y">2.12</y-offset-m><z-offset-m archive="y">2.37</z-offset-m><heading-offset-deg type="double">169.3</heading-offset-deg><default-heading-deg type="double">169.3</default-heading-deg><pitch-offset-deg type="double">-1</pitch-offset-deg><default-pitch-deg type="double">-1</default-pitch-deg><default-field-of-view-deg type="double">52</default-field-of-view-deg><ground-level-nearplane-m type="double">0.05</ground-level-nearplane-m><dynamic-view type="bool">false</dynamic-view><limits><enabled type="bool">false</enabled></limits></config></view>
  <flaps><setting>0.0</setting><setting>0.3333</setting><setting>0.6667</setting><setting>1.0</setting></flaps>
  <sound><path>Aircraft/X-57/Sounds/x57-%s-sound.xml</path></sound>
  <systems><electrical><path></path></electrical><property-rule n="100"><name>x57-levers</name><path>Aircraft/X-57/Systems/x57-rules.xml</path></property-rule></systems>
  <menubar><default><menu n="100"><label>X-57</label><enabled type="bool">true</enabled><item><label>Autostart</label><binding><command>nasal</command><script>x57cockpit.autostart();</script></binding></item><item><label>Autostop</label><binding><command>nasal</command><script>x57cockpit.autostop();</script></binding></item><item><label>Systems and failures</label><binding><command>dialog-show</command><dialog-name>x57-systems</dialog-name></binding></item><item><label>Tecnam cockpit lights (not on the X-57)</label><binding><command>dialog-show</command><dialog-name>x57-tecnam-lights</dialog-name></binding></item></menu></default></menubar>
 </sim>
 <controls>
  <engines><engine n="0"><throttle>0</throttle><propeller-pitch>0.685</propeller-pitch><reverser type="bool">false</reverser><cutoff type="bool">false</cutoff></engine><engine n="1"><throttle>0</throttle><propeller-pitch>0.685</propeller-pitch><reverser type="bool">false</reverser><cutoff type="bool">false</cutoff></engine></engines>
  <electric><battery-switch type="bool">true</battery-switch></electric>
  <flight><elevator-trim type="double">0.3</elevator-trim></flight>
  <gear><brake-parking>1</brake-parking></gear>
  <lighting><nav-lights type="bool">true</nav-lights><strobe type="bool">true</strobe><landing-lights type="bool">false</landing-lights><taxi-lights type="bool">false</taxi-lights><instr-light type="bool">false</instr-light><switches-norm type="double">0.5</switches-norm><avionics-norm type="double">0.5</avionics-norm><cockpit-norm type="double">0.5</cockpit-norm><cabin-light type="bool">false</cabin-light><emergency-light type="bool">false</emergency-light></lighting>
 </controls>
 <!-- FlightGear's comm radio loops radionoise.wav at (1 - signal quality) x volume unless add-noise is false: the radio works, without the static -->
 <instrumentation><comm n="0"><add-noise type="bool">false</add-noise></comm><comm n="1"><add-noise type="bool">false</add-noise></comm></instrumentation>
 <!-- FlightGear's vacuum pump turns with /engines/engine/rpm, which the electric motors in the flight model don't set: without it the gyros run down. The pumps are held running -->
 <engines><engine n="0"><rpm type="double">2400</rpm></engine><engine n="1"><rpm type="double">2400</rpm></engine></engines>
 <nasal><doors><file>Aircraft/X-57/Nasal/doors.nas</file></doors><x57lights><file>Aircraft/X-57/Nasal/lights.nas</file></x57lights><x57hdr><file>Aircraft/X-57/Nasal/hdr.nas</file></x57hdr><x57displays><file>Aircraft/X-57/Nasal/displays.nas</file></x57displays><x57cockpit><file>Aircraft/X-57/Nasal/cockpit.nas</file></x57cockpit><x57tecnam><file>Aircraft/X-57/Nasal/tecnam.nas</file></x57tecnam><x57pilot><file>Aircraft/X-57/Nasal/pilot.nas</file></x57pilot></nasal>
 <input><keyboard>%s
 </keyboard></input>
</PropertyList>
''' % (name, mod, mod, hl + regen)


def previews(mod):
    """the launcher's pictures, in X-57/Previews: taken in FlightGear, kept in the repository. Its aircraft list takes X-57/thumbnail.jpg, by that name only: one for both Mods"""
    kinds = dict(mod3=('exterior',) * 3 + ('interior',), mod4=('exterior',) * 4 + ('interior',))[mod]
    # every exterior picture is a splash: FlightGear shows one of them at random while it loads (the launcher only steps through them by hand)
    p = ''.join('<preview><type>%s</type>%s<path>Previews/x57-%s-%d.jpg</path></preview>' % (k, '<splash type="bool">true</splash>' if k == 'exterior' else '', mod, i) for i, k in enumerate(kinds))
    return '  <previews>%s</previews>\n' % p


def main():
    os.makedirs(os.path.join(OUT, 'Models'), exist_ok=True)
    for mod in ('mod3', 'mod4'):
        subprocess.run([sys.executable, os.path.join(HERE, 'genx57.py'), OUT, '--' + mod, '--fg', '--built'], check=True)
        with open(os.path.join(OUT, 'x57-%s-set.xml' % mod), 'w') as f:
            f.write(set_file(mod).replace('  <view n="0">', previews(mod) + '  <view n="0">', 1))
    text, hub, order, wp, nac, z0 = model()
    with open(os.path.join(OUT, 'Models', 'x57.ac'), 'w') as f:
        f.write(text)
    with open(os.path.join(OUT, 'Models', 'x57.xml'), 'w') as f:
        f.write(model_xml(hub, order, wp, nac, z0))
    MM.fuselage(OUT)
    x57boom.install(OUT)
    MM.panel(OUT)
    x57tecnam.install(OUT)
    x57cabin.install(OUT)
    x57pilot.install(OUT)
    LT.install(OUT)
    MM.dialog(OUT)
    MM.full_colour(OUT)
    print('wrote', os.path.relpath(OUT, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')))          # relative to the repository: no home folder in a log


if __name__ == '__main__':
    main()
