"""The pilot and the rudder pedals in FlightGear, from mkpilot.py's output (data/pilot) and x57pedals.py.

The pilot replaces the Tecnam's (an X-Plane collection's figure, seated by offsets). He is in the interior's frame, cut at his joints:
- his left hand rides the yoke, with the yoke's own slide and turn, and his right hand the torque levers, turned by their mean
  angle (Nasal/pilot.nas); his feet ride the pedals, turned with them about their floor tube;
- his arms and legs follow by forward kinematics: each upper part turns about its root joint and each lower part, nested in it,
  about its own, every swing as two turns about fixed axes square to the bone. mkpilot.py solved them over the controls'
  travel: the left arm's angles as polynomials in the yoke's two inputs (within 1.6 cm of the solved wrist anywhere), the right
  arm's and the legs' as tables of their one input. Forearm twist is left out: a sleeve's twist shows little;
- his head nods, turns and leans with the stabilator, rudder and ailerons, as the Tecnam's pilot did;
- he is hidden in the Cockpit View, as the Tecnam's was.

The pedals: x57pedals.py's geometry, the pilot's only (NASA removed the copilot's with his seat and yoke), each pedal
two arms from the floor tube to a bar, a toe brake above the bar, a tan master cylinder and its braided line. They swing with
controls/flight/rudder, the toe brakes tilt with controls/gear/brake-left and brake-right.
"""
import json, math, os, re, shutil
import numpy as np

import mkmodel as M
import x57cabin as C
import x57panel as P
import x57pedals as PD

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, 'data', 'pilot')
HEAD_MOVES = (('controls/flight/elevator', -15, (0, 1, 0)), ('controls/flight/rudder', -25, (0, 0, 1)), ('surface-positions/left-aileron-pos-norm', -10, (1, 0, 0)))
LEVER_TABLE = ((-0.174, 0.0), (0.0, 6.0), (1.0, 40.0), (1.66, 50.0))      # pedestal.xml's knobs


def names(objs):
    return ''.join('<object-name>%s</object-name>' % n for n in objs)


def vec(tag, v):
    return '<%s><x-m>%.5f</x-m><y-m>%.5f</y-m><z-m>%.5f</z-m></%s>' % ((tag,) + tuple(v) + (tag,))


def axis(v):
    return '<axis><x>%.6f</x><y>%.6f</y><z>%.6f</z></axis>' % tuple(v)


def interp(rows):
    return '<interpolation>%s</interpolation>' % ''.join('<entry><ind>%g</ind><dep>%g</dep></entry>' % tuple(r) for r in rows)


def rotate(obj, ax, centre, value):
    return '<animation><type>rotate</type><object-name>%s</object-name>%s%s%s</animation>' % (obj, value, vec('center', centre), axis(ax))


def poly(c, inputs):
    """a polynomial in two clipped inputs, as an expression; the terms in mkpilot.py's order"""
    terms = ((0, 0), (1, 0), (0, 1), (2, 0), (1, 1), (0, 2), (2, 1), (1, 2), (3, 0), (0, 3), (2, 2), (3, 1), (1, 3))
    if len(c) != len(terms):
        raise SystemExit('x57pilot: a polynomial of %d terms, not %d: mkpilot.py and x57pilot.py disagree' % (len(c), len(terms)))
    x = ['<clip><clipMin>-1</clipMin><clipMax>1</clipMax><property>%s</property></clip>' % p for p in inputs]
    parts = []
    for k, (i, j) in zip(c, terms):
        if abs(k) < 1e-6:
            continue
        parts.append('<product><value>%.6g</value>%s</product>' % (k, x[0] * i + x[1] * j))
    return '<expression><sum>%s</sum></expression>' % ''.join(parts)


def pilot_xml(J):
    ob, jt, tb = J['objects'], J['joints'], J['tables']
    seg = lambda *ks: [n for k in ks for n in ob.get(k, [])]
    gl = [n for n in ob['head'] if '.gl-' in n]
    x = ['<path>pilot.ac</path>',
         M.effect('x57-face', [n for n in seg(*[k for k in ob if k != 'head']) if '.belt-' not in n] + [n for n in ob['head'] if '.hs-' not in n and '.gl-' not in n]),
         M.effect('x57-webbing', [n for n in seg('torso') if re.search(r'\.belt-(lap|shoulder|stalk)$', n)]),
         # the webbing, 2.5 mm thick, would shadow itself in squares the size of the shadow map's texels: it casts none
         '<animation><type>noshadow</type>%s</animation>' % ''.join('<object-name>%s</object-name>' % n for n in seg('torso') if re.search(r'\.belt-(lap|shoulder|stalk)$', n)),
         M.effect('x57-steel', [n for n in seg('torso') if re.search(r'\.belt-(tongue|fit-)', n)]),
         M.effect('x57-case', [n for n in seg('torso') if n.endswith('.belt-buckle')]),
         M.effect('x57-red', [n for n in seg('torso') if n.endswith('.belt-button')]),
         M.effect('x57-case', [n for n in ob['head'] if '.hs-' in n]),
         M.effect('x57-gold', [n for n in gl if not re.search(r'\.gl-(lens|pad-|sleeve)', n)]),
         M.effect('x57-shades', [n for n in gl if '.gl-lens' in n]),
         M.effect('x57-clear', [n for n in gl if re.search(r'\.gl-(pad-|sleeve)', n)])]
    groups = []
    for side in ('l', 'r'):
        groups += [('pilot-arm-' + side, seg('arm-u-' + side, 'arm-f-' + side)), ('pilot-fore-' + side, seg('arm-f-' + side)),
                   ('pilot-hand-' + side, seg('hand-' + side)), ('pilot-leg-' + side, seg('leg-t-' + side, 'leg-c-' + side)),
                   ('pilot-calf-' + side, seg('leg-c-' + side)), ('pilot-foot-' + side, seg('foot-' + side))]
    groups.append(('pilot-head', seg('head')))
    for name, objs in groups:
        if objs:
            x.append('<animation><name>%s</name>%s</animation>' % (name, names(objs)))
    # the hands and feet on their controls
    y = J['yoke']
    x.append('<animation><type>translate</type><object-name>pilot-hand-l</object-name><property>fdm/jsbsim/x57/stab-norm</property>%s%s</animation>'
             % (interp(y['slide']), axis((1, 0, 0))))
    x.append(rotate('pilot-hand-l', (1, 0, 0), y['origin'], '<property>surface-positions/left-aileron-pos-norm</property>' + interp(y['turn'])))
    x.append(rotate('pilot-hand-r', (0, -1, 0), J['lever']['pivot'], '<property>/sim/model/x57/pilot/lever-deg</property><factor>1</factor>'))
    for side, ped in (('l', 'pl'), ('r', 'pr')):
        yy = PD.PILOT_Y + (-1 if side == 'l' else 1) * PD.SPREAD
        x.append(rotate('pilot-foot-' + side, (0, -1, 0), (PD.HINGE_X, yy, PD.HINGE_Z),
                        '<property>controls/flight/rudder</property>' + interp([(r, PD.swing(ped, r)) for r in (-1, 0, 1)])))
    # the arms and legs: the upper part about its root, the lower inside it about its own
    for grp, key, joint in (('pilot-arm-l', 'arm-l', 'shoulder_l'), ('pilot-fore-l', 'fore-l', 'elbow_l')):
        t = tb[key]
        for ax, c in zip(t['axes'], t['poly']):
            x.append(rotate(grp, ax, jt[joint], poly(c, t['inputs'])))
    for grp, key, joint in (('pilot-arm-r', 'arm-r', 'shoulder_r'), ('pilot-fore-r', 'fore-r', 'elbow_r'),
                            ('pilot-leg-l', 'leg-l', 'hip_l'), ('pilot-calf-l', 'calf-l', 'knee_l'),
                            ('pilot-leg-r', 'leg-r', 'hip_r'), ('pilot-calf-r', 'calf-r', 'knee_r')):
        t = tb[key]
        for ax, rows in zip(t['axes'], t['tables']):
            x.append(rotate(grp, ax, jt[joint], '<property>%s</property>%s' % (t['input'], interp(rows))))
    for prop, f, ax in HEAD_MOVES:
        x.append(rotate('pilot-head', ax, jt['neck'], '<property>%s</property><factor>%g</factor>' % (prop, f)))
    return ('<?xml version="1.0"?>\n<!-- the X-57\'s pilot: written by fdm/x57pilot.py from fdm/mkpilot.py\'s data/pilot (see there) -->\n'
            '<PropertyList>\n%s\n</PropertyList>\n' % '\n'.join(x))


ALL_PILOTS = '''<?xml version="1.0"?>
<!-- written by fdm/x57pilot.py: the pilot, in the interior's frame; hidden from his own eyes -->
<PropertyList>
  <model><name>ThePilot</name><path>pilot.xml</path></model>
  <animation><type>select</type><object-name>ThePilot</object-name>
    <condition><not-equals><property>sim/current-view/name</property><value>Cockpit View</value></not-equals></condition></animation>
</PropertyList>
'''

README = '''The X-57's pilot, made for this aircraft by fdm/mkpilot.py (Blender 4.3, MPFB 2) and fdm/x57pilot.py.

Sources, all CC0: MPFB's base mesh, explicitly released as CC0 in September 2020 (Data Collection AB, Joel Palmius, Jonas
Hauquier), its game-engine rig and weights, and the MakeHuman system assets' middle-aged skin, low-poly brown eyes and black
black sock of shoes03. The flight suit, its pockets and zipper, the painted hair and eyebrows, the headset, the sunglasses, the
seatbelt, and the work shoes, a photogrammetry scan of a pair of black work shoes by this aircraft's author (fdm/mkshoes.py), are
this aircraft's, under its licence.
'''


# ------------------------------------------------------------------------------------------------- the pedals
def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def plate(name, c, u, v, w, size, tex):
    """a box on axes u, v, w (unit): centre c, size along each"""
    c = np.asarray(c, float)
    h = [np.asarray(a, float) * s / 2 for a, s in zip((u, v, w), size)]
    pts = [tuple(c + sa * h[0] + sb * h[1] + sc * h[2]) for sa in (-1, 1) for sb in (-1, 1) for sc in (-1, 1)]
    quads = ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))
    s = [P.surf(q, ((0, 0), (1, 0), (1, 1), (0, 1)), flag='0x10') for q in quads]
    return P.obj(name, pts, s, tex)


def pedals(d):
    """the pilot's two pedals and their floor tube, interior frame; each pedal a group that swings, its toe brake inside it"""
    C.plain_texture(os.path.join(d, 'ped-black.png'), (20, 20, 22), 3)
    C.plain_texture(os.path.join(d, 'ped-tan.png'), (176, 146, 96), 4)
    C.plain_texture(os.path.join(d, 'ped-steel.png'), (176, 180, 186), 4)
    objs, xml, look = [], [], {}

    def add(o, eff):
        objs.append(o)
        look.setdefault(eff, []).append(re.search(r'name "([^"]+)"', o).group(1))
        return re.search(r'name "([^"]+)"', o).group(1)

    hx, hz = PD.HINGE_X, PD.HINGE_Z
    bx, bz = PD.bar()
    up = unit((bx - hx, 0, bz - hz))                   # along the arms, hinge to bar
    nrm = np.cross(up, (0, 1, 0))                      # the pedals' face, toward the pilot
    nrm = nrm if nrm[0] > 0 else -nrm
    for seat, yc in (('p', PD.PILOT_Y),):
        y0, y1 = yc - PD.SPREAD - 0.075, yc + PD.SPREAD + 0.075
        add(C.tube('ped-floor-' + seat, [(hx, y0, hz), (hx, y1, hz)], 0.011, 'ped-black.png', 12, 1.0, 0.05, cap=True), 'x57-case')
        for yy in (y0 + 0.012, y1 - 0.012):
            add(C.cuboid('ped-foot-%s-%d' % (seat, yy > yc), (hx + 0.005, yy, (hz + C.FLOOR) / 2 - 0.004), (0.05, 0.016, hz - C.FLOOR + 0.03), 'ped-black.png'), 'x57-case')
    for name, yc in PD.pedals():
        mine, toe = [], []
        for dy in (-0.032, 0.032):
            mine.append(add(C.tube('ped-%s-arm-%d' % (name, dy > 0), [(hx, yc + dy, hz), (bx, yc + dy, bz)], 0.0075, 'ped-black.png', 8, 1.0, 0.05), 'x57-case'))
        mine.append(add(C.tube('ped-%s-bar' % name, [(bx, yc - PD.WIDTH / 2, bz), (bx, yc + PD.WIDTH / 2, bz)], 0.011, 'ped-black.png', 12, 1.0, 0.05, cap=True), 'x57-case'))
        out = 1 if yc > PD.PILOT_Y else -1
        side = yc + out * (PD.WIDTH / 2 + 0.018)
        c0 = np.array((hx, side, hz)) + up * 0.035
        c1 = np.array((hx, side, hz)) + up * 0.125
        mine.append(add(C.tube('ped-%s-cyl' % name, [tuple(c0), tuple(c1)], 0.012, 'ped-tan.png', 12, 1.0, 0.05, cap=True), 'x57-face'))
        mine.append(add(C.tube('ped-%s-rod' % name, [tuple(c1), tuple(np.array((bx, side, bz)) - up * 0.02)], 0.004, 'ped-steel.png', 8, 1.0, 0.05), 'x57-steel'))
        line = [tuple(c1 + nrm * 0.0), tuple(c1 + up * 0.02 + nrm * 0.03), tuple(np.array((hx + 0.08, side, C.FLOOR + 0.06))), (hx + 0.12, side, C.FLOOR + 0.004)]
        mine.append(add(C.tube('ped-%s-line' % name, C.bezier(line, 12), 0.004, 'kit-braid.png', 6, 1.0, 0.03), 'x57-face'))
        tc = np.array((bx, yc, bz)) + up * (PD.TOE / 2 + 0.006) + nrm * 0.004
        toe.append(add(plate('ped-%s-toe' % name, tc, up, (0, 1, 0), nrm, (PD.TOE, PD.WIDTH, 0.008), 'ped-black.png'), 'x57-case'))
        xml.append('<animation><name>pedal-%s</name>%s</animation>' % (name, names(mine + toe)))
        xml.append('<animation><name>toe-%s</name>%s</animation>' % (name, names(toe)))
        xml.append(rotate('pedal-' + name, (0, -1, 0), (hx, yc, hz), '<property>controls/flight/rudder</property>' + interp([(r, PD.swing(name, r)) for r in (-1, 0, 1)])))
        brake = 'controls/gear/brake-left' if name.endswith('l') else 'controls/gear/brake-right'
        xml.append(rotate('toe-' + name, (0, -1, 0), (bx, yc, bz), '<property>%s</property><factor>%g</factor>' % (brake, PD.BRAKE_DEG)))
    with open(os.path.join(d, 'pedals.ac'), 'w') as f:
        f.write(M.ac(objs))
    xml = [M.effect(e, ns) for e, ns in look.items()] + xml
    with open(os.path.join(d, 'pedals.xml'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<!-- the rudder pedals, written by fdm/x57pilot.py (see there and x57pedals.py) -->\n<PropertyList>\n<path>pedals.ac</path>\n%s\n</PropertyList>\n' % '\n'.join(xml))
    x = os.path.join(d, 'interior.xml')
    t = open(x, errors='replace').read()
    if 'pedals.xml' not in t:
        t = t.replace('</PropertyList>', '<model><path>pedals.xml</path></model>\n</PropertyList>')
        open(x, 'w', errors='replace').write(t)
    return len(objs)


def nasal(out):
    rows = ', '.join('[%g, %g]' % r for r in LEVER_TABLE)
    s = '''# The pilot's right hand rides the two torque levers' mean angle (pedestal.xml's table): written by fdm/x57pilot.py
var TABLE = [%s];
var deg = func(x) {
    if (x <= TABLE[0][0]) return TABLE[0][1];
    for (var i = 1; i < size(TABLE); i += 1)
        if (x <= TABLE[i][0])
            return TABLE[i - 1][1] + (x - TABLE[i - 1][0]) / (TABLE[i][0] - TABLE[i - 1][0]) * (TABLE[i][1] - TABLE[i - 1][1]);
    return TABLE[size(TABLE) - 1][1];
}
var node = props.globals.getNode("/sim/model/x57/pilot/lever-deg", 1);
node.setDoubleValue(0);
var timer = maketimer(0, func {
    node.setDoubleValue((deg(getprop("fdm/jsbsim/prop/l/lever") or 0) + deg(getprop("fdm/jsbsim/prop/r/lever") or 0)) / 2);
});
timer.start();
''' % rows
    os.makedirs(os.path.join(out, 'Nasal'), exist_ok=True)
    open(os.path.join(out, 'Nasal', 'pilot.nas'), 'w').write(s)


def install(out):
    J = json.load(open(os.path.join(DATA, 'pilot.json')))
    d = os.path.join(out, 'Models', 'Pilot')
    for n in os.listdir(d):                                     # the Tecnam's pilot and copilot go
        os.remove(os.path.join(d, n))
    for n in os.listdir(DATA):
        if n.endswith(('.ac', '.png')) and n != 'shoes.png':                 # mkshoes.py's atlas: mkpilot.py copies it as pilot-boots.png
            shutil.copy(os.path.join(DATA, n), os.path.join(d, n))
    open(os.path.join(d, 'pilot.xml'), 'w').write(pilot_xml(J))
    open(os.path.join(d, 'all-pilots.xml'), 'w').write(ALL_PILOTS)
    open(os.path.join(d, 'README.txt'), 'w').write(README)
    n = pedals(os.path.join(out, 'Models', 'Interior'))
    nasal(out)
    print('  pilot: %d faces in %d parts, his left wrist within %.1f cm of the yoke; pedals: %d parts' % (
        J['faces'], sum(len(v) for v in J['objects'].values()), J['tables']['left_wrist_gap_cm'], n))
