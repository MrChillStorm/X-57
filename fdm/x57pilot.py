"""The pilot and the rudder pedals in FlightGear, from mkpilot.py's output (data/pilot) and x57pedals.py.

The pilot replaces the Tecnam's (an X-Plane collection's figure, seated by offsets). He is in the interior's frame, cut at his joints:
- his left hand rides the yoke, with the yoke's own slide and turn, and his right hand the torque levers, turned by their mean
  angle (Nasal/pilot.nas); his feet ride the pedals, turned with them about their floor tube;
- his arms and legs follow by forward kinematics: each upper part turns about its root joint and each lower part, nested in it,
  about its own, every swing as two turns about fixed axes square to the bone. mkpilot.py solved them over the controls'
  travel: the left arm's angles as polynomials in the yoke's two inputs (within 1.6 cm of the solved wrist anywhere), the right
  arm's and the legs' as tables of their one input. Forearm twist is left out: a sleeve's twist shows little;
- his head nods, turns and leans with the stabilator, rudder and ailerons, as the Tecnam's pilot did;
- he is hidden from his own eyes, as the Bo 105 hides its pilots: in the Cockpit View, as the Tecnam's was, and from an FGCamera
  cockpit camera (FGCamera1) in his head. His head and headset span x -0.42 to -0.15 m and z 2.65 (the mic boom) to 2.90 m in the
  views' frame, so the box reaches 3 cm past them to the right and 10 cm ahead and behind; a camera ahead of him looking back, as
  the Pilot Face View does from z 2.37, still sees him.

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
<!-- written by fdm/x57pilot.py: the pilot, in the interior's frame; hidden from his own eyes: the Cockpit View, or an FGCamera
     cockpit camera (FGCamera1) in his head (x -0.42 to -0.15 m, z 2.65 to 2.90 m in the views' frame, with a margin) -->
<PropertyList>
  <model><name>ThePilot</name><path>pilot.xml</path></model>
  <animation><type>select</type><object-name>ThePilot</object-name>
    <condition><not><or>
      <equals><property>sim/current-view/name</property><value>Cockpit View</value></equals>
      <and>
        <equals><property>sim/current-view/name</property><value>FGCamera1</value></equals>
        <less-than><property>sim/current-view/x-offset-m</property><value>-0.12</value></less-than>
        <greater-than><property>sim/current-view/z-offset-m</property><value>2.55</value></greater-than>
        <less-than><property>sim/current-view/z-offset-m</property><value>3.0</value></less-than>
      </and>
    </or></not></condition></animation>
</PropertyList>
'''

README = '''The X-57's pilot, made for this aircraft by fdm/mkpilot.py (Blender 4.3, MPFB 2) and fdm/x57pilot.py.

Sources, all CC0: MPFB's base mesh, explicitly released as CC0 in September 2020 (Data Collection AB, Joel Palmius, Jonas
Hauquier), its game-engine rig and weights, and the MakeHuman system assets' middle-aged skin, low-poly brown eyes and black
black sock of shoes03. The flight suit, its pockets and zipper, the painted hair and eyebrows, the headset, the sunglasses, the
seatbelt, and the work shoes, a photogrammetry scan of a pair of black work shoes by this aircraft's author (fdm/mkshoes.py), are
this aircraft's, under its licence. So is the chest patch (fdm/mkpatch.py), its X-57 drawn from NASA's OpenVSP geometry; the FlightGear logo on it
is fgdata's Docs/img/FlightGear_logo.svg, under the GPL as fgdata is, and its lettering DejaVu Sans Bold (Bitstream Vera licence).
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


PATCH_D = 0.095            # the chest patch's diameter, m: a 3.75 in mission patch
PATCH_LIFT = 0.0015        # its stand-off from the suit


def ac_objects(path, wanted):
    """from an AC file in absolute coordinates: each wanted object's vertices, in the model's frame (AC3D's y is up and its z the model's
    -y), and its triangles' vertex and UV indices"""
    t = open(path).read().split('\n')
    out, i, name = {}, 0, None
    while i < len(t):
        l = t[i]
        if l.startswith('name '):
            name = l.split('"')[1]
        elif l.startswith('numvert') and name in wanted:
            n = int(l.split()[1])
            V = np.array([[float(q) for q in t[i + 1 + k].split()] for k in range(n)])[:, [0, 2, 1]] * (1, -1, 1)
            i += n
            tris, uvs = [], []
            while i + 1 < len(t) and not t[i + 1].startswith('kids'):
                i += 1
                if t[i].startswith('refs'):
                    rr = [t[i + 1 + k].split() for k in range(int(t[i].split()[1]))]
                    for k in range(1, len(rr) - 1):
                        q = (rr[0], rr[k], rr[k + 1])
                        tris.append([int(r[0]) for r in q])
                        uvs.append([(float(r[1]), float(r[2])) for r in q])
                    i += len(rr)
            out[name] = dict(V=V, T=np.array(tris), UV=np.array(uvs))
        i += 1
    return out


def chest_patch(ac, body_png):
    """mkpatch.py's patch on his right breast, as an AC object in the torso's static frame. Placed by what mkpilot.py painted: the blank
    leather name tag on his left breast and the suit's zipper, read off the body's texture where each torso face samples it. The patch
    mirrors the tag across the zipper, its bottom level with the tag's, so it clears the right pocket as the tag clears the left. A disc
    of rings and sectors is cast onto the suit from in front, square to the zipper's line, so it lies on his chest's curves"""
    from PIL import Image
    from scipy import ndimage as ndi
    parts = ac_objects(ac, {'torso.body', 'torso.belt-shoulder'})
    body, belt = parts['torso.body'], parts['torso.belt-shoulder']['V']
    V, T, UV = body['V'], body['T'], body['UV']
    tex = np.asarray(Image.open(body_png).convert('RGB')).astype(int)
    H, W = tex.shape[:2]
    P3 = V[T]
    nrm = np.cross(P3[:, 1] - P3[:, 0], P3[:, 2] - P3[:, 0])
    nrm /= np.linalg.norm(nrm, axis=1)[:, None] + 1e-12

    def blobs(c, tol):
        m = ndi.binary_opening(np.abs(tex - c).max(2) < tol)
        lab, n = ndi.label(m)
        return [(int(ndi.sum(m, lab, k + 1)), sl) for k, sl in enumerate(ndi.find_objects(lab))]

    def at(row, col):
        """the torso's point painted at a texel"""
        p = np.array((col / W, 1 - row / H))
        a, b, c = UV[:, 0], UV[:, 1], UV[:, 2]
        den = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
        den = np.where(np.abs(den) < 1e-12, 1e-12, den)
        l1 = ((b[:, 1] - c[:, 1]) * (p[0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (p[1] - c[:, 1])) / den
        l2 = ((c[:, 1] - a[:, 1]) * (p[0] - c[:, 0]) + (a[:, 0] - c[:, 0]) * (p[1] - c[:, 1])) / den
        l3 = 1 - l1 - l2
        m = (l1 >= 0) & (l2 >= 0) & (l3 >= 0) & (nrm[:, 0] < 0)
        if not m.any():
            return None
        f = np.argmax(m)
        return l1[f] * P3[f, 0] + l2[f] * P3[f, 1] + l3[f] * P3[f, 2]
    # the tag: the leather blob about as wide as 9 to 4 is tall; the zipper: the longest thin upright line of its colour
    tags = [(n, sl) for n, sl in blobs((70, 52, 38), 10) if n > 2000 and 1.6 < (sl[1].stop - sl[1].start) / (sl[0].stop - sl[0].start) < 3.0]
    zips = [(n, sl) for n, sl in blobs((58, 60, 52), 6) if (sl[0].stop - sl[0].start) > 8 * (sl[1].stop - sl[1].start)]
    if not tags or not zips:
        raise SystemExit('x57pilot: the name tag or the zipper not found in %s' % body_png)
    sl = max(tags, key=lambda q: q[0])[1]
    T0 = at((sl[0].start + sl[0].stop) / 2, (sl[1].start + sl[1].stop) / 2)
    zs = max(zips, key=lambda q: q[0])[1]
    zcol = (zs[1].start + zs[1].stop) / 2
    zp = np.array([q for q in (at(r, zcol) for r in np.linspace(zs[0].start + 5, zs[0].stop - 5, 40)) if q is not None])
    if T0 is None or len(zp) < 10:
        raise SystemExit('x57pilot: the name tag or the zipper not on the torso')
    a = np.polyfit(zp[:, 2], zp[:, 0], 1)                          # x along z on the zipper: his lean
    up = np.array((a[0], 0.0, 1.0)); up /= np.linalg.norm(up)
    near = np.argsort(np.abs(zp[:, 2] - T0[2]))[:6]
    y_mid = float(np.mean(zp[near, 1]))
    side = np.array((0.0, 1.0, 0.0))                               # his right
    fwd = np.cross(up, side)                                       # out of his chest
    tag_h = 0.04                                                    # as mkpilot.py paints it, 4 by 9 cm
    c0 = np.array((T0[0], 2 * y_mid - T0[1], T0[2])) + up * (PATCH_D / 2 - tag_h / 2)
    e1, e2 = P3[:, 1] - P3[:, 0], P3[:, 2] - P3[:, 0]

    def cast(o, D):
        """Moeller-Trumbore against every torso face: the nearest hit's distance and face, or None"""
        pv = np.cross(D, e2)
        det = (e1 * pv).sum(1)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1 / np.where(ok, det, 1), 0)
        tv = o - P3[:, 0]
        u = (tv * pv).sum(1) * inv
        qv = np.cross(tv, e1)
        v = (qv * D).sum(1) * inv
        t = (e2 * qv).sum(1) * inv
        m = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 0)
        if not m.any():
            return None
        f = int(np.argmin(np.where(m, t, np.inf)))
        return t[f], f
    # his right side ends at the arm's cut: the patch's edge kept a centimetre inside it
    edge = next((d for d in np.arange(0.0, 0.25, 0.002) if cast(c0 + side * d + 0.25 * fwd, -fwd) is None), 0.25)
    c0 = c0 - side * max(0.0, PATCH_D / 2 + 0.01 - edge)
    # the shoulder belt runs from his left shoulder to the buckle by his right hip: the patch goes up till it clears it by 3 mm
    lift = 0.0
    while lift < 0.03 and (np.hypot((belt - c0 - up * lift) @ side, (belt - c0 - up * lift) @ up)[((belt - c0) @ fwd) > -0.03] < PATCH_D / 2 + 0.003).any():
        lift += 0.002
    c0 = c0 + up * lift
    # the disc: image x runs to his left, as one facing him reads it; y up his chest
    rings, sectors = 8, 64
    pts, tuv = [(0.0, 0.0)], [(0.5, 0.5)]
    k = 500.0 / 512.0                                              # the patch's edge on its picture
    for i in range(1, rings + 1):
        rho = i / rings
        for j in range(sectors):
            ph = 2 * math.pi * j / sectors
            pts.append((rho * math.cos(ph), rho * math.sin(ph)))
            tuv.append((0.5 + 0.5 * k * rho * math.cos(ph), 0.5 + 0.5 * k * rho * math.sin(ph)))
    pts = np.array(pts)
    O = c0 + (PATCH_D / 2) * (pts[:, :1] * -side + pts[:, 1:] * up) + 0.25 * fwd
    hit = []
    for o in O:
        h = cast(o, -fwd)
        if h is None:
            raise SystemExit('x57pilot: the chest patch runs off the torso')
        n = nrm[h[1]] if nrm[h[1]] @ fwd > 0 else -nrm[h[1]]
        hit.append(o - h[0] * fwd + PATCH_LIFT * n)
    hit = np.array(hit)
    faces = [(0, 1 + j, 1 + (j + 1) % sectors) for j in range(sectors)]
    for i in range(1, rings):
        a0, b0 = 1 + (i - 1) * sectors, 1 + i * sectors
        for j in range(sectors):
            j1 = (j + 1) % sectors
            faces += [(a0 + j, b0 + j, b0 + j1), (a0 + j, b0 + j1, a0 + j1)]
    fixed = []
    for f in faces:                                                # counter-clockwise seen from in front
        nn = np.cross(hit[f[1]] - hit[f[0]], hit[f[2]] - hit[f[0]])
        fixed.append(f if nn @ fwd > 0 else (f[0], f[2], f[1]))
    s = ['OBJECT poly', 'name "torso.patch"', 'texture "patch.png"', 'crease 60.000000', 'numvert %d' % len(hit)]
    s += ['%.6f %.6f %.6f' % (p[0], p[2], -p[1]) for p in hit]                  # back to AC3D's axes
    s.append('numsurf %d' % len(fixed))
    for f in fixed:
        s += ['SURF 0x10', 'mat 0', 'refs 3'] + ['%d %.6f %.6f' % (q, tuv[q][0], tuv[q][1]) for q in f]
    s.append('kids 0')
    txt = open(ac).read()
    m = re.search(r'(OBJECT world\nname "[^"]*"\nkids )(\d+)', txt)
    txt = txt[:m.start(2)] + str(int(m.group(2)) + 1) + txt[m.end(2):]
    open(ac, 'w').write(txt.rstrip('\n') + '\n' + '\n'.join(s) + '\n')
    lean = math.degrees(math.atan2(up[0], up[2]))
    return 'chest patch %.1f cm on his right breast, %.1f cm right of the zipper, %.1f cm up for the belt, leaning %.0f deg' % (PATCH_D * 100, (c0[1] - y_mid) * 100, lift * 100, lean)


def install(out):
    J = json.load(open(os.path.join(DATA, 'pilot.json')))
    d = os.path.join(out, 'Models', 'Pilot')
    for n in os.listdir(d):                                     # the Tecnam's pilot and copilot go
        os.remove(os.path.join(d, n))
    for n in os.listdir(DATA):
        if n.endswith(('.ac', '.png')) and n != 'shoes.png':                 # mkshoes.py's atlas: mkpilot.py copies it as pilot-boots.png
            shutil.copy(os.path.join(DATA, n), os.path.join(d, n))
    print('  pilot: ' + chest_patch(os.path.join(d, 'pilot.ac'), os.path.join(DATA, 'pilot-body.png')))
    J['objects']['torso'] = J['objects'].get('torso', []) + ['torso.patch']        # the suit's effect
    open(os.path.join(d, 'pilot.xml'), 'w').write(pilot_xml(J))
    open(os.path.join(d, 'all-pilots.xml'), 'w').write(ALL_PILOTS)
    open(os.path.join(d, 'README.txt'), 'w').write(README)
    n = pedals(os.path.join(out, 'Models', 'Interior'))
    nasal(out)
    print('  pilot: %d faces in %d parts, his left wrist within %.1f cm of the yoke; pedals: %d parts' % (
        J['faces'], sum(len(v) for v in J['objects'].values()), J['tables']['left_wrist_gap_cm'], n))
