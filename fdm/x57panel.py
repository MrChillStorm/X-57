"""The X-57's own cockpit: NASA's Cockpit Interface Control Document, ICD-CEPT-006 rev E, first [refs/cockpit-icd], then the photographs [20240010539 fig. 4, 20230005312 fig. 1, 20240007413 fig. 2]
and 20240006533 [refs/power-hmi]. Mod IV never existed as a whole aircraft: its high-lift panel, overhead section and crew panel are the handbook's design. Places come from the photographs on a linear map,
which the Tecnam's six-pack fixes on the left and the right panel's edges on the right: good to a few millimetres.

Placed by me, for want of a source: the LIGHT CHECK button, which stands on a panel of its own in the handbook and has no room beside Mod IV's high-lift panel; the right breaker panel's order,
whose names are NASA's power diagram [20240009560 fig. 5]. What every control does is in Docs/X-57-cockpit-guide.md.

    build(base, d)     base: Models/Interior/Panel of the package; d: its X57 folder. Logic: nasal/cockpit.nas; screens: nasal/displays.nas
"""
import math, os, re
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import mkmodel as M

INK = (16, 17, 19)
WHITE = (238, 238, 232)
GOLD = (222, 196, 120)


def R(x, y):
    """the right panel, from pixels of the 3x crop from (995, 600) of fig. 4"""
    return 0.097 + (x - 20) * 0.000387, 0.08 - (y - 20) * 0.000368


def L(x, y):
    """the left panel, from pixels of the 3x crop from (460, 600)"""
    return -0.436 + (x - 225) * 0.000382, -0.032 - (y - 300) * 0.000382


def font(n):
    return ImageFont.load_default(size=n)


def centred(g, x, y, words, n, fill=INK):
    f = font(n)
    g.text((x - g.textlength(words, font=f) / 2, y - n * 0.62), words, font=f, fill=fill)


def surf(idx, uv=None, flag='0x20', mat=0):
    return ['SURF ' + flag, 'mat %d' % mat, 'refs %d' % len(idx)] + ['%d %.4f %.4f' % ((i,) + (uv[k] if uv else (0, 0))) for k, i in enumerate(idx)]


def obj(name, verts, surfs, tex=None):
    """verts (x, y, z) in the model's frame"""
    return '\n'.join(['OBJECT poly', 'name "%s"' % name] + (['texture "%s"' % tex] if tex else []) + ['numvert %d' % len(verts)] + ['%.5f %.5f %.5f' % (x, z, -y) for x, y, z in verts]
                     + ['numsurf %d' % len(surfs)] + [l for s in surfs for l in s] + ['kids 0'])


def plate(name, y0, z0, y1, z1, x, tex=None, mat=0):
    return M.quad(name, y0, z0, y1, z1, x, tex, mat)


def box(name, y0, z0, y1, z1, x, tex, side=(0.0, 0.0), back=0.0):
    """a block standing out of the panel: its face the picture, its sides sampling one point of it"""
    v = [(back, y0, z0), (back, y1, z0), (back, y1, z1), (back, y0, z1), (x, y0, z0), (x, y1, z0), (x, y1, z1), (x, y0, z1)]
    s = [surf((4, 5, 6, 7), ((0, 0), (1, 0), (1, 1), (0, 1)))]
    for a, b in ((0, 1), (1, 2), (2, 3), (3, 0)):
        s.append(surf((a, b, b + 4, a + 4), (side,) * 4))
    return obj(name, v, s, tex)


def circle(name, y, z, r, x, tex=None, mat=0, n=40):
    a = [2 * math.pi * k / n for k in range(n)]
    v = [(x, y + r * math.cos(q), z + r * math.sin(q)) for q in a]
    return obj(name, v, [surf(range(n), [(0.5 + 0.5 * math.cos(q), 0.5 + 0.5 * math.sin(q)) for q in a], mat=mat)], tex)


def lathe(name, prof, y=0.0, z=0.0, tex=None, n=24):
    """round about an axis along x: (radius, x) from the panel out, closed at the end"""
    v, s = [], []
    for r, x in prof:
        v += [(x, y + r * math.cos(2 * math.pi * k / n), z + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
    for i in range(len(prof) - 1):
        for k in range(n):
            j = (k + 1) % n
            s.append(surf((i * n + k, i * n + j, (i + 1) * n + j, (i + 1) * n + k), flag='0x30'))
    s.append(surf([(len(prof) - 1) * n + k for k in range(n)], flag='0x30'))
    return obj(name, v, s, tex)


def chaikin(pts, n=2):
    """round a closed outline's corners"""
    for _ in range(n):
        q = []
        for a, b in zip(pts, pts[1:] + pts[:1]):
            q += [(0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1]), (0.25 * a[0] + 0.75 * b[0], 0.25 * a[1] + 0.75 * b[1])]
        pts = q
    return pts


def earclip(pts):
    """triangles of a simple closed outline, any winding"""
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1]))
    idx = list(range(len(pts))) if area > 0 else list(range(len(pts)))[::-1]
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    out = []
    while len(idx) > 3:
        for k in range(len(idx)):
            i, j, l = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = pts[i], pts[j], pts[l]
            if cross(a, b, c) <= 1e-12:
                continue
            if any(cross(a, b, pts[m]) >= 0 and cross(b, c, pts[m]) >= 0 and cross(c, a, pts[m]) >= 0 for m in idx if m not in (i, j, l)):
                continue
            out.append((i, j, l))
            idx.pop(k)
            break
        else:
            break
    out.append(tuple(idx[:3]))
    return out


def outline(name, pts, x, tex, back=None, uv=None, side=(0.5, 0.5)):
    """a shaped face at x: pts (y, z). With back, its sides run back to there. uv: a point's picture coordinates"""
    n = len(pts)
    uv = uv or (lambda p: side)
    v = [(x, p[0], p[1]) for p in pts]
    s = [surf(t, [uv(pts[i]) for i in t], flag='0x10') for t in earclip(pts)]
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1]))
    if back is not None:
        v += [(back, p[0], p[1]) for p in pts]
        for i in range(n):
            j = (i + 1) % n
            q = (i, i + n, j + n, j) if area > 0 else (j, j + n, i + n, i)
            s.append(surf(q, (side,) * 4, flag='0x30'))
    return obj(name, v, s, tex)


def lit(objs, level, colour='1 1 1', reflect=0.5):
    return ('<effect><inherits-from>Aircraft/X-57/Models/Effects/Lit/lit</inherits-from><parameters><lit-color type="vec3d">%s</lit-color><lit-level><use>%s</use></lit-level><lit-gain type="float">1</lit-gain>'
            '<lit-reflectance type="float">%g</lit-reflectance></parameters>%s</effect>' % (colour, level, reflect, ''.join('<object-name>%s</object-name>' % o for o in objs)))


LAMP = '/instrumentation/x57/lamp/'
NITS = '/instrumentation/x57/nits/'
X57 = 'sim/model/x57/'
SW = 'fdm/jsbsim/ctl/sw/'


def hold(objs, prop, value=1, rest=0):
    """a pick that holds a property while the mouse is down"""
    objs = objs if isinstance(objs, (list, tuple)) else [objs]
    return ('<animation><type>pick</type>%s<action><button>0</button><repeatable type="bool">false</repeatable><binding><command>property-assign</command><property>%s</property><value>%g</value></binding>'
            '<mod-up><binding><command>property-assign</command><property>%s</property><value>%g</value></binding></mod-up></action></animation>'
            % (''.join('<object-name>%s</object-name>' % o for o in objs), prop, value, prop, rest))


def word_plate(k, name, words, n=26, size=(192, 96), colour=INK, ink=WHITE):
    im = Image.new('RGB', size, colour)
    centred(ImageDraw.Draw(im), size[0] / 2, size[1] / 2, words, n, ink)
    return k.picture(name, im)


class Kit:
    def __init__(s, d):
        s.d, s.objs, s.xml, s.shots = d, [], [], []
        s.heads, s.dark, s.black, s.steel, s.red, s.toggles = [], [], [], [], [], 0
        Image.new('RGB', (8, 8), (0, 0, 0)).save(os.path.join(d, 'screen.png'))

    def screen(s, name, y0, z0, y1, z1, x):
        s.objs.append(plate(name + '.screen', y0, z0, y1, z1, x, 'screen.png'))
        s.xml.append(lit((name + '.screen',), NITS + name, reflect=0.04))

    def picture(s, name, im):
        im.save(os.path.join(s.d, name + '.png'))
        return name + '.png'

    def round_unit(s, name, y, z, r, tex, x=0.004, at=None):
        s.objs.append(lathe(name + '-case', ((r, 0.0), (r, x)), y, z))
        s.black.append(name + '-case')
        s.objs.append(circle(name + '-face', y, z, r, x + 0.0002, tex))         # clear of the case's closed end
        s.dark.append(name + '-face')
        if at:
            s.heads += M.four(y, z, at, 0.0026)

    def lamp(s, name, y, z, r, colour, level, x=0.0):
        s.objs.append(lathe(name + '-ring', ((r * 1.35, x), (r * 1.35, x + 0.0022), (r, x + 0.0022)), y, z))
        s.black.append(name + '-ring')
        s.objs.append(circle(name, y, z, r, x + 0.0024, 'lens-%s.png' % colour))
        s.xml.append(lit((name,), level, LENS[colour][0], DEAD))

    def led2(s, name, y, z, r, x=0.0):
        """a bicolor LED: green, or red while its flag says so"""
        s.objs.append(lathe(name + '-ring', ((r * 1.35, x), (r * 1.35, x + 0.0022), (r, x + 0.0022)), y, z))
        s.black.append(name + '-ring')
        for c, cond in (('g', '<not><property>%stc-red/%s</property></not>' % (X57, name)), ('r', '<property>%stc-red/%s</property>' % (X57, name))):
            s.objs.append(circle('led-%s-%s' % (name, c), y, z, r, x + 0.0024, 'lens-%s.png' % c))
            s.xml.append(lit(('led-%s-%s' % (name, c),), LAMP + 'tc-' + name, LENS[c][0], DEAD))
            s.xml.append('<animation><type>select</type><object-name>led-%s-%s</object-name><condition>%s</condition></animation>' % (name, c, cond))

    def push(s, name, y, z, r, prop, x=0.0, h=0.0028, red=False):
        """a button, held while the mouse is down"""
        s.objs.append(lathe(name, ((r, x), (r, x + h), (r * 0.8, x + h + 0.0006), (0.0, x + h + 0.0007)), y, z))
        (s.red if red else s.black).append(name)
        s.xml.append(hold(name, prop))

    def spring(s, name, y, z, prop, x=0.0, words=('START', 'FEATHER')):
        """START and FEATHER: sprung to the middle. Press its tags; the bat follows the property"""
        s.xml.append('<model><path>%s.xml</path><offsets><x-m>%.4f</x-m><y-m>%.4f</y-m><z-m>%.4f</z-m></offsets></model>' % (name, x, y, z))
        open(os.path.join(s.d, name + '.xml'), 'w').write(
            '<?xml version="1.0"?>\n<PropertyList><path>toggle.ac</path><animation><type>rotate</type><object-name>bat</object-name><property>%s</property>'
            '<interpolation><entry><ind>-1</ind><dep>-24</dep></entry><entry><ind>1</ind><dep>24</dep></entry></interpolation><axis><x>0</x><y>1</y><z>0</z></axis><center><x-m>0.002</x-m><y-m>0</y-m><z-m>0</z-m></center></animation>%s%s</PropertyList>\n'
            % (prop, M.effect('x57-red', ('nut',)), M.effect('x57-steel', ('bat',))))
        for w, dz, v_ in ((words[0], 0.0125, -1), (words[1], -0.0125, 1)):
            n = '%s-%s' % (name, w.lower())
            s.objs.append(plate(n, y - 0.0115, z + dz - 0.0048, y + 0.0115, z + dz + 0.0048, x + 0.0004, word_plate(s, 'tag-' + w.lower(), w)))
            s.dark.append(n)
            s.xml.append(hold(n, prop, v_))

    def toggle(s, name, y, z, prop, red=False, x=0.0, up=True, model='toggle', big=False):
        s.toggles += 1
        hi, lo = (-24, 24) if up else (24, -24)             # up: on is up
        s.xml.append('<model><path>%s.xml</path><offsets><x-m>%.4f</x-m><y-m>%.4f</y-m><z-m>%.4f</z-m></offsets></model>' % (name, x, y, z))
        open(os.path.join(s.d, name + '.xml'), 'w').write(
            ('<?xml version="1.0"?>\n<PropertyList><path>' + model + '.ac</path>'
            '<animation><type>rotate</type><object-name>bat</object-name><property>%s</property><interpolation><entry><ind>0</ind><dep>' + str(lo) + '</dep></entry><entry><ind>1</ind><dep>' + str(hi) + '</dep></entry></interpolation>'
            '<axis><x>0</x><y>1</y><z>0</z></axis><center><x-m>0.002</x-m><y-m>0</y-m><z-m>0</z-m></center></animation>'
            '<animation><type>pick</type><object-name>bat</object-name><object-name>nut</object-name><action><button>0</button><repeatable type="bool">false</repeatable><binding><command>property-toggle</command><property>%s</property></binding></action></animation>%s%s</PropertyList>\n')
            % (prop, prop, M.effect('x57-case' if not red else 'x57-red', ('nut',)), M.effect('x57-case' if big else 'x57-steel', ('bat',))))


def toggle_model(d):
    nut = lambda: lathe('nut', ((0.0058, 0.0), (0.0058, 0.0012), (0.0042, 0.0014), (0.0042, 0.0034), (0.0034, 0.0036)))
    for n, bat in (('long', ((0.0020, 0.003), (0.0026, 0.010), (0.0034, 0.030), (0.0040, 0.034), (0.0030, 0.036))),         # CRUISE TRACTION POWER's
                   ('bus', ((0.0034, 0.003), (0.0040, 0.008), (0.0058, 0.0165), (0.0058, 0.0225), (0.0044, 0.0235))),       # the low voltage lever locks: fat, flat-ended
                   ('ball', ((0.0018, 0.003), (0.0021, 0.008), (0.0040, 0.0150), (0.0052, 0.0190), (0.0040, 0.0230), (0.0, 0.0240)))):    # ESS BATT: told from the rest by feel
        with open(os.path.join(d, n + '.ac'), 'w') as f:
            f.write(M.ac([nut(), lathe('bat', bat)]))
    with open(os.path.join(d, 'lever.ac'), 'w') as f:                  # the landing gear's: a wheel-shaped black knob
        f.write(M.ac([lathe('nut', ((0.0075, 0.0), (0.0075, 0.0014), (0.0050, 0.0018), (0.0050, 0.0040))),
                      lathe('bat', ((0.0026, 0.003), (0.0026, 0.016), (0.0095, 0.0175), (0.0095, 0.0285), (0.0060, 0.030)))]))
    with open(os.path.join(d, 'toggle.ac'), 'w') as f:
        f.write(M.ac([lathe('nut', ((0.0058, 0.0), (0.0058, 0.0012), (0.0042, 0.0014), (0.0042, 0.0034), (0.0034, 0.0036))),
                      lathe('bat', ((0.0018, 0.003), (0.0021, 0.008), (0.0030, 0.0165), (0.0030, 0.0185), (0.0018, 0.0195)))]))


# an unlit lens: the daylight it gives back, as a fraction of its picture. Coloured glass over a dark lamp cavity is a dark
# dead front; at 0.7-0.8 the green ones looked lit with the gear up, so nothing seemed to change
DEAD = 0.15
LENS = {'g': ('0.25 1 0.35', (46, 170, 84)), 'b': ('0.2 0.45 1', (44, 92, 210)), 'y': ('1 0.8 0.15', (214, 186, 60)), 'r': ('1 0.18 0.1', (176, 44, 34)), 'a': ('1 0.72 0.15', (206, 136, 44))}


def lens(d):
    """the lamps' coloured lenses, the light falling on them from the upper left"""
    a = np.mgrid[0:64, 0:64]
    r = np.clip(1.25 - np.hypot(a[0] - 24, a[1] - 24) / 44.0, 0.3, 1.15)
    for k, (_, c) in LENS.items():
        Image.fromarray(np.clip(np.dstack([r * q for q in c]), 0, 255).astype('uint8'), 'RGB').save(os.path.join(d, 'lens-%s.png' % k))


def panel_faces(k, base):
    """each panel's printing: the panel's own front, moved to an object of its own with the picture laid flat"""
    s = open(os.path.join(base, 'panel.ac'), errors='replace').read()
    sides = {'r': (0.09, 0.53, -0.26, 0.09, []), 'l': (-0.53, -0.09, -0.26, 0.09, [])}
    parts = re.split(r'(?=^OBJECT )', s, flags=re.M)
    for w, o in enumerate(parts):
        l = o.split('\n')
        i = next((j for j, q in enumerate(l) if q.startswith('numvert')), None)
        if i is None:
            continue
        c = int(l[i].split()[1])
        v = np.array([q.split()[:3] for q in l[i + 1:i + 1 + c]], float) + np.array((re.findall(r'^loc (.*)', o, re.M) or ['0 0 0'])[0].split(), float)
        P = np.c_[v[:, 0], -v[:, 2], v[:, 1]]
        j = i + 2 + c
        kept = []
        while j < len(l) and l[j].startswith('SURF'):
            e = int(l[j + 2].split()[1])
            idx = [int(q.split()[0]) for q in l[j + 3:j + 3 + e]]
            if np.abs(P[idx, 0]).max() < 0.0015:
                sd = 'r' if P[idx, 1].mean() > 0 else 'l'
                sides[sd][4].append(P[idx])
            else:
                kept += l[j:j + 3 + e]
            j += 3 + e
        parts[w] = '\n'.join(l[:i + 1 + c] + ['numsurf %d' % sum(q.startswith('SURF') for q in kept)] + kept + l[j:])
    open(os.path.join(base, 'panel.ac'), 'w', errors='replace').write(''.join(parts))
    out = {}
    for sd, (y0, y1, z0, z1, polys) in sides.items():
        v, ss = [], []
        for p in polys:
            b = len(v)
            v += [(0.0, q[1], q[2]) for q in p]                # where the panel's own front was: the altimeter's pressure dial lies 0.1 mm out
            ss.append(surf(range(b, b + len(p)), [((q[1] - y0) / (y1 - y0), (q[2] - z0) / (z1 - z0)) for q in p]))
        k.objs.append(obj('face-' + sd, v, ss, 'face-%s.png' % sd))
        out[sd] = (y0, y1, z0, z1)
        FACE_POLYS[sd] = [np.asarray(p)[:, 1:] for p in polys]
    return out


FACE_POLYS = {}          # each panel's face, (y, z) polygons: what may be mounted on it


def on_face(sd, box_, inset, step=0.00025):
    """the outline of the panel face's part inside box_ (y0, y1, z0, z1), its edges brought in by inset: a part made with it
    stays on the panel, clear of its edge by inset"""
    from matplotlib.path import Path
    from scipy import ndimage
    import contourpy
    y0, y1, z0, z1 = box_
    ys, zs = np.arange(y0 - 0.02, y1 + 0.02, step), np.arange(z0 - 0.02, z1 + 0.02, step)
    Y, Z = np.meshgrid(ys, zs)
    q = np.c_[Y.ravel(), Z.ravel()]
    m = np.zeros(len(q), bool)
    for p in FACE_POLYS[sd]:
        m |= Path(p).contains_points(q)
    m = ndimage.binary_closing(m.reshape(Y.shape), iterations=2)        # the seams between the face's triangles
    r = int(round(inset / step))
    a, b = np.mgrid[-r:r + 1, -r:r + 1]
    m = ndimage.binary_erosion(m, structure=a * a + b * b <= r * r) & (Y >= y0) & (Y <= y1) & (Z >= z0) & (Z <= z1)
    line = max(contourpy.contour_generator(x=ys, y=zs, z=m.astype(float)).lines(0.5), key=len)

    def rdp(pts, tol):
        a_, b_ = pts[0], pts[-1]
        e_, f_ = b_ - a_, pts - a_
        d = np.abs(e_[0] * f_[:, 1] - e_[1] * f_[:, 0]) / (np.linalg.norm(e_) + 1e-12)
        i = int(np.argmax(d))
        return np.vstack([rdp(pts[:i + 1], tol)[:-1], rdp(pts[i:], tol)]) if d[i] > tol else np.vstack([a_, b_])
    line = np.asarray(line)
    if np.allclose(line[0], line[-1]):
        line = line[:-1]
    far = int(np.argmax(np.linalg.norm(line - line[0], axis=1)))           # a closed loop: simplified as two halves
    pts = np.vstack([rdp(line[:far + 1], 0.0002)[:-1], rdp(np.vstack([line[far:], line[:1]]), 0.0002)[:-1]])
    area = sum(p[0] * q_[1] - q_[0] * p[1] for p, q_ in zip(pts, np.roll(pts, -1, 0)))
    return [tuple(map(float, p)) for p in (pts if area > 0 else pts[::-1])]


def canvas_of(box_, colour, px=9000):
    y0, y1, z0, z1 = box_
    W, H = int((y1 - y0) * px), int((z1 - z0) * px)
    rng = np.random.default_rng(57)
    g = np.array(colour, float) + rng.normal(0, 0.5, (1, W, 1)) + rng.normal(0, 0.7, (H, W, 1))
    im = Image.fromarray(np.clip(g, 0, 255).astype('uint8'), 'RGB')
    return im, ImageDraw.Draw(im), (lambda y, z: ((y - y0) * px, (z1 - z) * px)), px


def tag(g, at, y, z, words, n=30, w=None):
    """a black engraved tag, as the panel's switches have"""
    x_, y_ = at(y, z)
    w = w or g.textlength(words, font=font(n)) + 20
    g.rectangle((x_ - w / 2, y_ - n * 0.8, x_ + w / 2, y_ + n * 0.8), fill=INK)
    centred(g, x_, y_, words, n, WHITE)


def right_face(k, b):
    im, g, at, px = canvas_of(b, (176, 179, 183))
    for i, c in enumerate('RPM'):
        centred(g, *at(*R(58, 100 + 38 * i)), c, 56, (46, 48, 52))
    for x, y, w in ((178, 222, 'Left Traction Bus'), (425, 218, 'Right Traction Bus'), (655, 213, 'Avionics Power')):
        centred(g, *at(*R(x, y)), w, 40, (46, 48, 52))
    centred(g, *at(*R(55, 348)), 'A', 62, (46, 48, 52))
    centred(g, *at(*R(58, 570)), 'B', 62, (46, 48, 52))
    a, c = at(*R(40, 450)), at(*R(740, 450))
    g.line((a[0], a[1], c[0], c[1]), fill=(30, 31, 34), width=7)
    for x in (132, 497):
        tag(g, at, *R(x, 710), 'MAN', 24, 120)
        tag(g, at, *R(x, 776), 'AUTO', 24, 120)
    centred(g, *at(*R(195, 738)), 'Prop Pitch', 22, (46, 48, 52))
    centred(g, *at(*R(195, 752)), 'Control', 22, (46, 48, 52))
    centred(g, *at(*R(438, 730)), 'Prop Pitch', 22, (46, 48, 52))
    centred(g, *at(*R(438, 744)), 'Control', 22, (46, 48, 52))
    tag(g, at, *R(785, 658), 'OAT', 28, 110)
    im.save(os.path.join(k.d, 'face-r.png'))


def left_face(k, b):
    im, g, at, px = canvas_of(b, (172, 175, 180))
    # ANNUNCIATOR PANEL: a black label plate under the box, annunciator() makes it
    # NASA's EXPERIMENTAL placard: a plate of its own, where the Tecnam's dimmer panel was, so that fitting the Tecnam's cockpit lights
    # takes it away for that panel (x57tecnam.py)
    (py0, pz1), (py1, pz0) = L(275, 705), L(480, 770)
    q = Image.new('RGB', (int(round((py1 - py0) * px)), int(round((pz1 - pz0) * px))), INK)
    gq = ImageDraw.Draw(q)
    centred(gq, q.size[0] / 2, 50, 'EXPERIMENTAL', 44, WHITE)
    centred(gq, q.size[0] / 2, 120, 'X-57 FLIGHT RESEARCH', 34, WHITE)
    plate = [(py0, pz0), (py1, pz0), (py1, pz1), (py0, pz1)]
    k.objs.append(outline('placard', plate, 0.0008, k.picture('placard', q), back=0.0, uv=lambda p: ((p[0] - py0) / (py1 - py0), (p[1] - pz0) / (pz1 - pz0))))
    k.dark.append('placard')
    k.xml.append('<animation><type>select</type><object-name>placard</object-name><condition><not><property>%s</property></not></condition></animation>' % TECNAM_FIT)
    tag(g, at, *L(800, 702), 'CABIN FAN', 22, 130)
    tag(g, at, *L(800, 722), 'ON', 22, 60)
    im.save(os.path.join(k.d, 'face-l.png'))


def round_face(size=512, words=(), hole=None, colour=(20, 21, 23)):
    im = Image.new('RGB', (size, size), colour)
    g = ImageDraw.Draw(im)
    g.ellipse((6, 6, size - 6, size - 6), outline=(52, 53, 56), width=8)
    for x, y, w, n in words:
        centred(g, x * size, y * size, w, n, WHITE)
    return im


def meter_face(k):
    im = Image.new('RGB', (1024, 1024), (236, 236, 232))
    g = ImageDraw.Draw(im)
    g.rectangle((10, 10, 1013, 1013), outline=(205, 205, 200), width=14)
    # the dark window, its lower left a tab that sweeps down in an S-curve from the right edge, the maker's name printed wholly
    # on the tab and the two buttons on the white beside it, as on the meters in 20230005312 fig. 1
    f = font(58)
    end = 130 + g.textlength('ACCUENERGY', font=f) + 45                  # the tab's foot: the name and a margin
    curve = [((1 - t) ** 3 * 920 + 3 * (1 - t) ** 2 * t * 800 + 3 * (1 - t) * t * t * (end + 90) + t ** 3 * end,
              (1 - t) ** 3 * 690 + 3 * (1 - t) ** 2 * t * 690 + 3 * (1 - t) * t * t * 880 + t ** 3 * 880) for t in np.linspace(0, 1, 40)]
    g.polygon([(100, 90), (920, 90)] + curve + [(100, 880)], fill=(10, 13, 14))
    g.text((130, 790), 'ACCUENERGY', font=f, fill=(60, 170, 220))
    for x in (712, 828):
        g.rounded_rectangle((x, 828, x + 92, 898), 12, fill=(46, 104, 210))
    return k.picture('meter', im)


ANN_UP = 0.007         # up a little from the photograph: the Tecnam's bezels are wider than the real ones, and its label sits between


def annunciator(k):
    """the strip, as ICD slide 36 and 20240006533 fig. 10: DC CONV, MAIN BATT, TRAC BUS and the four CMCs, the Tecnam's own lights
    moved to the right. A black box standing 8 mm out, as in ICD slide 11: its left end follows the panel's rounded corner and its
    top the panel's edge, a few millimetres in, each lamp in a framed cell, the buttons in chrome rings"""
    y0, z1 = L(200, 45)
    y1, z0 = L(770, 180)
    z0, z1 = z0 + ANN_UP, z1 + ANN_UP
    w_, h_ = y1 - y0, z1 - z0
    at = lambda u, v: (y0 + w_ * u / 1024.0, z1 - h_ * v / 256.0)            # places on the photograph's strip
    pts = on_face('l', (y0 - 0.002, y1, z0, z1 + 0.012), 0.0035)
    b0, b1 = min(p[0] for p in pts), max(p[0] for p in pts)
    c0, c1 = min(p[1] for p in pts), max(p[1] for p in pts)
    PX = 6000.0
    im = Image.new('RGB', (int((b1 - b0) * PX) + 1, int((c1 - c0) * PX) + 1), (30, 31, 34))
    g = ImageDraw.Draw(im)
    px = lambda y, z: ((y - b0) * PX, (c1 - z) * PX)
    scale = PX * w_ / 1024.0                                                  # the old strip's pixels in this one's
    for (u, v0, dv, n), words in (((80, 158, 26, 22), ('AUDIO', 'ANN.', 'ACK')), ((982, 92, 20, 17), ('AUDIO', 'CHECK')), ((982, 214, 20, 17), ('LIGHT', 'CHECK'))):
        for i, w in enumerate(words):
            centred(g, *px(*at(u, v0 + dv * i)), w, int(n * scale), WHITE)
    Y, Rd, G = (150, 104, 12), (150, 16, 22), (20, 112, 62)
    for n in range(16):
        y, z = L(318 + 54 * (n % 8), 92 + 58 * (n // 8))
        a, b = px(y - 0.0099, z + ANN_UP + 0.0099), px(y + 0.0099, z + ANN_UP - 0.0099)
        g.rectangle((a[0], a[1], b[0], b[1]), fill=(13, 13, 14), outline=(74, 76, 80), width=3)
    tex = k.picture('ann', im)
    k.objs.append(outline('ann-box', pts, 0.008, tex, uv=lambda p: ((p[0] - b0) / (b1 - b0), (p[1] - c0) / (c1 - c0))))
    k.dark.append('ann-box')
    k.objs.append(outline('ann-box-edge', pts, 0.0077, None, back=0.0))
    k.black.append('ann-box-edge')
    # ANNUNCIATOR PANEL, white on a black plate just under the box and flush with its front, narrower than it, as in ICD slide 11's
    # photograph, where it shows whole from behind the seats (slide 36's drawing has no lettering)
    ly0, ly1, lz1 = L(295, 0)[0], L(720, 0)[0], z0
    lz0 = lz1 - 0.0064
    q = Image.new('RGB', (int((ly1 - ly0) * PX), int((lz1 - lz0) * PX)), INK)
    centred(ImageDraw.Draw(q), q.size[0] / 2, q.size[1] / 2 + 2, 'ANNUNCIATOR PANEL', int(34 * PX / 9000), WHITE)
    lab = [(ly0, lz0), (ly1, lz0), (ly1, lz1), (ly0, lz1)]
    k.objs.append(outline('ann-label', lab, 0.008, k.picture('ann-label', q), uv=lambda p: ((p[0] - ly0) / (ly1 - ly0), (p[1] - lz0) / (lz1 - lz0))))
    k.dark.append('ann-label')
    k.objs.append(outline('ann-label-edge', lab, 0.0077, None, back=0.0))
    k.black.append('ann-label-edge')
    for name, (u, v), r, prop in (('ann-ack', (80, 90), 0.0048, 'audio-ack'), ('ann-check', (982, 46), 0.0036, 'audio-check'), ('light-check', (982, 168), 0.0036, 'light-check')):
        y, z = at(u, v)
        k.push(name, y, z, r, X57 + prop, 0.008)
        k.objs.append(lathe(name + '-chrome', ((r + 0.0018, 0.008), (r + 0.0018, 0.0092), (r + 0.0004, 0.0096)), y, z))
        k.steel.append(name + '-chrome')
    cells = (('DC|CONV A', Y, 'dc-conv-a'), ('MAIN|BATT A', Rd, 'main-batt-a'), ('TRAC|BUS A', Y, 'trac-bus-a'), ('LEFT|CMC A', Y, 'cmc-la'), ('RIGHT|CMC A', Y, 'cmc-ra'), ('MAIN|DOOR|OPEN', Rd, 'main-door'), ('PITOT|HEAT', G, 'pitot-on'), ('GEAR|PUMP|ON', Y, 'gear-pump'),
             ('DC|CONV B', Y, 'dc-conv-b'), ('MAIN|BATT B', Rd, 'main-batt-b'), ('TRAC|BUS B', Y, 'trac-bus-b'), ('LEFT|CMC B', Y, 'cmc-lb'), ('RIGHT|CMC B', Y, 'cmc-rb'), ('REAR|DOOR|OPEN', Rd, 'rear-door'), ('PITOT|HEAT', Y, 'pitot-fail'), ('ESS|BATT', Rd, 'ess-batt'))
    for n, (w, c, lamp_) in enumerate(cells):
        y, z = L(318 + 54 * (n % 8), 92 + 58 * (n // 8))
        z += ANN_UP
        q = Image.new('RGB', (192, 192), c)
        d = ImageDraw.Draw(q)
        d.rectangle((3, 3, 188, 188), outline=tuple(min(255, int(x * 1.5) + 20) for x in c), width=8)
        parts = w.split('|')
        for i, part in enumerate(parts):
            centred(d, 96, 96 + (i - (len(parts) - 1) / 2) * 46, part, 40, (252, 238, 205))
        name = 'ann-' + lamp_
        k.objs.append(plate(name, y - 0.0088, z - 0.0088, y + 0.0088, z + 0.0088, 0.0085, k.picture(name, q)))
        k.xml.append(lit((name,), LAMP + lamp_, reflect=DEAD))        # unlit, a dark tile, its legend just readable [the panel photographs]


def high_lift(k):
    """the Mod IV panel as ICD slides 64 and 66: ARM or DISARM/RESET, the yellow RESET REQ'D, RDY, RUN and FAULT for sides A and B, AIRSPEED or FIXED"""
    y0, z1 = L(785, 55)
    y1, z0 = L(1055, 170)
    im = Image.new('RGB', (1024, 440), (176, 186, 196))
    g = ImageDraw.Draw(im)
    g.rectangle((6, 6, 1017, 433), outline=(20, 21, 24), width=10)
    g.line((110, 100, 914, 100), fill=(26, 27, 30), width=4)
    g.line((470, 120, 470, 410), fill=(26, 27, 30), width=4)
    centred(g, 512, 58, 'High Lift System', 44)
    centred(g, 210, 124, 'ARM', 34)
    centred(g, 210, 412, 'DISARM/RESET', 32)
    centred(g, 400, 268, 'RESET', 22)
    centred(g, 400, 296, "REQ'D", 22)
    centred(g, 850, 124, 'AIRSPEED', 34)
    centred(g, 850, 412, 'FIXED', 34)
    centred(g, 548, 132, 'A', 30)
    centred(g, 668, 132, 'B', 30)
    for i, w in enumerate(('RDY', 'RUN', 'FAULT')):
        centred(g, 608, 190 + 86 * i, w, 24)
    k.objs.append(box('hl-box', y0, z0, y1, z1, 0.003, k.picture('hl', im), (0.02, 0.02)))
    k.dark.append('hl-box')
    w, h = y1 - y0, z1 - z0
    at = lambda u, v: (y0 + w * u / 1024.0, z1 - h * v / 440.0)
    for i, (n, c) in enumerate((('rdy', 'b'), ('run', 'g'), ('fault', 'r'))):
        for s_, u in (('a', 548), ('b', 668)):
            y, z = at(u, 190 + 86 * i)
            k.lamp('hl-%s-%s' % (n, s_), y, z, 0.0034, c, LAMP + 'hl-%s-%s' % (n, s_), 0.003)
    k.lamp('hl-reset', *at(400, 216), 0.0034, 'y', LAMP + 'hl-reset', 0.003)
    k.toggle('tg-hl-arm', *at(210, 226), X57 + 'hl-arm', x=0.003)
    k.toggle('tg-hl-mode', *at(850, 226), X57 + 'hl-airspeed', x=0.003)
    k.heads += [at(u, v) + (0.0022,) for u, v in ((40, 40), (984, 40), (40, 400), (984, 400))]


def crew_panel(k):
    """the high lift ground crew indicators, where the P2006's plaque is on Mod II [ICD slide 64]: its LEDs are a screen"""
    y0, y1, z0, z1 = -0.078, 0.078, 0.080, 0.128
    im = Image.new('RGB', (1560, 480), (176, 186, 196))
    g = ImageDraw.Draw(im)
    g.rectangle((5, 5, 1554, 474), outline=(20, 21, 24), width=8)
    for i in range(12):
        x = 330 + i * 75.0 + (16 if i >= 6 else 0)
        centred(g, x, 36, str(i + 1), 34)
        centred(g, x, 448, 'AB'[i % 2] if i < 6 else 'BA'[i % 2], 30)
    for x in (120, 1440):
        centred(g, x, 68, 'ON', 34)
        centred(g, x, 412, 'OFF', 34)
    k.objs.append(box('crew-box', y0, z0, y1, z1, -0.002, k.picture('crew', im), (0.02, 0.02), -0.0058))
    k.dark.append('crew-box')
    k.screen('crew', -0.0485, 0.0870, 0.0505, 0.1210, -0.0016)
    k.toggle('tg-crew-1', -0.0660, 0.104, X57 + 'crew-check', x=-0.002)
    k.toggle('tg-crew-2', 0.0660, 0.104, X57 + 'crew-display', x=-0.002)


def uhf_face(k):
    im = round_face(1024)
    g = ImageDraw.Draw(im)
    g.rounded_rectangle((170, 200, 870, 830), 24, fill=(36, 37, 40))
    g.rectangle((230, 250, 730, 500), fill=(8, 8, 9))
    centred(g, 480, 556, 'UHF', 46, WHITE)
    centred(g, 300, 640, 'VOL', 28, WHITE)
    centred(g, 500, 640, 'PUSH TEST', 28, WHITE)
    centred(g, 690, 760, 'CHAN', 28, WHITE)
    return k.picture('uhf-face', im)


def gmeter_face(k):
    im = round_face(1024)
    g = ImageDraw.Draw(im)
    centred(g, 512, 212, 'GT-50', 48, WHITE)
    centred(g, 512, 282, 'Flight Data Systems', 48, WHITE)
    g.rounded_rectangle((180, 340, 844, 630), 40, fill=(10, 10, 10))
    for x, w in ((280, 'Mode'), (512, 'Reset'), (744, 'Action')):
        centred(g, x, 690, w, 44, WHITE)
    return k.picture('g-face', im)


def stack(k):
    """the centre stack, top to bottom. Crop pixels of the 3x crop from (825, 555)"""
    Y = lambda x: -0.0795 + 0.159 * (x - 45) / 450.0
    k.objs.append(plate('stack', -0.0835, -0.2335, 0.0835, 0.0785, -0.004, None, 5))
    k.black.append('stack')
    # the navigator
    im = Image.new('RGB', (1024, 460), (30, 31, 34))
    g = ImageDraw.Draw(im)
    g.rectangle((140, 26, 900, 430), fill=(6, 6, 7))
    g.rectangle((26, 170, 44, 300), fill=(12, 12, 13))
    centred(g, 962, 52, 'GARMIN', 22, WHITE)
    k.objs.append(box('gtn', -0.0795, 0.0045, 0.0795, 0.0745, 0.006, k.picture('gtn', im), (0.5, 0.99)))
    k.dark.append('gtn')
    k.screen('gtn', -0.0575, 0.0080, 0.0605, 0.0710, 0.0064)
    k.objs.append(lathe('gtn-vol', ((0.0055, 0.006), (0.0055, 0.0125), (0.0, 0.013)), -0.0700, 0.0640))
    k.objs.append(lathe('gtn-dual', ((0.0075, 0.006), (0.0075, 0.012), (0.0048, 0.0125), (0.0048, 0.0175), (0.0, 0.018)), 0.0700, 0.0150))
    k.black += ['gtn-vol', 'gtn-dual']
    # the transponder
    im = Image.new('RGB', (1024, 288), (28, 29, 32))
    g = ImageDraw.Draw(im)
    g.rectangle((270, 26, 780, 170), fill=(6, 6, 8))
    for i, w in enumerate('0123456789'):
        x = 60 + i * 100
        g.rounded_rectangle((x - 34, 200, x + 34, 262), 10, fill=(52, 53, 56))
        centred(g, x, 232, w, 36, WHITE)
    for x, y, w in ((70, 46, 'VFR'), (70, 112, 'ALT'), (180, 46, 'STBY'), (180, 112, 'OFF'), (860, 46, 'FUNC'), (960, 46, 'CRSR'), (860, 112, 'START'), (960, 112, 'CLR')):
        g.rounded_rectangle((x - 46, y - 26, x + 46, y + 26), 8, fill=(52, 53, 56))
        centred(g, x, y, w, 24, WHITE)
    k.objs.append(box('gtx', -0.0795, -0.0415, 0.0795, 0.0025, 0.006, k.picture('gtx', im), (0.5, 0.99)))
    k.dark.append('gtx')
    k.screen('gtx', -0.0370, -0.0250, 0.0490, -0.0015, 0.0064)
    # the page switch, FAULT/MAIN and the propeller lights
    im = Image.new('RGB', (1024, 344), (168, 170, 174))
    g = ImageDraw.Draw(im)
    for w, x, y in (('1', 240, 40), ('2', 300, 96), ('3', 318, 186), ('4', 240, 290), ('5', 150, 290), ('6', 50, 230), ('7', 50, 130), ('8', 130, 40)):
        g.rectangle((x - 24, y - 24, x + 24, y + 24), fill=INK)
        centred(g, x, y, w, 34, WHITE)
    for w, y in (('FAULT', 80), ('MAIN', 300)):
        g.rectangle((395, y - 22, 520, y + 22), fill=INK)
        centred(g, 458, y, w, 30, WHITE)
    for w, x, y in (('LH', 650, 72), ('RH', 955, 72)):
        g.rectangle((x - 62, y - 42, x + 62, y + 42), fill=INK)
        centred(g, x, y - 18, w, 26, WHITE)
        centred(g, x, y + 16, 'FAULT', 26, WHITE)
    centred(g, 812, 12, 'PROPELLER PITCH', 19)
    g.rectangle((745, 24, 880, 54), fill=INK)
    centred(g, 812, 39, 'START', 24, WHITE)
    g.rectangle((730, 298, 895, 334), fill=INK)
    centred(g, 812, 316, 'FEATHER', 26, WHITE)
    k.objs.append(plate('strip', -0.0795, -0.0965, 0.0795, -0.0435, 0.0005, k.picture('strip', im)))
    k.dark.append('strip')
    knob = lambda n, y, z, prop, table, values: (open(os.path.join(k.d, n + '.xml'), 'w').write(
        '<?xml version="1.0"?>\n<PropertyList><path>knob.ac</path><animation><type>rotate</type><object-name>mark</object-name><property>%s</property><interpolation>%s</interpolation><axis><x>-1</x><y>0</y><z>0</z></axis><center><x-m>0</x-m><y-m>0</y-m><z-m>0</z-m></center></animation>'
        '<animation><type>pick</type><object-name>top</object-name><object-name>body</object-name><object-name>mark</object-name><action><button>0</button><repeatable type="bool">false</repeatable><binding><command>property-cycle</command><property>%s</property>%s</binding></action></animation>%s</PropertyList>\n'
        % (prop, ''.join('<entry><ind>%g</ind><dep>%g</dep></entry>' % e for e in table), prop, ''.join('<value>%d</value>' % i for i in values), M.effect('x57-case', ('body', 'top')) + M.effect('x57-needle', ('mark',)))),
        k.xml.append('<model><path>%s.xml</path><offsets><x-m>0.001</x-m><y-m>%.4f</y-m><z-m>%.4f</z-m></offsets></model>' % (n, y, z)))
    with open(os.path.join(k.d, 'knob.ac'), 'w') as f:
        f.write(M.ac([M.case('body', 0.0095, 0.0, 0.012, 5), M.disc('top', 0.0095, 0.012, None, 5), M.tri('mark', 0.0095, 1, 0.0125)]))
    knob('d175-knob', Y(122), -0.0705, 'sim/model/x57/d175-page', [(i, -45 + 45 * (i - 1) - 90) for i in range(1, 9)], range(1, 9))
    k.toggle('tg-fault', Y(232), -0.0710, 'sim/model/x57/d175-fault', True, 0.0005)
    for s_, g_, y_ in (('l', 388, 352), ('r', 427, 465)):
        k.lamp('start-' + s_, Y(g_), -0.0617, 0.0044, 'g', LAMP + 'start-' + s_, 0.0005)
        k.lamp('feather-' + s_, Y(g_), -0.0761, 0.0044, 'b', LAMP + 'feather-' + s_, 0.0005)
        k.lamp('fault-' + s_, Y(y_), -0.0696, 0.0044, 'y', LAMP + 'fault-' + s_, 0.0005)
    # the D175
    im = Image.new('RGB', (1024, 880), (168, 170, 174))
    g = ImageDraw.Draw(im)
    g.rounded_rectangle((16, 24, 1008, 866), 70, fill=(22, 23, 25))
    g.polygon(((70, 24), (950, 24), (1008, 150), (16, 150)), fill=(30, 31, 34))
    centred(g, 512, 826, 'MoTeC', 40, WHITE)
    centred(g, 900, 826, 'D175', 34, WHITE)
    for i in range(10):
        centred(g, 90 + i * 94, 124, str(i), 30, WHITE)
    k.objs.append(box('d175-box', -0.0685, -0.2130, 0.0685, -0.0965, 0.0065, k.picture('d175', im), (0.5, 0.5)))
    k.dark.append('d175-box')
    k.screen('d175', -0.0585, -0.1930, 0.0585, -0.1210, 0.0069)
    for i in range(10):
        y = -0.0685 + 0.137 * (90 + i * 94) / 1024.0
        k.objs.append(circle('can-%d' % i, y, -0.1030, 0.0021, 0.0070, 'lens-g.png'))
        k.xml.append(lit(('can-%d' % i,), LAMP + 'can-%d' % i, LENS['g'][0], DEAD))
    # the audio panel
    im = Image.new('RGB', (1024, 128), (26, 27, 30))
    g = ImageDraw.Draw(im)
    for i, w in enumerate(('MKR', 'COM1', 'COM2', 'COM3', 'NAV1', 'NAV2', 'DME', 'ADF', 'TEST')):
        x = 120 + i * 100
        g.rounded_rectangle((x - 40, 34, x + 40, 92), 8, fill=(52, 53, 56))
        centred(g, x, 62, w, 22, WHITE)
    k.objs.append(box('audio', -0.0795, -0.2330, 0.0795, -0.2150, 0.005, k.picture('audio', im), (0.5, 0.05)))
    k.dark.append('audio')


def finish(k, name):
    if k.dark:
        k.xml.append(M.effect('x57-face', k.dark))
    if k.black:
        k.xml.append(M.effect('x57-case', k.black))
    if k.steel:
        k.xml.append(M.effect('x57-steel', k.steel))
    if k.red:
        k.xml.append(M.effect('x57-red', k.red))
    with open(os.path.join(k.d, name + '.ac'), 'w') as f:
        f.write(M.ac(k.objs))
    with open(os.path.join(k.d, name + '.xml'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<!-- written by fdm/x57panel.py -->\n<PropertyList>\n<path>%s.ac</path>\n%s\n</PropertyList>\n' % (name, '\n'.join(k.xml)))


def overhead(d):
    """the overhead, ICD slides 43, 45 and 61, hung from the cabin's top where the Tecnam's console was. Places from the 3x crop from (765, 75) of fig. 4"""
    k = Kit(d)
    O = lambda x, y: ((x - 505) * 0.000306, (240 - y) * 0.000293)
    im = Image.new('RGB', (64, 64), (226, 220, 200))
    # its housing and plate, traced from fig. 4: the plate tapers to the foot, with a curved notch at the top and the foot
    housing = chaikin([O(x, y) for x, y in ((-80, -60), (-80, 50), (-30, 230), (30, 360), (90, 450), (160, 490), (310, 538), (710, 538), (815, 505), (875, 475), (920, 390), (980, 260), (1030, 140), (1060, 50), (1070, -60))])
    k.objs.append(outline('over-box', housing, 0.0, k.picture('cream', im), -0.05))
    k.dark.append('over-box')
    im = Image.new('RGB', (2048, 940), (15, 15, 17))
    g = ImageDraw.Draw(im)
    P = lambda x, y: (x * 2048 / 1005.0 - 30, (y - 10) * 940 / 460.0)
    for a, b in (((208, 12), (222, 275)), ((812, 30), (765, 290)), ((60, 278), (910, 302))):
        g.line(P(*a) + P(*b), fill=(208, 208, 200), width=6)
    for x, y, w, n in ((150, 72, 'MAIN', 34), (150, 96, 'BATT A', 34), (850, 88, 'MAIN', 34), (850, 112, 'BATT B', 34), (505, 82, 'CRUISE TRACTION POWER', 38), (195, 300, 'ARM', 22), (782, 316, 'ARM', 22), (195, 352, 'DISARM', 20), (782, 368, 'DISARM', 20),
                       (90, 132, 'ON', 26), (90, 178, 'OFF', 26), (252, 106, 'ON', 24), (252, 154, 'OFF', 24), (252, 222, 'ON', 24), (252, 264, 'OFF', 24), (762, 124, 'ON', 24), (752, 170, 'OFF', 24),
                       (742, 238, 'ON', 24), (736, 280, 'OFF', 24), (908, 156, 'ON', 26), (900, 200, 'OFF', 26), (150, 410, 'LEFT', 19), (150, 428, 'CMC A', 19), (250, 410, 'LEFT', 19), (250, 428, 'CMC B', 19),
                       (725, 426, 'RIGHT', 19), (725, 444, 'CMC A', 19), (815, 426, 'RIGHT', 19), (815, 444, 'CMC B', 19)):
        centred(g, *P(x, y), w, n, WHITE)
    for x, y, w in ((392, 130, 'LA'), (390, 250, 'LB'), (615, 132, 'RA'), (603, 250, 'RB')):
        a, b = P(x, y)
        g.rectangle((a - 62, b - 46, a + 62, b + 46), fill=(4, 4, 5))
        centred(g, a, b, w, 74, WHITE)
    # the high lift section between the long levers [slide 61]
    a, b = P(365, 304)
    c, d_ = P(655, 415)
    g.rectangle((a, b, c, d_), fill=(6, 6, 7))
    centred(g, *P(510, 318), 'HIGH-LIFT TRACTION POWER', 24, WHITE)
    for x, y, w, n in ((430, 344, 'H-LA', 18), (430, 376, 'H-LB', 18), (590, 344, 'H-RA', 18), (590, 376, 'H-RB', 18), (510, 346, 'ON', 18), (510, 372, 'OFF', 18), (470, 402, 'BUS A', 19), (550, 402, 'BUS B', 19)):
        centred(g, *P(x, y), w, n, WHITE)
    plate_ = chaikin([O(x, y) for x, y in ((15, 35), (35, 8), (200, 5), (350, 12), (380, 25), (410, 52), (520, 55), (610, 50), (640, 32), (680, 20), (810, 22), (975, 28), (995, 45), (985, 80), (950, 220), (910, 310), (865, 410),
                                           (840, 465), (810, 470), (610, 450), (550, 430), (380, 425), (350, 440), (160, 450), (125, 430), (80, 340), (35, 220), (15, 70))], 1)
    k.objs.append(outline('over-face', plate_, 0.0015, k.picture('overhead-face', im), uv=lambda p: (min(1, max(0, (p[0] + 0.150) / 0.300)), min(1, max(0, (p[1] + 0.0675) / 0.135)))))
    k.dark.append('over-face')
    X = 0.0015
    for n, x, y, p_ in (('tg-batt-a', 155, 175, 'fdm/jsbsim/ctl/bus-a'), ('tg-batt-b', 845, 195, 'fdm/jsbsim/ctl/bus-b')):
        k.toggle(n, *O(x, y), p_, True, X)
    for n, x, y, p_ in (('tg-la', 305, 150, 'tc-la'), ('tg-lb', 305, 262, 'tc-lb'), ('tg-ra', 700, 165, 'tc-ra'), ('tg-rb', 682, 275, 'tc-rb')):
        k.toggle(n, *O(x, y), SW + p_, False, X, model='long')
    for n, x, y, p_ in (('tg-arm-la', 150, 355, 'arm-la'), ('tg-arm-lb', 250, 355, 'arm-lb'), ('tg-arm-ra', 725, 370, 'arm-ra'), ('tg-arm-rb', 815, 372, 'arm-rb')):
        k.toggle(n, *O(x, y), SW + p_, True, X)
    for n, x, y, p_ in (('tg-hl-a', 470, 362, 'hl-tc-a'), ('tg-hl-b', 550, 362, 'hl-tc-b')):
        k.toggle(n, *O(x, y), SW + p_, False, X)
    for n, x, y in (('la', 450, 135), ('lb', 445, 245), ('ra', 555, 135), ('rb', 545, 250)):
        k.led2(n, *O(x, y), 0.0030, X)
    for n, x, y in (('hl-la', 395, 344), ('hl-lb', 395, 376), ('hl-ra', 625, 344), ('hl-rb', 625, 376)):
        k.led2(n, *O(x, y), 0.0022, X)
    M.screw_picture(os.path.join(d, 'screw-s.png'), 190)
    k.objs.append(M.screws('over-screws', 'screw-s.png', [O(x, y) + (0.0034,) for x, y in ((58, 38), (338, 40), (677, 50), (948, 62), (342, 428), (622, 437))], x0=0.0015))
    k.dark.append('over-screws')
    finish(k, 'overhead')
    return '<model><path>overhead.xml</path><offsets><x-m>0.395</x-m><y-m>0</y-m><z-m>0.375</z-m><pitch-deg>50</pitch-deg></offsets></model>'


TECNAM_FIT = 'sim/model/x57/tecnam-lights'           # x57tecnam.FIT: the INSTR rocker takes the blank's place when fitted
# the breaker panels [20240010539 fig. 4; ICD slide 28]: each plate in a cream moulding of its own, as the overhead's, under the glare shield's
# side: its face angled in from the wall at the door's hinge to the panel face's outer edge, so the glare shield's side overhangs its top, and
# behind the face the moulding fills only the corner's wedge: thin where it meets the panel and the wall, deepest between. The Tecnam cabin as
# modelled has only 17 cm from the panel's face to the door's hinge, where the photograph's cabin has about twice that, so the plate is
# 16 x 13 cm, its moulding's ends tucked against the panel and into the wall (slide 28's breakers make it nearer 20 x 13-15). Moulding and
# plate stand as high as the windshield's lower edge (z -0.23 by the panel) lets the moulding's top go
HOUSING_IN, HOUSING_OUT = (-2.744, 0.505), (-2.574, 0.560)     # the plate's line: (x, |y|) at the panel face's outer edge, and 6 mm off the wall at the door's hinge
HOUSING_LEAN = 0.0                                             # its lean back, deg
HOUSING_RIM = 0.012                                            # the moulding's rim round the plate's recess
PANEL_FACE_X = -2.746                                          # the instrument panel's face
CB_W, CB_Z = 0.072, -0.343                                     # the plate's half width to its outer breakers' edges (its height in proportion); its centre's height
CB_PIC = (1560, 1300)                                          # its picture: six columns 260 px apart, four rows 300 px apart, their labels below
CB_SHEAR = 18.0                                                # its rows and long edges rise toward the panel, its short sides upright: a parallelogram, as slide
                                                               # 28 (photographed with the rows level, the sides leaning about 16 deg) and fig. 4 show
WALL = lambda x, z=-0.4: (0.568 + (x + 2.58) * (0.028 / 0.26) - 0.25 * max(0.0, z + 0.29)
                          - 0.08 * max(0.0, -0.54 - z))      # the side walls' |y| there, narrowing toward the sill and toward the floor
LIGHTS_Y = (('LANDING', 0.214), ('TAXI', 0.245), ('NAV', 0.276), ('STROBE', 0.307))      # the X-57's photograph [ICD slide 11]: then a blank
BLANK_Y = 0.338                                       # where the P2006T has INSTR LIGHT [AFM fig. 11]: nothing shows there unless the Tecnam's lights are fitted
FLAPS_Y = 0.162                                       # the Tecnam's own place, which the photograph shows: lever 0.107, dial 0.162


def switch_plate(d, side):
    """the outline of the Tecnam's raised switch plate on the left or right of the slope, in lower.xml's frame (y across, z up
    the slope): its face is a fan of triangles 1.1 mm proud of the slope. NASA's black plate replaced the left one, the right one
    is the Tecnam's [ICD slide 11]"""
    from collections import Counter, defaultdict
    t = math.radians(-21)
    c_, s_ = math.cos(t), math.sin(t)
    for name, V, polys in M_objects(os.path.join(d, '..', 'panel.ac')):
        if name != 'panel4':
            continue
        q = np.c_[V[:, 0], -V[:, 2], V[:, 1]] - np.array((0.0135, 0, -0.285))
        Q = np.c_[q[:, 0] * c_ - q[:, 2] * s_, q[:, 1], q[:, 0] * s_ + q[:, 2] * c_]
        edges = Counter()
        for p in polys:
            T = Q[p]
            if abs(T[:, 0].mean() - 0.0011) > 0.0003 or abs(T[:, 2]).max() > 0.06 or (T[:, 1].mean() < 0) != (side == 'l'):
                continue
            for k in range(1, len(p) - 1):
                tri = (p[0], p[k], p[k + 1])
                for a, b in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
                    edges[tuple(sorted((a, b)))] += 1
        adj = defaultdict(list)
        for (a, b), n in edges.items():
            if n == 1:
                adj[a].append(b)
                adj[b].append(a)
        start = next(iter(adj))
        loop, prev, cur = [start], None, start
        while True:
            cur, prev = [v for v in adj[cur] if v != prev][0], cur
            if cur == start:
                break
            loop.append(cur)
        return [(float(Q[i, 1]), float(Q[i, 2])) for i in loop]


def M_objects(path):
    out = []
    for o in re.split(r'(?=^OBJECT )', open(path, errors='replace').read(), flags=re.M):
        n, c = re.search(r'^name "([^"]+)"', o, re.M), re.search(r'^numvert (\d+)', o, re.M)
        if not (n and c) or int(c.group(1)) == 0:
            continue
        loc = re.search(r'^loc (.*)', o, re.M)
        L = o[c.end():].lstrip('\n').split('\n')
        V = np.array([[float(q) for q in l.split()[:3]] for l in L[:int(c.group(1))]])
        V = V + (np.array([float(q) for q in loc.group(1).split()]) if loc else 0.0)
        polys = [[int(l.split()[0]) for l in m.group(2).strip().split('\n')] for m in re.finditer(r'^refs (\d+)\n((?:\d+ \S+ \S+\n?)+)', o, re.M)]
        out.append((n.group(1), V, polys))
    return out


def lower(d):
    """the panels on the slope under the main ones, and the box below the right one, ICD slides 13, 22, 44 and 61. Local frame: on the slope, y across, z up it"""
    k = Kit(d)
    X = 0.0025
    H = 0.043

    def sheet(y0, y1, colour, z0=-H, z1=H):
        W, Hh = int((y1 - y0) * 8000), int((z1 - z0) * 8000)
        im = Image.new('RGB', (W, Hh), colour)
        return im, ImageDraw.Draw(im), (lambda y, z: ((y - y0) * 8000, (z1 - z) * 8000))

    def done(name, im, y0, y1, z0=-H, z1=H, x=X):
        k.objs.append(plate(name, y0, z0, y1, z1, x, k.picture(name, im)))
        k.dark.append(name)

    def on_plate(name, im, pts, box, screw):
        """a plate cut to the Tecnam's switch plate, on it: its face just under the controls, its edge down to the Tecnam's
        face (1.1 mm proud of the slope), a screw in each corner as in the photographs"""
        y0, y1, z0, z1 = box
        k.objs.append(outline(name, pts, X - 0.0001, k.picture(name, im), back=0.0011, uv=lambda p: ((p[0] - y0) / (y1 - y0), (p[1] - z0) / (z1 - z0)), side=(0.02, 0.5)))
        k.dark.append(name)
        ys = [p[0] for p in pts]
        a, b = min(ys), max(ys)
        top = lambda y: max(p[1] for p in pts if abs(p[0] - y) < 0.006)
        bot = lambda y: min(p[1] for p in pts if abs(p[0] - y) < 0.006)
        heads = [(y, f(y) + dz, 0.0030) for y in (a + 0.0085, b - 0.0085) for f, dz in ((top, -0.0075), (bot, 0.0075))]
        k.objs.append(M.screws(name + '-screws', screw, heads, x0=X - 0.0001))
        k.dark.append(name + '-screws')

    def box_of(pts):
        return min(p[0] for p in pts), max(p[0] for p in pts), min(p[1] for p in pts), max(p[1] for p in pts)
    M.screw_picture(os.path.join(d, 'screw-d.png'), 70)
    M.screw_picture(os.path.join(d, 'screw-s.png'), 190)
    left, right = switch_plate(d, 'l'), switch_plate(d, 'r')
    # left: ESS BATT and the converters, then the landing gear [20240006533 fig. 5], on NASA's plate in the Tecnam's plate's place
    y0, y1, z0, z1 = box_of(left)
    im, g, at = sheet(y0, y1, (20, 20, 22), z0, z1)
    gy = -0.163
    centred(g, *at(-0.291, -0.020), 'ESS BATT', 28, WHITE)
    for y, w in ((-0.2545, 'A'), (-0.218, 'B')):
        centred(g, *at(y, -0.018), 'DC', 26, WHITE)
        centred(g, *at(y, -0.024), 'CONV', 26, WHITE)
        centred(g, *at(y, -0.030), w, 26, WHITE)
    # heights as the photographs have them, inside the plate's edges here (top about +0.036, bottom -0.032): the lever and
    # UP / DN at the top, NOSE and TRANS with their names above (below the lever's knob, gear down), LH, RH and LT TEST with theirs below, LNDG GEAR down the side
    # ending in a bracket [20240006533 fig. 5, ICD slide 11]
    for y, z, w in ((gy + 0.030, 0.030, 'UP'), (gy + 0.030, 0.019, 'DN'), (gy, 0.004, 'NOSE'), (gy + 0.027, 0.004, 'TRANS'), (gy - 0.013, -0.027, 'LH'), (gy + 0.013, -0.027, 'RH'), (gy + 0.031, -0.027, 'LT TEST')):
        centred(g, *at(y, z), w, 24, WHITE)
    for i, c in enumerate('LNDG GEAR'):
        centred(g, *at(gy - 0.026, 0.029 - i * 0.0060), c, 24, WHITE)
    a, b, c_ = at(gy - 0.026, -0.0225), at(gy - 0.026, -0.0285), at(gy - 0.020, -0.0285)
    g.line((a[0], a[1], b[0], b[1], c_[0], c_[1]), fill=WHITE, width=3)
    on_plate('low-left', im, left, (y0, y1, z0, z1), 'screw-d.png')
    k.toggle('tg-ess', -0.291, 0.006, SW + 'ess-batt', False, X, model='ball')
    k.toggle('tg-dc-a', -0.2545, 0.006, SW + 'dc-conv-a', False, X, model='bus')
    k.toggle('tg-dc-b', -0.218, 0.006, SW + 'dc-conv-b', False, X, model='bus')
    k.toggle('tg-gear', gy, 0.026, 'controls/gear/gear-down', False, X, up=False, model='lever', big=True)
    k.lamp('ldg-nose', gy, -0.004, 0.0040, 'g', LAMP + 'ldg-nose', X)
    k.lamp('ldg-lh', gy - 0.013, -0.0165, 0.0040, 'g', LAMP + 'ldg-lh', X)
    k.lamp('ldg-rh', gy + 0.013, -0.0165, 0.0040, 'g', LAMP + 'ldg-rh', X)
    k.lamp('ldg-trans', gy + 0.027, -0.004, 0.0040, 'r', LAMP + 'ldg-trans', X)
    k.push('lt-test', gy + 0.031, -0.0165, 0.0042, X57 + 'lt-test', X)
    def rounded(y0, z0, y1, z1, r, n=6):
        """a rectangle's outline with its corners rounded to r"""
        pts = []
        for cy, cz, a0 in ((y1 - r, z1 - r, 0), (y0 + r, z1 - r, 90), (y0 + r, z0 + r, 180), (y1 - r, z0 + r, 270)):
            pts += [(cy + r * math.cos(math.radians(a0 + 90 * i / n)), cz + r * math.sin(math.radians(a0 + 90 * i / n))) for i in range(n + 1)]
        return pts

    def edged(name, pts, x, back, tex, box_):
        """a black anodised part: its face the picture, its edges black (the case's finish) whatever the light"""
        y0, y1, z0, z1 = box_
        k.objs.append(outline(name, pts, x, tex, uv=lambda p: ((p[0] - y0) / (y1 - y0), (p[1] - z0) / (z1 - z0))))
        k.dark.append(name)
        k.objs.append(outline(name + '-edge', pts, x - 0.0003, None, back=back))
        k.black.append(name + '-edge')

    def corner_screws(name, y0, z0, y1, z1, tex, dy=0.008, dz=0.007):
        k.objs.append(M.screws(name, tex, [(y, z, 0.0028) for y in (y0 + dy, y1 - dy) for z in (z0 + dz, z1 - dz)], x0=X - 0.0001))
        k.dark.append(name)
    # centre: the buses [20240006533 fig. 5], on a black plate in the console under the audio panel, its corners round and
    # screwed, the switches a little above its middle, each with a mark and its name under it [ICD slide 11]
    y0, y1, z0, z1 = -0.080, 0.080, -0.028, 0.040
    im, g, at = sheet(y0, y1, (26, 27, 30), z0, z1)
    names = (('CROSS', 'BUS A', 'cross-bus-a'), ('CROSS', 'BUS B', 'cross-bus-b'), ('WING', 'AVIONICS A', 'wing-av-a'), ('WING', 'AVIONICS B', 'wing-av-b'), ('AVIONICS', 'BUS A', 'av-bus-a'), ('AVIONICS', 'BUS B', 'av-bus-b'))
    for i, (a, b, n) in enumerate(names):
        y = -0.0535 + 0.0214 * i
        u, v = at(y, 0.0045)
        g.polygon(((u - 10, v + 8), (u + 10, v + 8), (u, v - 9)), fill=WHITE)
        centred(g, *at(y, -0.0045), a, 21, WHITE)
        centred(g, *at(y, -0.0115), b, 21, WHITE)
        k.toggle('tg-' + n, y, 0.017, SW + n, False, X, model='bus')
    edged('low-mid', rounded(y0, z0, y1, z1, 0.006), X - 0.0001, -0.004, k.picture('low-mid', im), (y0, y1, z0, z1))
    corner_screws('low-mid-screws', y0, z0, y1, z1, 'screw-d.png')
    # right: the Tecnam's own plate [ICD slide 11]: flap lever and dial, then LANDING, TAXI, NAV, STROBE and a blank, under LIGHTS
    y0, y1, z0, z1 = box_of(right)
    im, g, at = sheet(y0, y1, (176, 179, 183), z0, z1)
    ink = (40, 42, 46)
    for w, y in LIGHTS_Y:
        centred(g, *at(y, -0.031), w, 24, ink)
    a, b = at(LIGHTS_Y[0][1] - 0.012, -0.033), at(LIGHTS_Y[-1][1] + 0.012, -0.033)
    g.line((a[0], a[1], b[0], b[1]), fill=ink, width=4)
    centred(g, *at((LIGHTS_Y[0][1] + LIGHTS_Y[-1][1]) / 2, -0.034), 'LIGHTS', 22, ink)
    centred(g, *at(FLAPS_Y - 0.055, 0.024), 'UP', 22, ink)
    centred(g, *at(FLAPS_Y - 0.055, -0.024), 'FULL', 22, ink)
    on_plate('low-right', im, right, (y0, y1, z0, z1), 'screw-s.png')
    # where the P2006T has INSTR LIGHT the X-57's photograph has a blank; at the user's word nothing shows there, as if no switch belonged, until
    # the Tecnam's cockpit lights are fitted and their INSTR rocker takes the place (x57tecnam)
    # the data system [slide 22]: a black bracket's face, its corners round, each toggle's names on black tags above and under it
    y0, y1, z0, z1 = 0.21, 0.40, -0.097, -0.052
    im, g, at = sheet(y0, y1, (22, 23, 26), z0, z1)

    def tag(y, z, lines, n=20):
        h = 0.0042 * len(lines)
        a, b = at(y - 0.011, z + h / 2), at(y + 0.011, z - h / 2)
        g.rectangle((a[0], a[1], b[0], b[1]), fill=(8, 8, 9), outline=(70, 71, 75), width=2)
        for i, w in enumerate(lines):
            centred(g, *at(y, z + h / 2 - 0.0021 - 0.0042 * i), w, n, WHITE)
    for y, a_, b_ in ((0.258, ('DAU', 'ENABLE'), ('OFF',)), (0.306, ('RF', 'ENABLE'), ('OFF',)), (0.355, ('RECORD', 'START'), ('STOP',))):
        tag(y, -0.0600, a_)
        tag(y, -0.0890, b_)
    edged('rec-box', rounded(y0, z0, y1, z1, 0.005), X, -0.03, k.picture('rec-box', im), (y0, y1, z0, z1))
    for n, y, p_ in (('tg-dau', 0.258, 'dau-enable'), ('tg-rf', 0.306, 'rf-enable'), ('tg-rec', 0.355, 'record-enable')):
        k.toggle(n, y, -0.0745, X57 + p_, False, X)
    k.lamp('time-lock', 0.2267, -0.0745, 0.0040, 'g', LAMP + 'time-lock', X)
    # RECORD ENABLE's tri-colour light: green ready, red archiving, amber a fault
    y, z = 0.384, -0.0745
    k.objs.append(lathe('rec-ring', ((0.0054, X), (0.0054, X + 0.0022), (0.0040, X + 0.0022)), y, z))
    k.black.append('rec-ring')
    for c, v_ in (('g', 1), ('r', 2), ('a', 3)):
        n = 'rec-' + c
        k.objs.append(circle(n, y, z, 0.0040, X + 0.0024, 'lens-%s.png' % c))
        k.xml.append(lit((n,), LAMP + 'rec', LENS[c][0], DEAD))
        k.xml.append('<animation><type>select</type><object-name>%s</object-name><condition><equals><property>%srec-colour</property><value>%d</value></equals></condition></animation>' % (n, X57, v_))
    finish(k, 'lower')
    return '<model><path>lower.xml</path><offsets><x-m>0.0135</x-m><y-m>0</y-m><z-m>-0.285</z-m><pitch-deg>-21</pitch-deg></offsets></model>'


def yoke_button(base):
    """the high-lift disarm: a red button on the yoke's left horn [ICD slide 64]"""
    p = os.path.join(base, 'Instruments', 'yokes', 'yoke.ac')
    t = open(p, errors='replace').read()
    o = next(q for q in re.split(r'(?=^OBJECT )', t, flags=re.M) if re.search(r'^name "yoke"', q, re.M))
    l = o.split('\n')
    i = next(j for j, q in enumerate(l) if q.startswith('numvert'))
    v = np.array([q.split()[:3] for q in l[i + 1:i + 1 + int(l[i].split()[1])]], float) + np.array((re.findall(r'^loc (.*)', o, re.M) or ['0 0 0'])[0].split(), float)
    P = np.c_[v[:, 0], -v[:, 2], v[:, 1]]
    top = P[(P[:, 1] < P[:, 1].min() + 0.25 * np.ptp(P[:, 1])) & (P[:, 2] > P[:, 2].max() - 0.3 * np.ptp(P[:, 2]))]
    y, z = top[:, 1].mean(), top[:, 2].mean()
    x = top[:, 0].max()
    b = lathe('hl-button', ((0.0075, x), (0.0075, x + 0.003), (0.0060, x + 0.0036), (0.0, x + 0.0038)), y, z)
    head, rest = t.split('OBJECT poly', 1) if 'OBJECT world' not in t else (t, '')
    n = int(re.search(r'^kids (\d+)', t, re.M).group(1))
    t = re.sub(r'^kids %d' % n, 'kids %d' % (n + 1), t, count=1, flags=re.M).rstrip('\n') + '\n' + b + '\n'
    open(p, 'w', errors='replace').write(t)
    p = os.path.join(base, 'Instruments', 'yokes', 'yoke.xml')
    x = open(p, errors='replace').read()
    x = x.replace('<object-name>yoke</object-name>', '<object-name>yoke</object-name><object-name>hl-button</object-name>')
    x = x.replace('</PropertyList>', M.effect('x57-red', ('hl-button',)) + '<animation><type>pick</type><object-name>hl-button</object-name><action><button>0</button><repeatable type="bool">false</repeatable><binding><command>property-assign</command><property>%shl-yoke</property><value>1</value></binding></action></animation>\n</PropertyList>' % X57)
    open(p, 'w', errors='replace').write(x)


def place(t, base):
    """the Tecnam's panel list to the real aircraft's: no right seat instruments, the real order, its overhead console and rocker rows out"""
    block = lambda path: r'<model>(?:(?!</model>).)*?<path>%s</path>.*?</model>\s*' % path
    t = re.sub(block(r'Overhead/overhead\.xml'), '', t, flags=re.S)
    t = re.sub(block(r'Instruments/gear/gear\.xml'), '', t, flags=re.S)
    t = re.sub(block(r'Instruments/switch/(?:alternator|battery|instr-lights|other-lights)\.xml'), '', t, flags=re.S)
    t = re.sub(r'<model>(?:(?!</model>).)*?<path>Instruments/(?:ai/ai|alt/alt|asi/asi)\.xml</path>\s*<offsets>(?:(?!</model>).)*?<y-m>\s*0\.\d+\s*</y-m>(?:(?!</model>).)*?</model>\s*', '', t, flags=re.S)
    t = re.sub(r'(<path>Instruments/hi/hi\.xml</path>\s*<offsets>(?:(?!</offsets>).)*?<y-m>)\s*-0\.146\s*(</y-m>)', r'\1 -0.341 \2', t, flags=re.S)
    for n, y in (('landing', LIGHTS_Y[0][1]), ('taxi', LIGHTS_Y[1][1]), ('nav', LIGHTS_Y[2][1])):
        t = re.sub(r'(<path>Instruments/switch/%s-lights\.xml</path>\s*<offsets>(?:(?!</offsets>).)*?<y-m>)[^<]*(</y-m>)' % n, r'\g<1> %.3f \2' % y, t, flags=re.S)
    t = re.sub(r'(<path>Instruments/flaps/flaps\.xml</path>\s*<offsets>\s*<x-m>)[^<]*(</x-m>\s*<y-m>)[^<]*(</y-m>)', r'\g<1> 0.0165 \g<2> %.3f \3' % FLAPS_Y, t, flags=re.S)
    p = os.path.join(base, 'Instruments', 'switch')
    s = open(os.path.join(p, 'nav-lights.xml'), errors='replace').read().replace('controls/lighting/nav-lights', 'controls/lighting/strobe')
    open(os.path.join(p, 'strobe-lights.xml'), 'w', errors='replace').write(s)
    t = t.replace('</PropertyList>', '<model><path>Instruments/switch/strobe-lights.xml</path><offsets><x-m>0.019</x-m><y-m>%.3f</y-m><z-m>-0.290</z-m><pitch-deg>-21</pitch-deg></offsets></model>\n</PropertyList>' % LIGHTS_Y[3][1])
    p = os.path.join(base, 'Instruments', 'vsi', 'vsi.xml')                       # the VSI reads nasal/cockpit.nas's, at 0 from the start
    x = open(p, errors='replace').read()
    open(p, 'w', errors='replace').write(x.replace('instrumentation/vertical-speed-indicator/indicated-speed-fpm', 'instrumentation/x57/vsi-fpm'))
    p = os.path.join(base, 'Instruments', 'turn', 'turn.xml')                     # and the turn coordinator's needle, cockpit.nas's too
    x = open(p, errors='replace').read()
    open(p, 'w', errors='replace').write(x.replace('instrumentation/turn-indicator/indicated-turn-rate', 'instrumentation/x57/turn-rate'))
    p = os.path.join(base, 'Instruments', 'flaps', 'flaps.xml')
    x = open(p, errors='replace').read()
    x = re.sub(r'(<object-name>needle</object-name>\s*)<property>controls/flight/flaps</property>\s*<factor> 70 </factor>\s*<center>.*?</center>',
               r'\1<property>fdm/jsbsim/ctl/flap-deg</property><factor>6</factor><center><x-m>0.00175</x-m><y-m>0</y-m><z-m>0</z-m></center>', x, count=1, flags=re.S)
    # the flap switch [ICD slide 58]: held down the flaps run down, held up they run up, let go they stop where they are, and the lever springs
    # back to the middle. The Tecnam's lever stands for it: it tips 20 deg while held. Press and hold just above it to raise the flaps, just
    # below it to lower them (two clear boxes, switch.ac); the wheel nudges them 2.5 deg; [ and ] step 0, 10, 20, 30 (x57cockpit)
    x = re.sub(r'(<object-name>flapslever</object-name>\s*)<property>controls/flight/flaps</property>\s*<factor> 45 </factor>',
               r'\1<property>sim/model/x57/flap-switch</property><factor>20</factor>', x, count=1)
    nasal = lambda b, src: ('<action><button>%d</button><repeatable type="bool">false</repeatable><binding><command>nasal</command><script>%s</script>'
                            '</binding>%s</action>' % (b, src, '<mod-up><binding><command>nasal</command><script>x57cockpit.flap_switch(0);</script></binding></mod-up>'
                                                     if 'flap_switch' in src else ''))
    tip = ('<hovered><binding><command>set-tooltip</command><tooltip-id>x57-flaps</tooltip-id><label>Flaps %s: hold above to raise, below to lower</label>'
           '<property>fdm/jsbsim/ctl/flap-deg</property><mapping>nasal</mapping><script>return sprintf("%d deg", math.round(arg[0]));</script></binding></hovered>')
    wheel = nasal(3, 'x57cockpit.flap_nudge(-1);') + nasal(4, 'x57cockpit.flap_nudge(1);')
    x = x.replace('</PropertyList>', '<animation><type>pick</type><object-name>flapslever</object-name>' + wheel + tip + '</animation>\n'
                  '<model><path>switch.xml</path></model>\n</PropertyList>')
    k = Kit(os.path.dirname(p))
    Image.new('RGBA', (8, 8), (0, 0, 0, 0)).save(os.path.join(os.path.dirname(p), 'clear.png'))
    k.objs.append(box('flap-up', -0.078, 0.0045, -0.033, 0.034, 0.030, 'clear.png', (0.5, 0.5), 0.0))
    k.objs.append(box('flap-down', -0.078, -0.034, -0.033, -0.0045, 0.030, 'clear.png', (0.5, 0.5), 0.0))
    k.xml.append('<animation><type>pick</type><object-name>flap-up</object-name>' + nasal(0, 'x57cockpit.flap_switch(-1);') + wheel + tip + '</animation>')
    k.xml.append('<animation><type>pick</type><object-name>flap-down</object-name>' + nasal(0, 'x57cockpit.flap_switch(1);') + wheel + tip + '</animation>')
    k.xml.append('<effect><inherits-from>Effects/model-transparent</inherits-from><object-name>flap-up</object-name><object-name>flap-down</object-name></effect>')
    k.xml.append('<animation><type>noshadow</type><object-name>flap-up</object-name><object-name>flap-down</object-name></animation>')
    finish(k, 'switch')
    open(p, 'w', errors='replace').write(x)
    yoke_button(base)
    # the yoke shows, as in the photograph; a click on its column's base hides it from the panel
    p = os.path.join(base, 'Instruments', 'yokes', 'yoke.xml')
    s = open(p, errors='replace').read().replace('sim/multiplay/generic/bool[0]', 'sim/model/x57/show-yoke')
    s = s.replace('</PropertyList>', '<animation><type>pick</type><object-name>base</object-name><action><button>0</button><repeatable type="bool">false</repeatable><binding><command>property-toggle</command><property>sim/model/x57/show-yoke</property></binding></action></animation>\n</PropertyList>')
    open(p, 'w', errors='replace').write(s)
    return t


LEFT_CB = (((25, 'LANDING|GEAR', 'cb-gear'), (5, 'LG|RELAY', 0), (7.5, 'FLAP|MOTOR', 'cb-flap'), (7.5, 'DG|STAB IND', 0), (3, 'TURN&BANK|LH', 0), (5, 'HORIZ|LH', 0)),
           ((10, 'COM2|GARMIN 650', 0), (5, 'NAV1|GARMIN 650', 0), (5, 'ANNUNC.|PANEL', 0), (5, 'OAT /|FLAP IND', 0), (3, 'ENCODER|AK A30', 0), (5, 'TDR|GTX-328', 0)),
           ((2, 'BMS A|POWER', 0), (2, 'TE-L', 0), (4, 'PROP|CONT. L', 'cb-prop-l'), (4, 'PROP|CONT. R', 'cb-prop-r'), (3, 'MOTEC|DISPLAY', 0), (5, 'AUDIO PANEL|GMA-340', 0)),
           ((7.5, 'CMC LA|FOBE LCA', 0), (7.5, 'CMC RA|FOBE RCA', 0), (2, 'FOBE A', 0), (10, 'BCM A|28V CONV', 0), (5, 'MOTEC|ACL', 0), (2, 'AIR DATA|PROBE', 0)))
RIGHT_CB = (((1, 'STALL|WARNING', 0), (2, 'LG|LIGHT', 0), (2, 'TE-R', 0), (2, 'BMS B|POWER', 0), (1, 'AUDIO|ANNUNC.', 0), (1, 'G METER', 0)),
            ((25, 'COM1|UHF', 0), (2, 'AVIONICS|RELAYS', 0), (3, 'CROSS|RELAYS', 0), (10, 'BCM B|28V CONV', 0), (2, 'FOBE B', 0), (2, 'HIPI|B', 0)),
            ((3, 'RUDDER|TRIM', 0), (7.5, 'STROBE|LIGHT', 0), (3, 'CABIN|FAN', 0), (1, 'CABIN|DOOR', 0), (5, 'INSTR|LIGHT', 0), (3, 'NAV|LIGHT', 0)),
            ((7.5, 'CMC RB|FOBE RCB', 0), (7.5, 'CMC LB|FOBE LCB', 0), (10, 'HLMC B', 0), (2, 'NOVATEL|GPS', 0), (2, 'HIPC B', 0), (2, 'HIPC A', 0)))


PARK_AT = (-2.4835, 0.0, -0.660)                       # the parking brake's knob: the pedestal's lower aft face, at its foot [P2006T AFM fig. 7, item 11]
PARK_PULL = 0.020                                      # pulled out this far when set


def park_brake(d):
    """the parking brake: a round knob at the foot of the pedestal's face [P2006T AFM 7-12, fig. 7, item 11; 7-10: it works a valve on the cabin
    floor that holds the brakes' pressure]. NASA's photographs stop above it; the X-57 kept the P2006T's brakes, and its takeoff procedure has
    a brake release [ICD slide 69]. Pulled out when set; a click sets or releases it (FlightGear's B key still does too)"""
    k = Kit(d)
    k.objs.append(lathe('park-bezel', ((0.0150, 0.0), (0.0150, 0.0015), (0.0120, 0.0028), (0.0055, 0.0028))))
    k.black.append('park-bezel')
    k.objs.append(lathe('park-stem', ((0.0040, -PARK_PULL - 0.004), (0.0040, 0.0105))))     # long enough to reach into the pedestal when pulled out
    k.steel.append('park-stem')
    k.objs.append(lathe('park-knob', ((0.0040, 0.0100), (0.0110, 0.0100), (0.0128, 0.0125), (0.0128, 0.0215), (0.0112, 0.0240), (0.0050, 0.0250))))
    k.black.append('park-knob')
    knob = '<object-name>park-stem</object-name><object-name>park-knob</object-name>'
    prop = '/controls/gear/brake-parking'
    k.xml.append('<animation><type>translate</type>%s<property>%s</property><interpolation><entry><ind>0</ind><dep>0</dep></entry>'
                 '<entry><ind>1</ind><dep>%g</dep></entry></interpolation><axis><x>1</x><y>0</y><z>0</z></axis></animation>' % (knob, prop, PARK_PULL))
    k.xml.append('<animation><type>pick</type>%s<action><button>0</button><repeatable type="bool">false</repeatable><binding><command>property-toggle</command>'
                 '<property>%s</property></binding></action><hovered><binding><command>set-tooltip</command><tooltip-id>x57-park</tooltip-id>'
                 '<label>Parking brake: %%s</label><property>%s</property><mapping>on-off</mapping></binding></hovered></animation>' % (knob, prop, prop))
    finish(k, 'park')
    org = [float(q) for q in re.search(r'<x-m>\s*([-\d.]+)\s*</x-m>\s*<y-m>\s*([-\d.]+)\s*</y-m>\s*<z-m>\s*([-\d.]+)', open(os.path.join(d, '..', 'all-panels.xml')).read()).groups()]
    return ['<model><path>park.xml</path><offsets><x-m>%.4f</x-m><y-m>%.4f</y-m><z-m>%.4f</z-m></offsets></model>' % tuple(a - b for a, b in zip(PARK_AT, org))]


def offset(loop, d):
    """a closed outline (y, z) moved inward by d: each point along its corner's inward normal"""
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(loop, loop[1:] + loop[:1]))
    sgn = 1.0 if area > 0 else -1.0
    out = []
    for i, p in enumerate(loop):
        a, b = loop[i - 1], loop[(i + 1) % len(loop)]
        e1 = np.array((p[0] - a[0], p[1] - a[1])); e2 = np.array((b[0] - p[0], b[1] - p[1]))
        n1 = np.array((-e1[1], e1[0])) / (np.linalg.norm(e1) or 1); n2 = np.array((-e2[1], e2[0])) / (np.linalg.norm(e2) or 1)
        nv = (n1 + n2) * sgn
        nv /= np.linalg.norm(nv) or 1
        out.append((p[0] + nv[0] * d, p[1] + nv[1] * d))
    return out


def ring(loop, d=0.0, n=128):
    """a closed outline (y, z), moved inward by d, resampled where n rays from the origin (the plate's centre) cross it, counter-clockwise"""
    loop = offset(loop, d) if d else loop
    pts = []
    for k_ in range(n):
        u = np.array((math.cos(2 * math.pi * k_ / n), math.sin(2 * math.pi * k_ / n)))
        best = None
        for p, q in zip(loop, loop[1:] + loop[:1]):
            p, q = np.array(p), np.array(q)
            e = q - p
            den = u[0] * e[1] - u[1] * e[0]
            if abs(den) < 1e-12:
                continue
            t = (p[0] * e[1] - p[1] * e[0]) / den
            s_ = (p[0] * u[1] - p[1] * u[0]) / den
            if t > 0 and -1e-9 <= s_ <= 1 + 1e-9 and (best is None or t < best):
                best = t
        pts.append((float(u[0] * best), float(u[1] * best)))
    return pts


def side_panels(d):
    """the breakers, on a plate in a cream moulding each side between the panel and the door's frame (HOUSING_*). The left plate is ICD slide 28's;
    the right, its names from 20240009560 fig. 5, is in my order. Pull one by clicking it: four do something"""
    out = []
    W = CB_W
    H_ = W * CB_PIC[1] / CB_PIC[0]                               # the picture's proportions
    s_ = 0.62                                                    # the breakers' hardware: collars about 11 mm across, as slide 28's
    org = [float(q) for q in re.search(r'<x-m>\s*([-\d.]+)\s*</x-m>\s*<y-m>\s*([-\d.]+)\s*</y-m>\s*<z-m>\s*([-\d.]+)', open(os.path.join(d, '..', 'all-panels.xml')).read()).groups()]
    lean = math.radians(HOUSING_LEAN)
    for side, rows, sg in (('l', LEFT_CB, -1), ('r', RIGHT_CB, 1)):
        A = np.array((HOUSING_IN[0], sg * HOUSING_IN[1], CB_Z))
        B = np.array((HOUSING_OUT[0], sg * HOUSING_OUT[1], CB_Z))
        n = np.array((sg * (A[1] - B[1]), -sg * (A[0] - B[0]), 0.0)) * -1.0
        n /= np.linalg.norm(n)                                   # the face's normal, level: into the cabin, aft
        h = math.atan2(n[1], n[0])
        ex = np.array((math.cos(h) * math.cos(lean), math.sin(h) * math.cos(lean), math.sin(lean)))      # the plate's out, leaning back
        ey = np.array((-math.sin(h), math.cos(h), 0.0))                                                  # its right, as the pilot sees it
        ez = np.cross(ex, ey)
        T = (A + B) / 2
        O = T                                                    # the plate's centre: the model's origin
        yA, yB = (A - O) @ ey, (B - O) @ ey
        sA = math.copysign(1.0, yA - yB)                         # +1 where the plate's right, as the pilot sees it, is toward the panel
        kz = math.tan(math.radians(CB_SHEAR)) * sA               # its rows rise toward the panel: z grows by kz per metre of y
        shear = lambda pts: [(y, z + kz * y) for y, z in pts]
        k = Kit(d)
        im = Image.new('RGB', CB_PIC, (14, 15, 16))
        g = ImageDraw.Draw(im)
        for r_, row in enumerate(rows):
            for c, (amps, word, prop) in enumerate(row):
                u, v_ = 130 + 260 * c, 110 + 300 * r_
                parts = word.split('|')
                wd = max(g.textlength(q, font=font(30)) for q in parts) + 22
                g.rectangle((u - wd / 2, v_ + 78, u + wd / 2, v_ + 86 + 36 * len(parts)), outline=WHITE, width=3)
                for i, part in enumerate(parts):
                    centred(g, u, v_ + 100 + 34 * i, part, 30, WHITE)
                y = -W + 2 * W * u / CB_PIC[0]
                z = H_ - 2 * H_ * v_ / CB_PIC[1] + kz * y
                name = 'cb-%s-%d-%d' % (side, r_, c)
                k.objs.append(lathe(name + '-ring', ((0.0085 * s_, 0.0), (0.0085 * s_, 0.0016), (0.0062 * s_, 0.0020)), y, z))
                k.black.append(name + '-ring')
                k.objs.append(lathe(name, ((0.0056 * s_, 0.0), (0.0056 * s_, 0.0034)), y, z))
                k.black.append(name)
                k.objs.append(circle(name + '-top', y, z, 0.0056 * s_, 0.0035, 'cb-%g.png' % amps))
                k.dark.append(name + '-top')
                p_ = SW + prop if prop else X57 + 'cb/%s-%d-%d' % (side, r_, c)
                pull = 0.008 * s_
                table = ((0, pull), (1, 0.0)) if prop else ((0, 0.0), (1, pull))
                both = '<object-name>%s</object-name><object-name>%s-top</object-name>' % (name, name)
                k.xml.append('<animation><type>translate</type>%s<property>%s</property><interpolation>%s</interpolation><axis><x>1</x><y>0</y><z>0</z></axis></animation>'
                             % (both, p_, ''.join('<entry><ind>%g</ind><dep>%g</dep></entry>' % e for e in table)))
                k.xml.append('<animation><type>pick</type>%s<action><button>0</button><repeatable type="bool">false</repeatable><binding><command>property-toggle</command><property>%s</property></binding></action></animation>' % (both, p_))
                q = Image.new('RGB', (96, 96), (22, 23, 25))
                centred(ImageDraw.Draw(q), 48, 48, {7.5: '7.5'}.get(amps, '%g' % amps), 52, WHITE)
                q.save(os.path.join(d, 'cb-%g.png' % amps))
        pw, ph = W + 0.006, H_ + 0.006
        c4 = [(-pw, -ph + kz * -pw), (pw, -ph + kz * pw), (pw, ph + kz * pw), (-pw, ph + kz * -pw)]       # the plate: a parallelogram, its sides upright
        v = [(-0.02, y, z) for y, z in c4] + [(0.0, y, z) for y, z in c4]
        sides = [surf((i, (i + 1) % 4, 4 + (i + 1) % 4, 4 + i), ((0.02, 0.02),) * 4) for i in range(4)]
        k.objs.append(obj('side-box', v, [surf((4, 5, 6, 7), ((0, 0), (1, 0), (1, 1), (0, 1)))] + sides, k.picture('side-' + side, im)))
        k.dark.append('side-box')
        # the housing: a moulding round the plate and no more, as the overhead's, filling only the wedge of space between its face and the corner
        # behind (the wall and the panel's face): thin where its face meets them, deepest between. From the wall and the panel over a rounded
        # edge onto a narrow face 8 mm in front of the plate's, and over a sloping lip down into the recess the plate sits in
        pw, ph = W + 0.006, H_ + 0.006
        def rounded(hw, hh, r):
            return chaikin([(-hw + r, -hh), (hw - r, -hh), (hw, -hh + r), (hw, hh - r), (hw - r, hh), (-hw + r, hh), (-hw, hh - r), (-hw, -hh + r)], 3)
        opening = shear(rounded(pw + 0.010, ph + 0.010, 0.008))
        outline_ = shear(rounded(pw + 0.010 + HOUSING_RIM, ph + 0.010 + HOUSING_RIM, 0.018))
        xf = 0.008
        loops = [(xf - 0.010, outline_, 0.0), (xf - 0.010, outline_, 0.0), (xf - 0.0035, outline_, 0.0035), (xf, outline_, 0.008),
                 (xf, opening, 0.0), (xf - 0.006, opening, 0.005), (-0.002, opening, 0.0085)]
        L = [[(x, y, z) for y, z in ring(loop, dd)] for x, loop, dd in loops]
        loc = lambda P: tuple(float(c) for c in ((P - O) @ ex, (P - O) @ ey, (P - O) @ ez))
        nh = np.array((ex[0], ex[1], 0.0)) / math.hypot(ex[0], ex[1])
        for i, f in enumerate(L[0]):                             # back, level, to the wall or the panel's face behind
            P = O + ex * f[0] + ey * f[1] + ez * f[2]
            t = 0.0
            while t < 0.12:
                q = P - nh * t
                if abs(q[1]) >= WALL(q[0], q[2]) or q[0] <= PANEL_FACE_X:
                    break
                t += 0.001
            L[0][i] = loc(P - nh * t)
        cream = k.picture('housing-' + side, Image.new('RGB', (32, 32), (212, 198, 164)))
        n_ = len(L[0])
        for name, a_, b_, flag in (('housing-side', 0, 3, '0x30'), ('housing', 3, 4, '0x20'), ('housing-lip', 4, 6, '0x30')):
            v = [p for l in L[a_:b_ + 1] for p in l]
            k.objs.append(obj(name, v, [surf((j * n_ + i, j * n_ + (i + 1) % n_, (j + 1) * n_ + (i + 1) % n_, (j + 1) * n_ + i), flag=flag)
                                        for j in range(b_ - a_) for i in range(n_)], cream))
            k.dark.append(name)
        finish(k, 'breakers-' + side)
        x_c, y_c, z_c = O[0] - org[0], O[1] - org[1], O[2] - org[2]
        out.append('<model><path>breakers-%s.xml</path><offsets><x-m>%.4f</x-m><y-m>%.4f</y-m><z-m>%.4f</z-m><heading-deg>%.2f</heading-deg><pitch-deg>%.2f</pitch-deg></offsets></model>'
                   % (side, x_c, y_c, z_c, math.degrees(h), -HOUSING_LEAN))
        print('X57PANEL breakers-%s: plate %.0f x %.0f mm, its centre (%.3f, %.3f, %.3f), heading %.1f' % (
            side, 2000 * (W + 0.006), 2000 * (H_ + 0.006), O[0], O[1], O[2], math.degrees(h)))
    return out


def clicks(base):
    """every panel control makes its sound: a pick also flips a property the sound file listens to. Rockers and toggles click; buttons, breakers and knobs tick, on press and on release.
    The gear and flap levers have sounds of their own, and what repeats while held has none"""
    n = dict(rocker=0, push=0)

    def one(m):
        b = m.group(0)
        a = re.search(r'<action>.*?</action>', b, re.S)            # never the <hovered> bindings: those run on every mouse-over
        if not a or 'x57/click' in b or '<binding>' not in a.group(0) or re.search(r'flapslever|gear-down|property-adjust|<repeatable[^>]*>\s*true', b):
            return b
        kind = 'rocker' if 'property-toggle' in b and '<mod-up>' not in b and 'cb-' not in b and 'park-' not in b else 'push'
        n[kind] += 1
        bind = '<binding><command>property-toggle</command><property>/sim/model/x57/click/%s</property></binding>' % kind
        act = a.group(0).replace('<binding>', bind + '<binding>', 1)
        act = act.replace('<mod-up>', '<mod-up>' + bind, 1) if '<mod-up>' in act else act
        return b[:a.start()] + act + b[a.end():]

    for root, _, files in os.walk(base):
        for f in files:
            if f.endswith('.xml'):
                p = os.path.join(root, f)
                t = open(p, encoding='latin-1').read()
                u = re.sub(r'<animation>\s*<type>pick</type>.*?</animation>', one, t, flags=re.S)
                if u != t:
                    open(p, 'w', encoding='latin-1').write(u)
    print('  clicks on %(rocker)d switches and %(push)d buttons' % n)


def build(base, d):
    k = Kit(d)
    M.screw_picture(os.path.join(d, 'screw.png'))
    lens(d)
    toggle_model(d)
    faces = panel_faces(k, base)
    right_face(k, faces['r'])
    left_face(k, faces['l'])
    k.xml.append(M.effect('x57-face', ('face-r', 'face-l')))
    # the right panel
    tach = k.picture('digi', round_face(512))
    for n, x, y in (('rpm-l', 175, 125), ('rpm-r', 425, 125), ('avi-a', 660, 330), ('avi-b', 667, 560)):
        yy, zz = R(x, y)
        k.round_unit(n, yy, zz, 0.0300, tach, at=0.0245)
        k.screen(n, yy - 0.0212, zz - 0.0212, yy + 0.0212, zz + 0.0212, 0.0044)
    m = meter_face(k)
    for n, x, y in (('acc-al', 185, 338), ('acc-ar', 438, 335), ('acc-bl', 192, 573), ('acc-br', 445, 572)):
        yy, zz = R(x, y)
        k.objs.append(box(n + '-box', yy - 0.0375, zz - 0.0375, yy + 0.0375, zz + 0.0375, 0.0065, m, (0.5, 0.995)))
        k.dark.append(n + '-box')
        k.screen(n, yy - 0.0285, zz - 0.0150, yy + 0.0285, zz + 0.0300, 0.0069)
    k.toggle('tg-man-l', *R(135, 742), 'fdm/jsbsim/ctl/pitch-man-l', False, 0.0007)
    k.toggle('tg-man-r', *R(503, 742), 'fdm/jsbsim/ctl/pitch-man-r', False, 0.0007)
    k.spring('sp-l', *R(258, 742), 'fdm/jsbsim/ctl/pitch-sw-l', 0.0007)
    k.spring('sp-r', *R(378, 742), 'fdm/jsbsim/ctl/pitch-sw-r', 0.0007)
    M.gauge(d, 'oat', 'C', 'environment/temperature-degc', -40, 60, 20, 0.92, colour=(235, 80, 30))
    k.xml.append('<model><path>oat.xml</path><offsets><x-m>0.001</x-m><y-m>%.4f</y-m><z-m>%.4f</z-m></offsets></model>' % R(785, 760))
    k.heads += M.four(*R(785, 760), 0.0255, 0.0026)
    # the left panel
    annunciator(k)
    high_lift(k)
    k.round_unit('uhf', -0.146, -0.032, 0.0425, uhf_face(k), at=None)
    k.screen('uhf', -0.1678, -0.0310, -0.1263, -0.0103, 0.0044)
    k.round_unit('gmeter', -0.146, -0.128, 0.0425, gmeter_face(k), at=None)
    k.screen('gmeter', -0.1735, -0.1378, -0.1184, -0.1137, 0.0044)
    for n, u in (('mode', 280), ('reset', 512), ('action', 744)):       # on the face: its picture is 85 mm across, 1024 pixels
        k.push('gm-' + n, -0.146 + (u - 512) * 0.085 / 1024, -0.128 - (766 - 512) * 0.085 / 1024, 0.0030, X57 + 'gmeter/' + n, 0.0042, 0.002, red=True)
    k.toggle('tg-fan', *L(800, 752), 'sim/model/x57/cabin-fan', False, 0.0007)
    trim = Image.new('RGB', (256, 256), (16, 17, 19))
    g = ImageDraw.Draw(trim)
    g.polygon(((128, 24), (146, 96), (236, 120), (236, 140), (146, 134), (138, 206), (168, 226), (88, 226), (118, 206), (110, 134), (20, 140), (20, 120), (110, 96)), fill=WHITE)
    yy, zz = L(1030, 740)
    k.round_unit('rudder-trim', yy, zz, 0.0170, k.picture('rudder-trim', trim))
    k.heads += M.four(yy, zz, 0.0150, 0.0020)
    stack(k)
    crew_panel(k)
    k.objs.append(M.screws('screws', 'screw.png', M.SCREWS + k.heads))
    with open(os.path.join(d, 'flap-screws.ac'), 'w') as f:                       # the flap indicator lies back on the pedestal's slope
        f.write(M.ac([M.screws('screws', 'screw.png', M.four(0.0, 0.0, 0.0262, 0.0023))]))
    with open(os.path.join(d, 'flap-screws.xml'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<PropertyList><path>flap-screws.ac</path>%s</PropertyList>\n' % M.effect('x57-face', ('screws',)))
    k.xml.append('<model><path>flap-screws.xml</path><offsets><x-m>0.0165</x-m><y-m>%.3f</y-m><z-m>-0.283</z-m><pitch-deg>-21</pitch-deg></offsets></model>' % FLAPS_Y)
    k.xml.append(overhead(d))
    k.xml.append(lower(d))
    k.xml += side_panels(d)
    k.xml += park_brake(d)
    k.xml.append(M.effect('x57-face', ['screws'] + k.dark))
    k.xml.append(M.effect('x57-case', k.black))
    k.xml.append(M.effect('x57-red', k.red))
    with open(os.path.join(d, 'x57-panel.ac'), 'w') as f:
        f.write(M.ac(k.objs))
    with open(os.path.join(d, 'x57-panel.xml'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<!-- the X-57\'s own panels: written by fdm/x57panel.py -->\n<PropertyList>\n<path>x57-panel.ac</path>\n%s\n</PropertyList>\n' % '\n'.join(k.xml))
