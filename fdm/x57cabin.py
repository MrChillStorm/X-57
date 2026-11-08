#!/usr/bin/env python3
"""The X-57's cabin as NASA's photographs show it, in place of the P2006T's furnished one.

What NASA changed [20240007413, 20240010807, 20230005312]: the copilot's seat, yoke and pedals came out for a research
instrumentation pallet, which holds the data logger; the rear seats came out for the traction battery modules and their
control systems in the aft cabin. Photographs: ICD-CEPT-006 slide 11 and 20240014847 (the cockpit: the Tecnam's cream
mouldings low on the walls and round the doors, the front pillars white-painted with their rivets showing, green
zinc-chromate primer overhead, orange ducts low under the panel, a braided harness up the left pillar), 20205006955 (the aft
cabin in the ground vibration test: finned battery modules, orange vent hoses, the battery control modules in dark blue, a
green primed floor with its seat rails, the pilot's navy leather seat), 20250001718 (the flight batteries from above: black
finned packs with orange connectors forward, standing modules aft, braided harnesses up the walls) and 20230005312 and
20240009560 (the pallet's blue anodised frames, the logger bolted on them).

The batteries' design is Chin et al., Battery Performance Modeling on Maxwell X-57 (2019), table 1 and figs. 1 to 3: 16 modules
of 320 cells, 8 to a pack, A forward and B aft, the control modules between them; 20240009560: each pack 8 modules.

What isn't published, and so is this model's: every size and place in the aft cabin and on the pallet (the photographs and
drawings give their look and layout, not their dimensions), the cables' and hoses' routes, the frames' stations (0.30 m apart)
and the rivet pitch.

Frame: the Tecnam interior's (Models/Interior), x aft, y right, z up, metres. Floor -0.818, roof about 0.3 to 0.37, walls
about 0.53 to 0.59 out. The batteries' mass is in the flight model's empty weight, not here.

    install(out)    out: the package (../X-57)
"""
import math, os, re
import numpy as np
from PIL import Image, ImageDraw

import mkmodel as M
import x57panel as P

FLOOR = -0.818
SILL = -0.33                    # the side windows' lower edge: the Tecnam's cream mouldings below it
PILLAR_X = -2.55                # forward of this the walls are the white-painted front pillars and the windshield's frame
FRAMES = np.arange(-2.55, 0.0, 0.30)
PRIMER, CREAM, WHITE_PAINT, ALU = (171, 184, 138), (224, 218, 200), (233, 233, 227), (196, 199, 202)
RNG = np.random.default_rng(57)


# ------------------------------------------------------------------------------------------------- textures
def grain(h, w, colour, amount=3.0, streak=0.0):
    """a flat colour with fine noise (and streaks along x, for brushed or moulded surfaces)"""
    g = np.array(colour, float)[None, None, :] + RNG.normal(0, amount, (h, w, 1))
    if streak:
        g += RNG.normal(0, streak, (h, 1, 1))
    return g


def to_img(a):
    return Image.fromarray(np.clip(a, 0, 255).astype(np.uint8), 'RGB')


def rivets(g, pts, r, base):
    """domed rivet heads: a darker rim, a lit top"""
    dark = tuple(int(c * 0.72) for c in base)
    lit = tuple(min(255, int(c * 1.12)) for c in base)
    for x, y in pts:
        g.ellipse((x - r, y - r, x + r, y + r), fill=dark)
        g.ellipse((x - r * 0.55, y - r * 0.75, x + r * 0.35, y + r * 0.15), fill=lit)


class Sheet:
    """a texture laid over a world rectangle: a along u, b along v (v up)"""
    def __init__(s, a, b, size, colour, amount=3.0, streak=0.0):
        s.a, s.b = a, b
        s.W, s.H = size
        s.arr = grain(s.H, s.W, colour, amount, streak)
        s.ppm = s.W / (a[1] - a[0])

    def px(s, p, q):
        return (p - s.a[0]) / (s.a[1] - s.a[0]) * s.W, (s.b[1] - q) / (s.b[1] - s.b[0]) * s.H

    def fill(s, p0, p1, q0, q1, colour, amount=3.0, streak=0.0):
        x0, y1 = s.px(p0, q0)
        x1, y0 = s.px(p1, q1)
        x0, x1, y0, y1 = (int(max(0, min(v, lim))) for v, lim in ((x0, s.W), (x1, s.W), (y0, s.H), (y1, s.H)))
        if x1 > x0 and y1 > y0:
            s.arr[y0:y1, x0:x1] = grain(y1 - y0, x1 - x0, colour, amount, streak)

    def uv(s, p, q):
        return (p - s.a[0]) / (s.a[1] - s.a[0]), (q - s.b[0]) / (s.b[1] - s.b[0])

    def save(s, path, draw=None):
        im = to_img(s.arr)
        if draw:
            draw(ImageDraw.Draw(im))
        im.save(path)


def wall_texture(path):
    """the side walls, seen from inside, over x and z: cream mouldings below the sill; above it the front pillars white-painted,
    the rest green primer, its frames riveted"""
    s = Sheet((-3.36, 0.06), (-0.86, 0.42), (2048, 1024), PRIMER, 4.0)
    s.fill(-3.36, 0.06, -0.86, SILL, CREAM, 2.0, 1.5)
    s.fill(-3.36, PILLAR_X, SILL, 0.42, WHITE_PAINT, 2.0)

    def draw(g):
        r = 0.0021 * s.ppm
        for x in FRAMES:                                                   # the frames: a shaded flange, two rows of rivets
            if x <= PILLAR_X + 0.01:
                continue
            a, b = s.px(x - 0.012, 0.42)
            c, d = s.px(x + 0.012, SILL)
            g.rectangle((a, b, c, d), fill=tuple(int(v * 0.86) for v in PRIMER))
            rivets(g, [s.px(x + dx, z) for dx in (-0.007, 0.007) for z in np.arange(SILL + 0.02, 0.40, 0.025)], r, PRIMER)
        for z in (0.06, 0.24):                                             # stringers along
            a, b = s.px(PILLAR_X, z + 0.008)
            c, d = s.px(0.06, z - 0.008)
            g.rectangle((a, b, c, d), fill=tuple(int(v * 0.9) for v in PRIMER))
            rivets(g, [s.px(x, z) for x in np.arange(PILLAR_X + 0.01, 0.0, 0.03)], r, PRIMER)
        for x in np.arange(-3.30, PILLAR_X, 0.035):                         # the pillars' rivet lines, as in slide 11
            rivets(g, [s.px(x, z) for z in (SILL + 0.05, 0.30)], r, WHITE_PAINT)
        for x in np.arange(-3.0, 0.0, 0.62):                                # the mouldings' joints and their screws
            a, b = s.px(x, SILL)
            c, d = s.px(x + 0.004, -0.86)
            g.rectangle((a, b, c, d), fill=(170, 165, 150))
            rivets(g, [s.px(x + dx, z) for dx in (-0.02, 0.024) for z in (-0.40, -0.72)], 0.003 * s.ppm, CREAM)
        a, b = s.px(-3.36, SILL + 0.004)
        c, d = s.px(0.06, SILL - 0.004)
        g.rectangle((a, b, c, d), fill=(40, 40, 42))                       # the window seal's black edge
    s.save(path, draw)
    return s


def roof_texture(path):
    """the roof from below, over x and y: green primer, frames across, stringers along"""
    s = Sheet((-3.36, 0.06), (-0.64, 0.64), (2048, 1024), PRIMER, 4.0)

    def draw(g):
        r = 0.0021 * s.ppm
        for x in FRAMES:
            a, b = s.px(x - 0.014, 0.64)
            c, d = s.px(x + 0.014, -0.64)
            g.rectangle((a, b, c, d), fill=tuple(int(v * 0.84) for v in PRIMER))
            rivets(g, [s.px(x + dx, y) for dx in (-0.008, 0.008) for y in np.arange(-0.60, 0.61, 0.025)], r, PRIMER)
        for y in (-0.36, -0.12, 0.12, 0.36):
            a, b = s.px(-3.36, y + 0.007)
            c, d = s.px(0.06, y - 0.007)
            g.rectangle((a, b, c, d), fill=tuple(int(v * 0.9) for v in PRIMER))
            rivets(g, [s.px(x, y) for x in np.arange(-3.3, 0.0, 0.03)], r, PRIMER)
    s.save(path, draw)
    return s


def floor_texture(path):
    """the floor from above, over x and y: green primer, the floor beams' rivets across, access panels, and the seat rails
    the pilot's seat and the pallet stand on"""
    s = Sheet((-3.36, 0.06), (-0.64, 0.64), (2048, 1024), (160, 176, 128), 5.0)

    def draw(g):
        r = 0.0024 * s.ppm
        for x in FRAMES:
            rivets(g, [s.px(x, y) for y in np.arange(-0.56, 0.57, 0.03)], r, (160, 176, 128))
        for x0, x1, y0, y1 in ((-1.62, -1.38, -0.40, -0.12), (-1.62, -1.38, 0.12, 0.40), (-0.70, -0.46, -0.14, 0.14)):
            a, b = s.px(x0, y1)
            c, d = s.px(x1, y0)
            g.rectangle((a, b, c, d), outline=(118, 130, 96), width=4)
            rivets(g, [s.px(x, y) for x in np.linspace(x0 + 0.015, x1 - 0.015, 6) for y in (y0 + 0.012, y1 - 0.012)], 0.003 * s.ppm, (190, 190, 186))
        for yc in (-0.29, 0.29):                                            # seat rails: aluminium, their holes every 25 mm
            for y in (yc - 0.17, yc + 0.17):
                a, b = s.px(-2.50, y + 0.014)
                c, d = s.px(-1.66, y - 0.014)
                g.rectangle((a, b, c, d), fill=(186, 189, 192), outline=(120, 122, 126), width=2)
                for x in np.arange(-2.48, -1.68, 0.025):
                    u, v = s.px(x, y)
                    g.ellipse((u - 4, v - 4, u + 4, v + 4), fill=(70, 72, 76))
    s.save(path, draw)
    return s


def bulkhead_texture(path):
    """the aft bulkhead and the firewall's back, over y and z: bare aluminium sheets, riveted at their joints"""
    s = Sheet((-0.64, 0.64), (-0.86, 0.42), (1024, 1024), ALU, 3.0, 2.0)

    def draw(g):
        r = 0.0021 * s.ppm
        for y in (-0.32, 0.0, 0.32):
            rivets(g, [s.px(y, z) for z in np.arange(-0.84, 0.41, 0.025)], r, ALU)
            a, b = s.px(y - 0.002, 0.42)
            c, d = s.px(y + 0.002, -0.86)
            g.rectangle((a, b, c, d), fill=(150, 152, 156))
        for z in (-0.40, 0.0):
            rivets(g, [s.px(y, z) for y in np.arange(-0.62, 0.63, 0.025)], r, ALU)
    s.save(path, draw)
    return s


def plain_texture(path, colour, amount=3.0, size=64):
    to_img(grain(size, size, colour, amount)).save(path)


# ------------------------------------------------------------------------------------------------- the shell
def ac_blocks(t):
    head, *blocks = re.split(r'(?=^OBJECT poly)', t, flags=re.M)
    return head, blocks


def parse_block(b):
    lines = b.rstrip('\n').split('\n')
    i = next(j for j, q in enumerate(lines) if q.startswith('numvert'))
    n = int(lines[i].split()[1])
    loc = re.search(r'^loc (.*)', b, re.M)
    V = np.array([[float(q) for q in l.split()[:3]] for l in lines[i + 1:i + 1 + n]])
    V = V + (np.array([float(q) for q in loc.group(1).split()]) if loc else 0.0)
    world = np.c_[V[:, 0], -V[:, 2], V[:, 1]]
    j = i + 2 + n
    surfs = []
    while j < len(lines) and lines[j].startswith('SURF'):
        k = int(lines[j + 2].split()[1])
        refs = [int(q.split()[0]) for q in lines[j + 3:j + 3 + k]]
        surfs.append((lines[j], lines[j + 1], refs))
        j += 3 + k
    return lines[:i + 1 + n], world, surfs


def write_block(header, name, tex, world, surfs, uv):
    h = []
    for q in header:
        if q.startswith('name '):
            q = 'name "%s"' % name
        elif q.startswith('texture '):
            q = 'texture "%s"' % tex
        h.append(q)
    out = h + ['numsurf %d' % len(surfs)]
    for flag, mat, refs in surfs:
        out += [flag, mat, 'refs %d' % len(refs)] + ['%d %.5f %.5f' % ((r,) + uv(world[r], refs, world)) for r in refs]
    return '\n'.join(out + ['kids 0']) + '\n'


def normal(world, refs):
    T = world[refs]
    n = np.zeros(3)
    for k in range(1, len(refs) - 1):
        n += np.cross(T[k] - T[0], T[k + 1] - T[0])
    return n / (np.linalg.norm(n) + 1e-12)


def shell(d):
    """the cabin's shell retextured: its faces sorted into walls, roof and bulkheads, each laid flat on its own picture"""
    wall, roof, floor, bulk = (wall_texture(os.path.join(d, 'cabin-wall.png')), roof_texture(os.path.join(d, 'cabin-roof.png')),
                               floor_texture(os.path.join(d, 'cabin-floor.png')), bulkhead_texture(os.path.join(d, 'cabin-bulk.png')))
    plain_texture(os.path.join(d, 'cabin-trim.png'), CREAM, 2.0)
    plain_texture(os.path.join(d, 'cabin-frame.png'), PRIMER, 4.0)
    xz = lambda s: (lambda p, refs, w: s.uv(p[0], p[2]))
    xy = lambda s: (lambda p, refs, w: s.uv(p[0], p[1]))
    yz = lambda s: (lambda p, refs, w: s.uv(p[1], p[2]))
    flat = lambda p, refs, w: (0.5, 0.5)
    p = os.path.join(d, 'interior.ac')
    head, blocks = ac_blocks(open(p, errors='replace').read())
    out, added = [], 0
    for b in blocks:
        name = re.search(r'^name "([^"]+)"', b, re.M).group(1)
        if name == 'interieur':
            header, world, surfs = parse_block(b)
            groups = {'interieur': [], 'int-roof': [], 'int-bulk': []}
            for sf in surfs:
                n = normal(world, sf[2])
                groups['int-roof' if abs(n[2]) > 0.6 else 'int-bulk' if abs(n[0]) > max(abs(n[1]), abs(n[2])) else 'interieur'].append(sf)
            out.append(write_block(header, 'interieur', 'cabin-wall.png', world, groups['interieur'], xz(wall)))
            out.append(write_block(header, 'int-roof', 'cabin-roof.png', world, groups['int-roof'], xy(roof)))
            out.append(write_block(header, 'int-bulk', 'cabin-bulk.png', world, groups['int-bulk'], yz(bulk)))
            added += 2
        elif name == 'planchet':
            header, world, surfs = parse_block(b)
            out.append(write_block(header, name, 'cabin-floor.png', world, surfs, xy(floor)))
        elif name in ('intporteD', 'intporteG'):
            header, world, surfs = parse_block(b)
            out.append(write_block(header, name, 'cabin-trim.png', world, surfs, flat))
        elif name in ('inttour', 'inttourporteD', 'inttourporteG'):
            header, world, surfs = parse_block(b)
            out.append(write_block(header, name, 'cabin-frame.png', world, surfs, flat))
        else:
            out.append(b if b.endswith('\n') else b + '\n')
    k = int(re.search(r'^OBJECT world\nkids (\d+)', head, re.M).group(1))
    head = re.sub(r'^(OBJECT world\nkids )\d+', r'\g<1>%d' % (k + added), head, count=1, flags=re.M)
    open(p, 'w', errors='replace').write(head + ''.join(out))
    x = os.path.join(d, 'interior.xml')
    t = open(x, errors='replace').read()
    t = t.replace('<object-name>interieur</object-name>', '<object-name>interieur</object-name>\n    <object-name>int-roof</object-name>\n    <object-name>int-bulk</object-name>')
    t = t.replace('</PropertyList>', M.effect('x57-face', ('interieur', 'int-roof', 'int-bulk', 'planchet', 'intporteD', 'intporteG', 'inttour', 'inttourporteD', 'inttourporteG'))
                  + '\n<model><path>cabin.xml</path></model>\n</PropertyList>')
    open(x, 'w', errors='replace').write(t)


# ------------------------------------------------------------------------------------------------- seats and figures
# The P2006T model's seat is high for its cabin: its cushion 30 cm over the floor and 83 cm under the roof's lining, where a median man
# sits 91 cm tall (86 cm leaning back 20 deg), and FlightGear's cockpit eye point 64.5 cm over it, where his eyes are about 75 cm over a
# seat. The cabin itself is within 5 cm of the 46 in (1.17 m) published [Plane & Pilot; All-Aero]. So the seat comes down 8 cm: its
# cushion and back, its base squashed toward its foot on the floor. It stays where it was fore and aft: slid back, his arm fell short of
# the yoke pushed full forward; his knees clear the yoke instead by the pedals' place (x57pedals.py)
SEAT_DROP = 0.08


def lower_seat(path, drop=SEAT_DROP):
    """the seat's cushion and back (and its lever) down by drop, its base's top with them, its foot where it was"""
    parts = re.split(r'(?=^OBJECT )', open(path, errors='replace').read(), flags=re.M)
    done = set()
    for w, o in enumerate(parts):
        n = re.search(r'^name "([^"]*)"', o, re.M)
        m = re.search(r'^numvert (\d+)', o, re.M)
        if not (n and m):
            continue
        l = o.split('\n')
        i = next(j for j, q in enumerate(l) if q.startswith('numvert'))
        c = int(l[i].split()[1])
        V = np.array([q.split()[:3] for q in l[i + 1:i + 1 + c]], float)       # ac: y is up
        if n.group(1) == 'Base':
            lo, hi = V[:, 1].min(), V[:, 1].max()
            V[:, 1] = lo + (V[:, 1] - lo) * (hi - lo - drop) / (hi - lo)
        else:
            V[:, 1] -= drop
        l[i + 1:i + 1 + c] = ['%.6f %.6f %.6f' % tuple(v) for v in V]
        parts[w] = '\n'.join(l)
        done.add(n.group(1))
    if not {'Base', 'Seat'} <= done:
        raise SystemExit('x57cabin: the seat model has no Base and Seat: %s' % sorted(done))
    open(path, 'w', errors='replace').write(''.join(parts))


def seats(d):
    """only the pilot's seat stays, in navy leather on its cream base [20205006955 fig. 7]; the copilot's figure goes with its seat"""
    p = os.path.join(d, 'Seats', 'all-seats.xml')
    t = open(p, errors='replace').read()
    models = re.findall(r'<model>.*?</model>', t, flags=re.S)
    keep = [m for m in models if re.search(r'<x-m>\s*-2\.08\s*</x-m>', m) and re.search(r'<y-m>\s*-0\.29\s*</y-m>', m)]
    if len(keep) != 1:
        raise SystemExit('x57cabin: the pilot seat not found once in all-seats.xml')
    open(p, 'w', errors='replace').write(re.sub(r'<model>.*</model>', keep[0], t, count=1, flags=re.S))
    lower_seat(os.path.join(d, 'Seats', 'seat.ac'))
    s = os.path.join(d, 'Seats', 'Seat.png')
    a = np.asarray(Image.open(s).convert('RGB'), float)
    W = a.shape[1]
    lum = a.mean(2, keepdims=True) / 255.0
    sat = a.max(2) - a.min(2)
    cushion = (np.arange(W)[None, :] < 0.53 * W) & (lum[..., 0] > 0.45) & (sat < 40)
    navy = np.array((30, 38, 64), float) * (0.55 + 0.75 * lum)
    a = np.where(cushion[..., None], navy, a)
    base = (np.arange(W)[None, :] > 0.585 * W)
    a = np.where(base[..., None], np.array(CREAM, float) * (0.9 + 0.1 * lum), a)
    to_img(a).save(s)
    p = os.path.join(d, '..', 'Pilot', 'all-pilots.xml')
    t = open(p, errors='replace').read()
    t = re.sub(r'<model>(?:(?!</model>).)*?copilot\.xml.*?</model>\s*', '', t, flags=re.S)
    t = re.sub(r'<animation>(?:(?!</animation>).)*?<object-name>TheCoPilot</object-name>.*?</animation>\s*', '', t, flags=re.S)     # and its select
    open(p, 'w', errors='replace').write(t)


# ------------------------------------------------------------------------------------------------- the equipment
def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def cuboid(name, c, size, tex, tile=0.1):
    """a box on the frame's axes: centre c, size (x, y, z); each face's picture in metres / tile"""
    c, h = np.asarray(c, float), np.asarray(size, float) / 2
    v = [tuple(c + h * np.array((sx, sy, sz))) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    faces = []
    for ax in range(3):
        for sgn in (-1, 1):
            idx = [i for i in range(8) if ((i >> (2 - ax)) & 1) == (sgn > 0)]
            o = [a for a in range(3) if a != ax]
            pts = np.array([v[i] for i in idx])
            cen = pts.mean(0)
            ang = np.arctan2(pts[:, o[1]] - cen[o[1]], pts[:, o[0]] - cen[o[0]])
            idx = [idx[i] for i in np.argsort(ang)]
            n = np.cross(np.array(v[idx[1]]) - v[idx[0]], np.array(v[idx[2]]) - v[idx[0]])
            if n[ax] * sgn < 0:
                idx = idx[::-1]
            faces.append(P.surf(idx, [(v[i][o[0]] / tile, v[i][o[1]] / tile) for i in idx], flag='0x00'))
    return P.obj(name, v, faces, tex)


def bezier(pts, n=24):
    p = [np.asarray(q, float) for q in pts]
    out = []
    for t in np.linspace(0, 1, n):
        q = p[:]
        while len(q) > 1:
            q = [(1 - t) * a + t * b for a, b in zip(q, q[1:])]
        out.append(q[0])
    return np.array(out)


def route(corners, r, n=6):
    """a path through corners, straight between them, each corner rounded over r either side"""
    c = [np.asarray(q, float) for q in corners]
    out = [c[0]]
    for a, b, d in zip(c, c[1:], c[2:]):
        u, w = unit(a - b), unit(d - b)
        p, q = b + u * min(r, np.linalg.norm(a - b) / 2), b + w * min(r, np.linalg.norm(d - b) / 2)
        out += [(1 - t) ** 2 * p + 2 * (1 - t) * t * b + t * t * q for t in np.linspace(0, 1, n)]
    out.append(c[-1])
    keep = [out[0]]
    for q in out[1:]:
        if np.linalg.norm(q - keep[-1]) > 1e-4:
            keep.append(q)
    return np.array(keep)


def tube(name, path, r, tex, n=10, around=1.0, along=0.05, cap=False):
    """a tube along path: n sides, its picture once around and once per `along` metres; cap closes its far end"""
    path = np.asarray(path, float)
    t = np.gradient(path, axis=0)
    t /= np.linalg.norm(t, axis=1)[:, None]
    ref = np.array((0, 0, 1.0)) if abs(t[0][2]) < 0.9 else np.array((1.0, 0, 0))
    u = unit(np.cross(t[0], ref))
    v, s, length = [], [], 0.0
    for i, (p, tt) in enumerate(zip(path, t)):
        u = unit(u - (u @ tt) * tt)
        w = np.cross(tt, u)
        if i:
            length += np.linalg.norm(p - path[i - 1])
        v += [(tuple(p + r * (math.cos(2 * math.pi * k / n) * u + math.sin(2 * math.pi * k / n) * w)), k / n * around, length / along) for k in range(n + 1)]
    m = n + 1
    for i in range(len(path) - 1):
        for k in range(n):
            q = (i * m + k, i * m + k + 1, (i + 1) * m + k + 1, (i + 1) * m + k)
            s.append(P.surf(q, [(v[j][1], v[j][2]) for j in q], flag='0x10'))
    if cap:
        last = (len(path) - 1) * m
        s.append(P.surf([last + k for k in range(n)], [(0.5 + 0.5 * math.cos(2 * math.pi * k / n), 0.5 + 0.5 * math.sin(2 * math.pi * k / n)) for k in range(n)], flag='0x00'))
    return P.obj(name, [q[0] for q in v], s, tex)


def disc_y(name, c, r, axis, tex, n=16, h=0.012):
    """a short cylinder (a connector) from c along axis"""
    c = np.asarray(c, float)
    return tube(name, [c, c + h * unit(axis)], r, tex, n, 1.0, 0.02, cap=True)


_ROOF = {}


def roof_hit(d, x, y):
    """the cabin roof above (x, y): its height and its normal into the cabin, from the interior's own mesh"""
    if 'T' not in _ROOF:
        tris = []
        for o in re.split(r'(?=^OBJECT )', open(os.path.join(d, 'interior.ac'), errors='replace').read(), flags=re.M):
            n, c = re.search(r'^name "([^"]+)"', o, re.M), re.search(r'^numvert (\d+)', o, re.M)
            if not (n and c) or n.group(1) not in ('interieur', 'int-roof'):
                continue
            header, world, surfs = parse_block(o)
            for _, _, refs in surfs:
                for k in range(1, len(refs) - 1):
                    tris.append(world[[refs[0], refs[k], refs[k + 1]]])
        _ROOF['T'] = np.array(tris)
    T, o, dvec = _ROOF['T'], np.array((x, y, 0.0)), np.array((0, 0, 1.0))
    best = None
    for tri in T:
        e1, e2 = tri[1] - tri[0], tri[2] - tri[0]
        h = np.cross(dvec, e2)
        det = e1 @ h
        if abs(det) < 1e-12:
            continue
        s_ = o - tri[0]
        u = (s_ @ h) / det
        q = np.cross(s_, e1)
        v = (dvec @ q) / det
        t = (e2 @ q) / det
        if u >= 0 and v >= 0 and u + v <= 1 and t > 0 and (best is None or t < best[0]):
            best = (t, unit(np.cross(e1, e2)))
    nrm = best[1] if best[1][2] > 0 else -best[1]                          # pointing up, out of the cabin through the roof
    return best[0], nrm


def equipment(d):
    objs, xml, look = [], [], {}

    def add(o, eff):
        objs.append(o)
        look.setdefault(eff, []).append(re.search(r'name "([^"]+)"', o).group(1))
    tex = {}
    for n, c, a in (('kit-blue', (34, 74, 176), 4), ('kit-black', (22, 22, 24), 3), ('kit-navy', (28, 38, 74), 3), ('kit-silver', (188, 191, 195), 4),
                    ('kit-white', (226, 226, 222), 3), ('kit-orange', (214, 92, 40), 4), ('kit-fabric', (182, 190, 172), 6), ('kit-green', (60, 150, 70), 6)):
        plain_texture(os.path.join(d, n + '.png'), c, a)
        tex[n] = n + '.png'
    plate = grain(256, 256, (34, 74, 176), 4)                              # the pallet's frames: anodised blue, cut with pockets
    im = to_img(plate)
    g = ImageDraw.Draw(im)
    for i in range(2):
        for j in range(2):
            g.rounded_rectangle((20 + 120 * i, 20 + 120 * j, 116 + 120 * i, 116 + 120 * j), 14, fill=(16, 34, 84))
    im.save(os.path.join(d, 'kit-pocket.png'))
    tex['kit-pocket'] = 'kit-pocket.png'
    fins = np.zeros((128, 128, 3))
    for i in range(128):
        fins[:, i] = 205 if (i // 8) % 2 == 0 else 120
    fins += RNG.normal(0, 4, fins.shape)
    to_img(fins).save(os.path.join(d, 'kit-fins.png'))
    to_img(fins * 0.22).save(os.path.join(d, 'kit-blackfins.png'))
    hose = np.zeros((64, 64, 3)) + np.array((206, 84, 36))
    for j in range(64):
        hose[j, :] *= 0.72 + 0.28 * abs(math.sin(math.pi * j / 16))
    to_img(hose).save(os.path.join(d, 'kit-hose.png'))
    braid = np.zeros((64, 64, 3))
    yy, xx = np.mgrid[0:64, 0:64]
    braid += 18 + 28 * (((xx + yy) // 8 + (xx - yy) // 8) % 2)[..., None]
    to_img(braid).save(os.path.join(d, 'kit-braid.png'))
    tex.update({n: n + '.png' for n in ('kit-fins', 'kit-blackfins', 'kit-hose', 'kit-braid')})

    # the instrumentation pallet in the copilot's place [20240007413; 20230005312 fig. 2]: blue anodised frames on the seat rails,
    # the data logger, the data units and their power on two shelves, cables down to the floor and forward
    x0, x1, y0, y1 = -2.30, -1.86, 0.11, 0.49
    zt = FLOOR + 0.48
    xc, yc = (x0 + x1) / 2, (y0 + y1) / 2
    for i, (x, y) in enumerate(((x0, y0), (x0, y1), (x1, y0), (x1, y1))):
        add(cuboid('pal-post-%d' % i, (x, y, (FLOOR + zt) / 2), (0.025, 0.025, zt - FLOOR), tex['kit-blue']), 'x57-face')
    for i, y in enumerate((y0, y1)):
        add(cuboid('pal-rail-%d' % i, (xc, y, FLOOR + 0.012), (x1 - x0 + 0.06, 0.03, 0.024), tex['kit-blue']), 'x57-face')
    for i, z in enumerate((FLOOR + 0.20, zt)):
        add(cuboid('pal-shelf-%d' % i, (xc, yc, z), (x1 - x0 + 0.025, y1 - y0 + 0.025, 0.008), tex['kit-pocket'], 0.2), 'x57-face')
    add(cuboid('pal-side', (xc, y1 + 0.004, (FLOOR + zt) / 2), (x1 - x0, 0.006, zt - FLOOR - 0.03), tex['kit-pocket'], 0.22), 'x57-face')
    for i, (a, b) in enumerate((((x0, FLOOR + 0.03), (x1, zt - 0.01)), ((x0, zt - 0.01), (x1, FLOOR + 0.03)))):   # the outboard side's X brace
        add(tube('pal-brace-%d' % i, [(a[0], y1 + 0.012, a[1]), (b[0], y1 + 0.012, b[1])], 0.007, tex['kit-blue'], 4), 'x57-face')
    add(cuboid('pal-logger', (xc - 0.06, yc - 0.02, zt + 0.032), (0.20, 0.15, 0.056), tex['kit-black']), 'x57-case')
    add(cuboid('pal-switch-1', (xc + 0.13, yc - 0.09, zt + 0.024), (0.12, 0.08, 0.04), tex['kit-black']), 'x57-case')
    add(cuboid('pal-switch-2', (xc + 0.13, yc + 0.06, zt + 0.024), (0.12, 0.08, 0.04), tex['kit-black']), 'x57-case')
    add(cuboid('pal-mdau', (xc, yc, FLOOR + 0.20 + 0.06), (0.26, 0.20, 0.11), tex['kit-silver']), 'x57-steel')
    add(cuboid('pal-psu', (xc + 0.02, yc + 0.02, FLOOR + 0.08), (0.24, 0.16, 0.10), tex['kit-blackfins']), 'x57-face')
    # the cables leave through the floor, each by a rubber grommet: the data unit's from its back face behind the pallet, the
    # logger's ahead of it, every one turning straight down into its hole (their routing below the floor isn't published)
    def grommet(name, g):
        add(tube(name, [(g[0], g[1], FLOOR - 0.002), (g[0], g[1], FLOOR + 0.007)], 0.032, tex['kit-black'], 16, 1.0, 0.02), 'x57-case')
        add(tube(name + '-hole', [(g[0], g[1], FLOOR), (g[0], g[1], FLOOR + 0.0015)], 0.029, tex['kit-black'], 16, 1.0, 0.02, cap=True), 'x57-case')

    def down(start, g, k, n):
        """a cable from start, level out of the frame's open side to above the grommet g, then down into it: the k-th of n,
        spread in the hole. Straight runs with rounded bends, so it clears the shelves and posts"""
        h = (g[0], g[1] + 0.015 * (k - (n - 1) / 2))                      # side by side across the hole, in their order: no crossing
        return route([start, (h[0], start[1], start[2]), (h[0], h[1], start[2] - 0.08), (h[0], h[1], FLOOR - 0.08)], 0.025)
    ga, gf = (x1 + 0.06, yc - 0.03), (x0 - 0.07, yc - 0.03)                # 4.7 and 4.5 cm clear of the shelves' edges
    grommet('pal-grommet-a', ga)
    grommet('pal-grommet-f', gf)
    for i in range(4):
        y = yc - 0.075 + 0.05 * i
        add(disc_y('pal-conn-%d' % i, (xc + 0.13, y, FLOOR + 0.27), 0.011, (1, 0, 0), tex['kit-silver']), 'x57-steel')
        add(tube('pal-cable-%d' % i, down((xc + 0.145, y, FLOOR + 0.27), ga, i, 4),
                 0.006, tex['kit-orange' if i % 2 else 'kit-white'], 6), 'x57-face')
    for i in range(3):
        y = yc - 0.06 + 0.04 * i
        add(tube('pal-loop-%d' % i, down((xc - 0.16, y, zt + 0.03), gf, i, 3), 0.005, tex['kit-black' if i else 'kit-orange'], 6), 'x57-face')

    # The traction batteries [Chin 2019 table 1 and figs. 1-3; 20240009560]: two packs, A and B, of 8 modules each, every module
    # 320 cells (16 in series of 20 in parallel) in a solid aluminium block with fins, a central blow-off vent on top and a
    # terminal either side of it, standing on feet bolted to the floor. Pack A where the rear seats were, pack B further aft, the
    # two battery control modules between them on the right, as the ground vibration test's photograph has them [20205006955
    # fig. 4]. Each pack is one series string, 8 x 16 = 128 cells: the cables snake across its two rows of four, every terminal
    # taking one cable, and its two ends go to its own control module, whose dual outputs rise through the roof to the wing (the
    # P2006T is high-winged). The vents: each module's stub rises into a header over its row, the headers run down the left
    # side into a plenum on the floor between the packs, which drains overboard through the floor. What's this model's: the
    # cables' and hoses' routes, the plenum, every size (a module 0.17 x 0.20 x 0.42 m, from 320 cells in two layers)
    F = FLOOR
    up_ = lambda a, h: np.add(a, (0, 0, h))
    base = lambda x: float(np.interp(x, (-0.6, -0.5, -0.3, -0.1), (-0.818, -0.812, -0.797, -0.786)))

    def jack(name, at, axis, r=0.016, h=0.016, kind='kit-orange', eff='x57-face'):
        add(disc_y(name, at, r, axis, tex[kind], 12, h), eff)
        return np.add(at, h * unit(axis))
    plain_texture(os.path.join(d, 'kit-gold.png'), (196, 170, 84), 4)
    tex['kit-gold'] = 'kit-gold.png'
    cols = (-0.33, -0.11, 0.11, 0.33)
    MX, MY, MZ = 0.17, 0.20, 0.42
    term, vent = {}, {}
    for pk, rows in (('a', (-1.42, -1.18)), ('b', (-0.50, -0.26))):
        for r, xr in enumerate(rows):
            b0 = base(xr)
            for c, yc in enumerate(cols):
                n = 'bat-%s%d%d' % (pk, r, c)
                add(cuboid(n, (xr, yc, b0 + MZ / 2), (MX, MY, MZ), tex['kit-fins'], 0.08), 'x57-steel')
                for k, dy in enumerate((-1, 1)):
                    add(cuboid('%s-foot%d' % (n, k), (xr, yc + dy * (MY / 2 + 0.004), b0 + 0.03), (0.22, 0.006, 0.06), tex['kit-gold']), 'x57-face')
                    term[pk, r, c, k] = jack('%s-t%d' % (n, k), (xr - 0.045, yc + dy * 0.065, b0 + MZ), (0, 0, 1), 0.013, 0.016)
                vent[pk, r, c] = jack('%s-v' % n, (xr + 0.03, yc, b0 + MZ), (0, 0, 1), 0.022, 0.025, 'kit-silver', 'x57-steel')

    def link(name, a, b, h):
        """a short cable between two terminals, rising h above the higher"""
        z = max(a[2], b[2]) + h
        add(tube(name, route([a, (a[0], a[1], z), (b[0], b[1], z), b], 0.008), 0.009, tex['kit-orange'], 8), 'x57-face')
    # each pack: (0,3) (0,2) (0,1) (0,0) (1,0) (1,1) (1,2) (1,3); across a row a module's +y terminal to its neighbour's -y one,
    # between the rows the -y terminals of column 0, low under the vent headers; the ends: column 3's +y terminals
    for pk in ('a', 'b'):
        for c in (3, 2, 1):
            link('bat-%s-l0%d' % (pk, c), term[pk, 0, c, 0], term[pk, 0, c - 1, 1], 0.03)
        link('bat-%s-lx' % pk, term[pk, 0, 0, 0], term[pk, 1, 0, 0], 0.012)
        for c in (0, 1, 2):
            link('bat-%s-l1%d' % (pk, c), term[pk, 1, c, 1], term[pk, 1, c + 1, 0], 0.03)
    bcm = {'a': (-0.80, 0.36), 'b': (-0.80, 0.20)}                        # 0.16 x 0.13 x 0.40; A outboard, B inboard
    for pk, (xb, yb) in bcm.items():
        add(cuboid('bcm-%s' % pk, (xb, yb, F + 0.20), (0.16, 0.13, 0.40), tex['kit-navy']), 'x57-face')
    # pack A's ends: outboard of the rows (y 0.47), aft to the control module's forward face
    ja1 = jack('bcm-a-in1', (-0.88, 0.39, F + 0.25), (-1, 0, 0))
    ja2 = jack('bcm-a-in2', (-0.88, 0.33, F + 0.15), (-1, 0, 0))
    a = term['a', 0, 3, 1]
    add(tube('bat-a-end1', route([a, up_(a, 0.06), (a[0], 0.47, a[2] + 0.06), (-0.99, 0.47, a[2] + 0.06), (-0.99, 0.39, a[2] + 0.06), (-0.99, 0.39, ja1[2]), ja1], 0.025),
             0.011, tex['kit-orange'], 8), 'x57-face')
    a = term['a', 1, 3, 1]
    add(tube('bat-a-end2', route([a, up_(a, 0.02), (a[0], 0.47, a[2] + 0.02), (-0.95, 0.47, a[2] + 0.02), (-0.95, 0.33, a[2] + 0.02), (-0.95, 0.33, ja2[2]), ja2], 0.025),
             0.011, tex['kit-orange'], 8), 'x57-face')
    # pack B's ends: outboard, forward, then across behind the control modules into B's aft face
    jb1 = jack('bcm-b-in1', (-0.72, 0.17, F + 0.25), (1, 0, 0))
    jb2 = jack('bcm-b-in2', (-0.72, 0.23, F + 0.15), (1, 0, 0))
    a = term['b', 1, 3, 1]
    add(tube('bat-b-end1', route([a, up_(a, 0.06), (a[0], 0.47, a[2] + 0.06), (-0.62, 0.47, a[2] + 0.06), (-0.62, 0.17, a[2] + 0.06), (-0.62, 0.17, jb1[2]), jb1], 0.025),
             0.011, tex['kit-orange'], 8), 'x57-face')
    a = term['b', 0, 3, 1]
    add(tube('bat-b-end2', route([a, up_(a, 0.02), (a[0], 0.47, a[2] + 0.02), (-0.66, 0.47, a[2] + 0.02), (-0.66, 0.23, a[2] + 0.02), (-0.66, 0.23, jb2[2]), jb2], 0.025),
             0.011, tex['kit-orange'], 8), 'x57-face')
    # the control modules' dual outputs, up through the roof to the wing, each by a collar square to the roof
    for pk, (xb, yb) in bcm.items():
        for k, dy in enumerate((-0.035, 0.035)):
            o = jack('bcm-%s-out%d' % (pk, k), (xb + 0.045, yb + dy, F + 0.40), (0, 0, 1), 0.016, 0.02)
            hit, nrm = roof_hit(d, o[0], o[1])
            add(tube('bcm-%s-hv%d' % (pk, k), route([o, (o[0], o[1], hit - 0.06), tuple(np.add((o[0], o[1], hit), 0.06 * nrm))], 0.02), 0.011, tex['kit-orange'], 8), 'x57-face')
            add(tube('bcm-%s-hv%d-collar' % (pk, k), [tuple(np.add((o[0], o[1], hit), -0.006 * nrm)), tuple(np.add((o[0], o[1], hit), 0.004 * nrm))], 0.022, tex['kit-black'], 16, 1.0, 0.02), 'x57-case')
    # vents: each module's stub into its row's header; the headers down the left side to the plenum, which drains through the floor
    pl = np.array((-0.74, -0.40, F + 0.08))
    add(cuboid('bat-plenum', pl, (0.16, 0.20, 0.16), tex['kit-fabric']), 'x57-face')
    for pk, rows in (('a', (-1.42, -1.18)), ('b', (-0.50, -0.26))):
        for r, xr in enumerate(rows):
            hz = base(xr) + MZ + 0.08
            for c in range(4):
                v = vent[pk, r, c]
                add(tube('bat-%s%d%d-stub' % (pk, r, c), [v, (v[0], v[1], hz)], 0.015, tex['kit-hose'], 10, 1.0, 0.03), 'x57-face')
            xh = xr + 0.03
            near = (pk == 'a') == (r == 1)                                     # the row nearer the plenum
            if near:                                                            # down the left side, along the floor into the plenum's face
                into = pl[0] - 0.06 if pk == 'a' else pl[0] + 0.06
                path = [(into, -0.475, F + 0.05), (xh, -0.475, F + 0.05), (xh, -0.475, hz), (xh, 0.43, hz)]
            else:                                                               # over the nearer row's corner, down into the plenum's top
                xd = pl[0] - 0.04 if pk == 'a' else pl[0] + 0.04
                path = [(xd, -0.44, pl[2] + 0.04), (xd, -0.44, hz + 0.06), (xd, -0.475, hz + 0.06), (xh, -0.475, hz + 0.06), (xh, -0.475, hz), (xh, 0.43, hz)]
            add(tube('bat-%s%d-header' % (pk, r), route(path, 0.04), 0.022, tex['kit-hose'], 12, 1.0, 0.03, cap=True), 'x57-face')
    add(tube('bat-drain', route([pl + (0, 0.06, -0.02), (pl[0], -0.25, F + 0.06), (pl[0], -0.25, F - 0.08)], 0.03), 0.028, tex['kit-hose'], 12, 1.0, 0.03), 'x57-face')
    add(tube('bat-drain-collar', [(pl[0], -0.25, F - 0.002), (pl[0], -0.25, F + 0.012)], 0.036, tex['kit-black'], 16, 1.0, 0.02), 'x57-case')

    # braided harnesses along the floor at both walls [slide 11, 20250001718]. The right one comes up through the floor aft of
    # the pallet, runs aft by the wall, rises in front of the control modules and plugs into A's forward face; both modules'
    # white wires come forward and down into it as a loom. The left one comes from behind the panel, runs aft by the wall and
    # rises up the left wall into the roof by a collar
    add(tube('harness-l', route([(-3.00, -0.30, F + 0.25), (-2.95, -0.30, F + 0.03), (-2.70, -0.535, F + 0.03), (-0.75, -0.535, F + 0.03)], 0.08), 0.017, tex['kit-braid'], 10, 1.0, 0.04), 'x57-face')   # aft of the left duct
    plug = jack('bcm-a-harness', (-0.88, 0.405, F + 0.33), (-1, 0, 0), 0.018, 0.020, 'kit-silver', 'x57-steel')
    hr = route([(-1.78, 0.515, F - 0.06), (-1.78, 0.515, F + 0.025), (-1.40, 0.525, F + 0.025), (-0.92, 0.525, F + 0.025), (-0.92, 0.525, F + 0.33),
                (-0.92, 0.405, F + 0.33), plug], 0.035)
    add(tube('harness-r', hr, 0.015, tex['kit-braid'], 10, 1.0, 0.04), 'x57-face')
    add(tube('harness-r-collar', [(-1.78, 0.515, F - 0.002), (-1.78, 0.515, F + 0.010)], 0.024, tex['kit-black'], 16, 1.0, 0.02), 'x57-case')
    zl = F + 0.52                                                         # a laced loom over both control modules' connectors
    add(tube('bcm-loom', route([(-0.845, 0.13, zl), (-0.845, 0.46, zl), (-0.918, 0.46, zl), (-0.918, 0.46, F + 0.335)], 0.03)[::-1], 0.012, tex['kit-white'], 10, 1.0, 0.04, cap=True), 'x57-face')
    for pk, (xb, yb) in bcm.items():
        for k, dy in enumerate((-0.045, -0.015, 0.015, 0.045)):
            t = jack('bcm-%s-c%d' % (pk, k), (xb - 0.045, yb + dy, F + 0.40), (0, 0, 1), 0.012, 0.03, 'kit-silver', 'x57-steel')
            add(tube('bcm-%s-w%d' % (pk, k), [t, (t[0], t[1], zl)], 0.005, tex['kit-white'], 6), 'x57-face')   # up into the loom
    roof_at = np.array((-0.70, -0.36, 0.285))                               # the roof here, sloping 26 deg: its normal into the cabin
    ax_ = unit((0, -0.44, 0.90))
    up = [(-0.75, -0.535, F + 0.03), (-0.71, -0.535, -0.62), (-0.70, -0.545, -0.45), (-0.70, -0.537, -0.20), (-0.70, -0.505, 0.05),
          (-0.70, -0.455, 0.18), tuple(roof_at - 0.07 * ax_), tuple(roof_at + 0.08 * ax_)]
    add(tube('harness-up', route(up, 0.03), 0.014, tex['kit-braid'], 10, 1.0, 0.04), 'x57-face')
    add(tube('harness-up-collar', [tuple(roof_at - 0.006 * ax_), tuple(roof_at + 0.004 * ax_)], 0.026, tex['kit-black'], 16, 1.0, 0.02), 'x57-case')

    # the orange ducts low at the panel's ends [20240014847 cockpit photograph]
    for s_ in (-1, 1):                                                    # each down through the floor by a collar
        g = (-2.86, 0.48 * s_)
        add(tube('duct-%d' % (s_ > 0), bezier([(-3.02, 0.36 * s_, -0.42), (-2.92, 0.50 * s_, -0.50), (g[0], g[1], FLOOR + 0.22), (g[0], g[1], FLOOR + 0.05), (g[0], g[1], FLOOR - 0.10)], 24),
                 0.032, tex['kit-hose'], 12, 1.0, 0.03), 'x57-face')
        add(tube('duct-%d-collar' % (s_ > 0), [(g[0], g[1], FLOOR - 0.002), (g[0], g[1], FLOOR + 0.012)], 0.040, tex['kit-black'], 16, 1.0, 0.02), 'x57-case')

    with open(os.path.join(d, 'cabin.ac'), 'w') as f:
        f.write(M.ac(objs))
    xml += [M.effect(e, ns) for e, ns in look.items()]
    with open(os.path.join(d, 'cabin.xml'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<!-- the X-57\'s cabin equipment, written by fdm/x57cabin.py (see there) -->\n<PropertyList>\n<path>cabin.ac</path>\n%s\n</PropertyList>\n' % '\n'.join(xml))
    return len(objs)


def install(out):
    d = os.path.join(out, 'Models', 'Interior')
    shell(d)
    seats(d)
    n = equipment(d)
    print('  cabin: the pilot seat only, the pallet, the batteries and %d parts' % n)
