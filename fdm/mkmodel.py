"""The 3-D model. The fuselage, tail, gear and cockpit are the Tecnam P2006T's from FGAddon (GPL v2): the X-57 keeps them, and NASA's own mesh is empty inside and has no windows. The wing,
nacelles and propellers are lofted from NASA's geometry, in pieces along the span, so that each piece can ride on the wing's first bending mode.

    fuselage(out)      writes Models/fuselage.ac, fuselage.xml, the texture and the interior
    wing_parts(...)    the meshes
    wing_anim(...)     their animations
"""
import math, os, re, shutil
import numpy as np
import x57data as X
import x57flex as FX

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'refs', 'fgaddon-p2006t', 'Tecnam-P2006T')
X0, Z0 = 4.93, 1.96                # the Tecnam's model to the structural frame, metres: lines up the nose, cabin and tailplane with NASA's fuselage
KEEP = ('fuselage', 'vitres', 'vitreD', 'vitreG', 'HDRvitres', 'HDRvitreD', 'HDRvitreG', 'porteD', 'porteG', 'porteA', 'porteAD', 'porteAG', 'portetrainD', 'portetrainG',
        'axeAB', 'axeAH', 'axeDB', 'axeDH', 'axeGB', 'axeGH', 'roueA', 'roueD', 'roueG', 'ailes2', 'direction')
SEG = (24.14, 46.32, 69.0, 91.68, 114.36, 136.61, 159.0, 181.97, 189.74)     # the wing's pieces, inches, breaking at the flap, the aileron and between the high-lift nacelles
Y_FLAP, Y_AIL, HINGE = (24.14, 136.61), (136.61, 181.97), 0.75
PROPS = {'sim/multiplay/generic/float[0]': 'sim/model/door-left', 'sim/multiplay/generic/float[1]': 'sim/model/door-right', 'sim/multiplay/generic/float[11]': 'gear/gear[0]/rollspeed-ms',
         'sim/multiplay/generic/float[12]': 'gear/gear[1]/rollspeed-ms', 'sim/multiplay/generic/float[13]': 'gear/gear[2]/rollspeed-ms', 'sim/multiplay/generic/float[14]': 'gear/gear[0]/steering-norm',
         'sim/multiplay/generic/float[15]': 'gear/gear[0]/compression-norm', 'sim/multiplay/generic/float[16]': 'gear/gear[1]/compression-norm', 'sim/multiplay/generic/float[17]': 'gear/gear[2]/compression-norm',
         'sim/multiplay/generic/bool[2]': 'sim/rendering/hdr/hdr-enabled', 'surface-positions/rudder-pos-norm': 'fdm/jsbsim/x57/rudder-norm', 'surface-positions/elevator-pos-norm': 'fdm/jsbsim/x57/stab-norm'}


# NASA's fuselage crown over the wing root, read off X-57.glb: inches, from the stations and half spans below. It stands about 2 in over the wing
ROOF_X = (126, 132, 138, 144, 150, 156, 162, 168, 174, 180, 186, 192, 198, 204)
ROOF_Y = (0, 4, 8, 12, 16, 20)
ROOF_Z = ((91.5, 91.5, 91.5, 91.4, 90.6, 88.5), (91.9, 91.9, 91.9, 91.6, 90.7, 88.5), (92.1, 92.1, 92.0, 91.8, 91.0, 88.4), (92.2, 92.2, 92.1, 91.8, 91.0, 88.8), (92.2, 92.2, 92.1, 91.6, 90.8, 89.8),
          (92.1, 92.0, 91.9, 91.4, 90.5, 90.0), (91.9, 91.9, 91.7, 91.2, 90.2, 90.0), (91.5, 91.5, 91.3, 90.7, 89.7, 89.2), (91.0, 90.9, 90.7, 90.1, 89.1, 87.3), (89.9, 89.8, 89.7, 89.1, 88.1, 85.0),
          (89.1, 89.0, 88.8, 88.3, 86.9, 83.5), (87.6, 87.6, 87.4, 86.8, 85.2, 81.8), (86.3, 86.3, 86.0, 85.3, 83.7, 79.3), (85.3, 85.2, 84.9, 84.1, 82.4, 77.0))


def roof(o):
    """the cabin's top over the wing, in the fuselage's own skin. The Tecnam's wing was the roof there, 1.5 m of it, and the X-57's is half as deep. One smooth surface in polar sections about the cabin's
    middle is fitted to the skin around the opening and NASA's crown across it. The skin near the opening is eased onto that surface, so its rough edge goes, and the opening is filled on it with the
    edge's own vertices, so the shading runs across without a seam. The wing comes out through it. Returns the object"""
    from scipy.spatial import Delaunay
    from scipy.sparse import lil_matrix
    from scipy.sparse.linalg import spsolve
    import collections
    l = o.split('\n')
    i, c = next((i, int(q.split()[1])) for i, q in enumerate(l) if q.startswith('numvert'))
    d = np.array((re.findall(r'^loc (.*)', o, re.M) or ['0 0 0'])[0].split(), float)
    v = np.array([q.split()[:3] for q in l[i + 1:i + 1 + c]], float) + d
    P = np.c_[v[:, 0] + X0, -v[:, 2], v[:, 1] + Z0]
    tris, edges, uv = [], collections.Counter(), None
    k = i + 1 + c
    head = None
    faces = []
    while k < len(l):
        if l[k].startswith('SURF') and head is None:
            head = l[k:k + 2]
        if l[k].startswith('refs'):
            n = int(l[k].split()[1])
            r = [q.split() for q in l[k + 1:k + 1 + n]]
            ids = [int(q[0]) for q in r]
            faces.append((k, r))
            tris += [(ids[0], ids[a], ids[a + 1]) for a in range(1, n - 1)]
            for a, b in zip(ids, ids[1:] + ids[:1]):
                edges[(min(a, b), max(a, b))] += 1
            if uv is None and P[ids, 2].min() > 2.3 and 3.0 < P[ids, 0].mean() < 3.5:
                uv = (float(r[0][1]), float(r[0][2]))
            k += n
        k += 1
    ZC = 1.9                                                                       # the sections' middle
    T = P[np.array(tris)]

    def polar(y, z):
        return np.arctan2(y, z - ZC), np.hypot(y, z - ZC)                           # angle from straight up, right positive
    # the big flat faces around the opening go too: where they met the new skin their facets showed as lines
    th0 = polar(P[:, 1], P[:, 2])[0]
    gone = set()
    for k, r in faces:
        ids = np.array([int(q[0]) for q in r])
        if ((P[ids, 0] > 3.08) & (P[ids, 0] < 5.52) & (np.abs(th0[ids]) < 1.12) & (P[ids, 2] > ZC)).all():
            gone.add(k)
    edges = collections.Counter()
    for k, r in faces:
        if k not in gone:
            ids = [int(q[0]) for q in r]
            for a, b in zip(ids, ids[1:] + ids[:1]):
                edges[(min(a, b), max(a, b))] += 1
    nxt = collections.defaultdict(list)
    for (a, b), n in edges.items():
        if n == 1:
            nxt[a].append(b)
            nxt[b].append(a)
    start = next(a for a in nxt if 3.6 < P[a, 0] < 4.9 and P[a, 2] > 2.1 and abs(P[a, 1]) > 0.3)
    loop, seen = [start], {start}
    while True:
        q = [b for b in nxt[loop[-1]] if b not in seen]
        if not q or len(loop) > 900:
            break
        loop.append(q[0])
        seen.add(q[0])
    data = []
    for x in np.r_[np.arange(3.0, 3.54, 0.06), np.arange(3.6, 5.05, 0.1), np.arange(5.06, 5.62, 0.06)]:
        cut = T[(T[:, :, 0].min(1) <= x) & (T[:, :, 0].max(1) > x)]
        for a, b in ((0, 1), (1, 2), (2, 0)):
            pa, pb = cut[:, a], cut[:, b]
            m = (pa[:, 0] - x) * (pb[:, 0] - x) < 0
            q = pa[m] + ((x - pa[m, 0]) / (pb[m, 0] - pa[m, 0]))[:, None] * (pb[m] - pa[m])
            th, rr = polar(q[:, 1], q[:, 2])
            th = np.abs(th)
            use = (th < 1.5) & (q[:, 2] > ZC) & ((x < 3.54) | (x > 5.05) | (th > 1.25))    # over the opening only the sides below it
            data += [(x, t_, r_) for t_, r_ in zip(th[use], rr[use])]
    for a, x in enumerate(ROOF_X):                                                 # NASA's crown across it
        if 3.6 < x * 0.0254 < 5.0:
            for b, y in enumerate(ROOF_Y[:5]):
                th, rr = polar(y * 0.0254, ROOF_Z[a][b] * 0.0254)
                data += [(x * 0.0254, th, rr)] * 8
    D = np.array(data)
    terms = lambda x, t: np.stack([((x - 4.3) / 1.3) ** a * (t / 1.5) ** (2 * b) for a in range(7) for b in range(5)], axis=-1)
    fit = np.linalg.lstsq(terms(D[:, 0], D[:, 1]), D[:, 2], rcond=None)[0]
    print('  roof: surface fitted to %.0f mm RMS' % (1000 * np.sqrt(np.mean((terms(D[:, 0], D[:, 1]) @ fit - D[:, 2]) ** 2))))
    step = lambda u: (lambda q: q * q * (3 - 2 * q))(np.clip(u, 0, 1))
    th, rr = polar(P[:, 1], P[:, 2])
    w = step((P[:, 0] - 3.0) / 0.5) * step((5.6 - P[:, 0]) / 0.5) * step((1.5 - np.abs(th)) / 0.22) * (P[:, 2] > ZC)
    r2 = rr + w * (terms(P[:, 0], np.abs(th)) @ fit - rr)
    Q = np.c_[P[:, 0], r2 * np.sin(th), ZC + r2 * np.cos(th)]
    # long edges of the opening are cut into short ones on the surface, in the old faces too, or the chord leaves a slit
    extra, split = [], {}
    off = r2 - terms(P[:, 0], np.abs(th)) @ fit                                     # how far the edge lies off the surface
    for a, b in zip(loop, loop[1:] + loop[:1]):
        n = int(np.ceil(np.linalg.norm(Q[b] - Q[a]) / 0.04))
        ts = np.arange(1, n) / max(n, 1)
        split[(a, b)] = [c + len(extra) + q for q in range(len(ts))]
        extra += [(P[a, 0] + t_ * (P[b, 0] - P[a, 0]), th[a] + t_ * (th[b] - th[a]), off[a] + t_ * (off[b] - off[a])) for t_ in ts]
    extra = np.array(extra).reshape(-1, 3)
    # the opening, filled in the (x, angle) plane
    ring = []
    for a, b in zip(loop, loop[1:] + loop[:1]):
        ring.append(a)
        ring += split[(a, b)]
    xt = np.r_[np.c_[P[:, 0], th, off], extra]
    B = xt[ring, :2]
    edge_off = xt[ring, 2]

    def inside(p):
        x, y = p[:, 0], p[:, 1]
        r = np.zeros(len(p), bool)
        for a, b in zip(B, np.roll(B, -1, axis=0)):
            r ^= ((a[1] > y) != (b[1] > y)) & (x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1] + 1e-12) + a[0])
        return r
    g = np.mgrid[B[:, 0].min():B[:, 0].max():0.016, B[:, 1].min():B[:, 1].max():0.036].reshape(2, -1).T
    g = g[inside(g)]
    g = g[np.min(np.hypot(g[:, None, 0] - B[None, :, 0], 0.45 * (g[:, None, 1] - B[None, :, 1])), axis=1) > 0.032]
    pts = np.r_[B, g]
    # in metres, inside a far frame: on the hull a straight run of edge loses its middle points, and they show as pinholes
    sc = np.array((1.0, 0.45))
    tri = Delaunay(np.r_[pts * sc, np.array(((2.0, -5.0), (7.0, -5.0), (7.0, 5.0), (2.0, 5.0)))]).simplices
    tri = tri[(tri < len(pts)).all(1)]
    tri = tri[inside(pts[tri].mean(1))]
    # the edge's small difference from the surface dies away inwards
    nb, N = len(B), len(pts)
    near = collections.defaultdict(set)
    for t in tri:
        for a in t:
            near[a] |= set(t) - {a}
    M_ = lil_matrix((N, N))
    rhs = np.zeros(N)
    for a in range(N):
        M_[a, a] = 1.0 if a < nb else 1.03
        if a < nb:
            rhs[a] = edge_off[a]
        else:
            for b in near[a]:
                M_[a, b] = -1.0 / len(near[a])
    dev = spsolve(M_.tocsr(), rhs)
    on = np.r_[extra[:, :2], g]
    rg = terms(on[:, 0], np.abs(on[:, 1])) @ fit + np.r_[extra[:, 2], dev[nb:]]
    new = np.c_[on[:, 0], rg * np.sin(on[:, 1]), ZC + rg * np.cos(on[:, 1])]
    ids = np.r_[np.array(ring), c + len(extra) + np.arange(len(g))]
    A = np.r_[Q, new]
    # the fuselage's picture is a side view of each half: the new skin takes the same mapping, so its soft shading runs on across the joins
    fitted = {}
    for sg in (-1, 1):
        rows = np.array([(P[int(q[0]), 0], P[int(q[0]), 2], float(q[1]), float(q[2])) for k, r in faces for q in r
                         if min(sg * P[int(w_[0]), 1] for w_ in r) > 0.02 and P[int(q[0]), 2] > 2.05 and (2.45 < P[int(q[0]), 0] < 3.05 or 5.45 < P[int(q[0]), 0] < 6.3)])
        fitted[sg] = np.linalg.lstsq(np.c_[rows[:, :2], np.ones(len(rows))], rows[:, 2:], rcond=None)[0]
    chart = lambda sg, p: np.array((p[0], p[2], 1.0)) @ fitted[sg]
    # the faces left along the edge take its new vertices, as fans about their middles: a long thin polygon on a curve won't triangulate cleanly
    patch, centres, added = {k: None for k in gone}, [], 0
    for k, r in faces:
        ids_ = [int(q[0]) for q in r]
        if k in gone or not any((a, b) in split or (b, a) in split for a, b in zip(ids_, ids_[1:] + ids_[:1])):
            continue
        rows = []
        for q, (a, b) in enumerate(zip(ids_, ids_[1:] + ids_[:1])):
            rows.append((a, float(r[q][1]), float(r[q][2])))
            mid = split.get((a, b)) or split.get((b, a), [])[::-1]
            ua, ub = np.array(r[q][1:3], float), np.array(r[(q + 1) % len(r)][1:3], float)
            for m_, w_ in enumerate(mid):
                u_ = ua + (ub - ua) * (m_ + 1) / (len(mid) + 1)
                rows.append((w_, u_[0], u_[1]))
        cid = len(A) + len(centres)
        centres.append(A[[q[0] for q in rows]].mean(0))
        cu = np.mean([q[1:] for q in rows], axis=0)
        block = []
        for p_, q_ in zip(rows, rows[1:] + rows[:1]):
            block += l[k - 2:k] + ['refs 3', '%d %.5f %.5f' % (cid, cu[0], cu[1]), '%d %.5f %.5f' % p_, '%d %.5f %.5f' % q_]
        patch[k] = block
        added += len(rows) - 1
    A = np.r_[A, np.array(centres).reshape(-1, 3)]
    out = []
    for t in ids[tri]:
        a, b, e = t
        nrm = np.cross(A[b] - A[a], A[e] - A[a])
        if nrm @ np.array((0.0, A[a, 1], A[a, 2] - ZC)) < 0:
            b, e = e, b
        sg = 1 if A[[a, b, e], 1].mean() > 0 else -1
        out += head + ['refs 3'] + ['%d %.5f %.5f' % ((q,) + tuple(chart(sg, A[q]))) for q in (a, b, e)]
    print('  roof: %d old faces out, %d new in, edge %d vertices, %.0f mm off the surface at most' % (len(gone), len(tri), len(loop), 1000 * np.abs(edge_off).max()))
    for k in sorted(patch, reverse=True):
        n = int(l[k].split()[1])
        l[k - 2:k + 1 + n] = patch[k] or []
    ns = next(q for q, s_ in enumerate(l) if s_.startswith('numsurf'))
    kids = max(q for q, s_ in enumerate(l) if s_.startswith('kids'))
    vert = ['%.6f %.6f %.6f' % (a - X0 - d[0], e - Z0 - d[1], -b - d[2]) for a, b, e in A]
    l = l[:i] + ['numvert %d' % len(A)] + vert + ['numsurf %d' % (int(l[ns].split()[1]) + len(tri) - len(gone) + added)] + l[ns + 1:kids] + out + l[kids:]
    return '\n'.join(l)

# the doors' mouse-over hint: open or shut, and whether a click works now (the pick's own condition: all wheels down, below 10 m/s). Only '>' in the script, so the XML needs no escapes
# (the hint watches the door's own property: the tooltip listens to it, and sim/model/door-left is an alias, which takes no listener)
DOORS = {'doors.crew.toggle();': ('main', 'Main door', 'sim/model/door-positions/crew/position-norm'),
         'doors.passenger.toggle();': ('rear', 'Rear door', 'sim/model/door-positions/passenger/position-norm')}
DOOR_TIP = ("var v = arg[0]; var w = getprop('/gear/gear[0]/wow') and getprop('/gear/gear[1]/wow') and getprop('/gear/gear[2]/wow') and 10 > getprop('/gear/gear[0]/rollspeed-ms'); "
            "if (!w) return v > 0.01 ? 'open (shuts on the ground)' : 'shut (opens on the ground)'; return v > 0.99 ? 'open, click to shut' : 0.01 > v ? 'shut, click to open' : 'moving';")


# each door's parts, its HDR glass too: the Tecnam left the HDR glass out, so it stayed shut when the door opened. The glass selects split a door's named group into one group per
# parent; rotate moves them all, but a pick takes only the first it finds, which was the non-HDR glass, hidden under HDR: the door had nothing to click. So the pick names the parts
MAIN_DOOR_DEG = 110             # the main door's full swing: the P2006T flight manual's "fully opened pilot's door (up to 110°)"; the Tecnam model had 70
DOOR_PARTS = {'PorteGauche': ('porteG', 'vitreG', 'HDRvitreG'), 'PorteDroite': ('porteD', 'vitreD', 'HDRvitreD'),
              'IntPorteGauche': ('intporteG', 'inttourporteG', 'intvitreG', 'HDRintvitreG'), 'IntPorteDroite': ('intporteD', 'inttourporteD', 'intvitreD', 'HDRintvitreD')}


def door_hover(a):
    """one animation, fixed if it is a door's: its group with all its parts, its pick on each part, with the mouse-over hint"""
    objs = lambda g: ''.join('<object-name>%s</object-name>' % n for n in DOOR_PARTS[g])
    m = re.search(r'<name>\s*(\w+)\s*</name>', a)
    if m and m.group(1) in DOOR_PARTS and '<type>' not in a:
        a = re.sub(r'\s*<object-name>.*?</object-name>', '', a, flags=re.S).replace('</name>', '</name>' + objs(m.group(1)), 1)
    if re.search(r'<type>\s*rotate\s*</type>', a) and re.search(r'<object-name>\s*(Int)?PorteGauche\s*</object-name>', a):
        a, n = re.subn(r'<factor>\s*-70\s*</factor>', '<factor>%d</factor>' % -MAIN_DOOR_DEG, a)
        assert n == 1, a
    if re.search(r'<type>\s*pick\s*</type>', a):
        for g in DOOR_PARTS:
            a = re.sub(r'<object-name>\s*%s\s*</object-name>' % g, objs(g), a)
    for script, (k, label, prop) in DOORS.items():
        if '<script>%s</script>' % script in a and '<hovered>' not in a:
            a = a.replace('</action>', '</action><hovered><binding><command>set-tooltip</command><tooltip-id>x57-door-%s</tooltip-id><label>%s: %%s</label><property>%s</property>'
                          '<mapping>nasal</mapping><script>%s</script></binding></hovered>' % (k, label, prop, DOOR_TIP), 1)
    return a


def fuselage(out):
    m = os.path.join(out, 'Models')
    t = open(os.path.join(SRC, 'Models', 'p2006t.ac'), errors='replace').read()
    head, *objs = re.split(r'(?=^OBJECT poly)', t, flags=re.M)
    keep = [o for o in objs if re.search(r'^name "([^"]+)"', o, re.M).group(1) in KEEP]
    head = re.sub(r'kids \d+', 'kids %d' % len(keep), head)
    # the Tecnam's wheels stop short of the flight model's contact points, once its model is lined up with NASA's fuselage. So the legs lengthen as they come down
    contact = (X.GEAR['ground_z_in'] - X.GEAR['sag_in']) * 0.0254
    obj = lambda n: next(o for o in keep if re.search(r'^name "%s"' % n, o, re.M))

    def heights(n):
        l = obj(n).split('\n')
        i, c = next((i, int(q.split()[1])) for i, q in enumerate(l) if q.startswith('numvert'))
        return np.array([float(q.split()[1]) for q in l[i + 1:i + 1 + c]]) + Z0 + float((re.search(r'^loc \S+ (\S+)', obj(n), re.M) or [0, 0])[1])
    drop = {'Avant': heights('roueA').min() - contact, 'Gauche': heights('roueG').min() - contact, 'Droit': heights('roueD').min() - contact}
    keep = [roof(o) if re.search(r'^name "fuselage"', o, re.M) else o for o in keep]
    with open(os.path.join(m, 'fuselage.ac'), 'w', encoding='latin-1', errors='replace') as f:
        f.write(head + ''.join(keep))
    for n in re.findall(r'^texture "([^"]+)"', ''.join(keep), re.M):
        shutil.copy(os.path.join(SRC, 'Models', n), m)
    import x57paint                     # the Tecnam's paint off, the X-57's on
    x57paint.paint(os.path.join(m, 'fuselage.ac'), os.path.join(m, 'texture.png'))
    x = open(os.path.join(SRC, 'Models', 'p2006t.xml'), errors='replace').read()
    x = re.sub(r'<!--.*?-->', '', x, flags=re.S)
    an = []
    for a in re.findall(r'<animation>.*?</animation>', x, re.S):
        names = re.findall(r'<object-name>(.*?)</object-name>', a)
        strip = [n for n in names if n in KEEP or n[:1].isupper() and n in ('EnsembleAvant', 'EnsembleGauche', 'EnsembleDroit', 'TrainAvant', 'TrainGauche', 'TrainDroit', 'RoueAvant', 'PorteGauche', 'PorteDroite')]
        if 'Helice' in a or 'turret' in a or not strip:
            continue
        for n in set(names) - set(strip):
            a = a.replace('<object-name>%s</object-name>' % n, '')
        for k, v in PROPS.items():
            a = a.replace(k, v)
        a = door_hover(a)
        if re.search(r'<type>rotate</type>\s*<object-name>Ensemble', a):        # the struts' travel, redone below
            continue
        an.append(a)
    # each wheel on the ground at any compression: down by the leg's extension, up by what the flight model compresses, in feet. The strut stretches from its pivot to reach it
    for n, (s_, d_, leg, c_) in enumerate((('Avant', drop['Avant'], 'axeAH', (-3.44014, 0.0, -0.85695)), ('Gauche', drop['Gauche'], 'axeGH', (-0.79439, -0.82529, -0.68454)), ('Droit', drop['Droit'], 'axeDH', (-0.79439, 0.82529, -0.68454)))):
        up = '<axis><x>0</x><y>0</y><z>1</z></axis>'
        an.append('<animation><type>translate</type><object-name>Ensemble%s</object-name><property>gear/gear[%d]/position-norm</property><interpolation><entry><ind>0</ind><dep>0</dep></entry><entry><ind>1</ind><dep>%.4f</dep></entry></interpolation>%s</animation>' % (s_, n, -d_, up))
        an.append('<animation><type>translate</type><object-name>Ensemble%s</object-name><property>gear/gear[%d]/compression-norm</property><factor>0.3048</factor>%s</animation>' % (s_, n, up))
        low = heights(leg).min() - Z0
        an.append('<animation><type>scale</type><object-name>%s</object-name><property>gear/gear[%d]/position-norm</property><x-factor>0</x-factor><x-offset>1</x-offset><y-factor>0</y-factor><y-offset>1</y-offset><z-factor>%.4f</z-factor><z-offset>1</z-offset>'
                  '<center><x-m>%g</x-m><y-m>%g</y-m><z-m>%g</z-m></center></animation>' % (leg, n, d_ / (c_[2] - low), *c_))
    with open(os.path.join(m, 'fuselage.xml'), 'w') as f:
        skin = [k for k in KEEP if 'vitre' not in k]
        f.write('<?xml version="1.0"?>\n<!-- The P2006T from FGAddon, GPL v2, less its wing, engines and fittings: written by fdm/mkmodel.py -->\n<PropertyList>\n<path>fuselage.ac</path>\n%s\n%s\n%s\n'
                '<model><path>Interior/interior.xml</path></model>\n</PropertyList>\n' % ('\n'.join(an), effect('x57-skin', skin), effect('Glass/glass', ('HDRvitres', 'HDRvitreD', 'HDRvitreG'))))
    effects(out)
    dst = os.path.join(m, 'Interior')
    shutil.rmtree(dst, ignore_errors=True)
    shutil.copytree(os.path.join(SRC, 'Models', 'Interior'), dst)
    for r, _, fs in os.walk(dst):
        for n in fs:
            if n.endswith('.xml'):
                p = os.path.join(r, n)
                s = open(p, errors='replace').read().replace('Aircraft/Tecnam-P2006T', 'Aircraft/X-57').replace('<inherits-from>../Effects/Glass/glass</inherits-from>', '<inherits-from>Aircraft/X-57/Models/Effects/Glass/glass</inherits-from>')
                s = s.replace('<inherits-from>../Effects/Glass/glassrain</inherits-from>', '<inherits-from>Aircraft/X-57/Models/Effects/Glass/glassrain</inherits-from>')
                s = re.sub(r'<inherits-from>(?:\.\./){2,}Effects/Glass/glass</inherits-from>', '<inherits-from>Aircraft/X-57/Models/Effects/Glass/instrument-glass</inherits-from>', s)
                for k, v in PROPS.items():      # the door linings and the HDR glass read multiplayer properties, which this aircraft doesn't set: the doors opened without their linings
                    s = s.replace(k, v)
                s = re.sub(r'<animation>.*?</animation>', lambda q: door_hover(q.group(0)), s, flags=re.S)
                open(p, 'w', errors='replace').write(s)
    shutil.rmtree(os.path.join(m, 'Pilot'), ignore_errors=True)
    shutil.copytree(os.path.join(SRC, 'Models', 'Pilot'), os.path.join(m, 'Pilot'))
    for r, _, fs in os.walk(os.path.join(m, 'Pilot')):
        for n in fs:
            if n.endswith('.xml'):
                p = os.path.join(r, n)
                s = open(p, errors='replace').read().replace('Aircraft/Tecnam-P2006T', 'Aircraft/X-57')
                open(p, 'w', errors='replace').write(s)
    shutil.copy(os.path.join(SRC, 'COPYING'), os.path.join(m, 'COPYING-P2006T'))


EFF = {'x57-skin': ('Effects/model-pbr', '<metallic-factor type="float">0.0</metallic-factor><roughness-factor type="float">0.35</roughness-factor>'),
       'x57-paint': ('Effects/model-pbr', '<base-color-factor type="vec4d">0.92 0.93 0.95 1.0</base-color-factor><metallic-factor type="float">0.0</metallic-factor><roughness-factor type="float">0.35</roughness-factor>'),
       'x57-blade': ('Effects/model-pbr', '<base-color-factor type="vec4d">0.045 0.045 0.05 1.0</base-color-factor><metallic-factor type="float">0.0</metallic-factor><roughness-factor type="float">0.5</roughness-factor>'),
       'x57-cblade': ('Effects/model-pbr', '<base-color-factor type="vec4d">0.74 0.74 0.76 1.0</base-color-factor><metallic-factor type="float">0.0</metallic-factor><roughness-factor type="float">0.38</roughness-factor>'),
       'x57-green': ('Effects/model-pbr', '<base-color-factor type="vec4d">0.04 0.42 0.12 1.0</base-color-factor><metallic-factor type="float">0.0</metallic-factor><roughness-factor type="float">0.2</roughness-factor>'),
       'x57-lens': ('Effects/model-pbr', '<base-color-factor type="vec4d">0.8 0.82 0.85 1.0</base-color-factor><metallic-factor type="float">0.2</metallic-factor><roughness-factor type="float">0.15</roughness-factor>'),
       'x57-spinner': ('Effects/model-pbr', '<base-color-factor type="vec4d">0.018 0.045 0.20 1.0</base-color-factor><metallic-factor type="float">0.3</metallic-factor><roughness-factor type="float">0.22</roughness-factor>'),
       'x57-steel': ('Effects/model-pbr', '<base-color-factor type="vec4d">0.72 0.73 0.75 1</base-color-factor><metallic-factor type="float">0.4</metallic-factor><roughness-factor type="float">0.35</roughness-factor>'),
       'x57-red': ('Effects/model-pbr', '<base-color-factor type="vec4d">0.55 0.06 0.04 1</base-color-factor><metallic-factor type="float">0.0</metallic-factor><roughness-factor type="float">0.4</roughness-factor>'),
       'x57-face': ('Effects/model-pbr', '<metallic-factor type="float">0.0</metallic-factor><roughness-factor type="float">0.7</roughness-factor>'),
       'x57-needle': ('Effects/model-pbr', '<base-color-factor type="vec4d">0.95 0.95 0.92 1.0</base-color-factor><metallic-factor type="float">0.0</metallic-factor><roughness-factor type="float">0.5</roughness-factor>'),
       'x57-polished': ('Effects/model-pbr', '<base-color-factor type="vec4d">0.80 0.80 0.82 1.0</base-color-factor><metallic-factor type="float">1.0</metallic-factor><roughness-factor type="float">0.18</roughness-factor>'),
       'x57-case': ('Effects/model-pbr', '<base-color-factor type="vec4d">0.015 0.015 0.018 1.0</base-color-factor><metallic-factor type="float">0.0</metallic-factor><roughness-factor type="float">0.45</roughness-factor>'),
       'x57-plate': ('Effects/model-pbr', '<base-color-factor type="vec4d">0.42 0.45 0.49 1.0</base-color-factor><metallic-factor type="float">0.0</metallic-factor><roughness-factor type="float">0.7</roughness-factor>'),
       'x57-webbing': ('Effects/model-pbr', '<base-color-factor type="vec4d">0.035 0.035 0.038 1.0</base-color-factor><metallic-factor type="float">0.0</metallic-factor><roughness-factor type="float">0.85</roughness-factor>'),
       'x57-gold': ('Effects/model-pbr', '<base-color-factor type="vec4d">1.0 0.78 0.36 1.0</base-color-factor><metallic-factor type="float">1.0</metallic-factor><roughness-factor type="float">0.22</roughness-factor>'),
       'x57-shades': ('Effects/model-pbr-transparent', '<base-color-factor type="vec4d">0.045 0.085 0.045 0.8</base-color-factor><metallic-factor type="float">0.0</metallic-factor><roughness-factor type="float">0.04</roughness-factor>'),
       'x57-clear': ('Effects/model-pbr-transparent', '<base-color-factor type="vec4d">0.92 0.92 0.9 0.45</base-color-factor><metallic-factor type="float">0.0</metallic-factor><roughness-factor type="float">0.15</roughness-factor>'),
       'x57-screen': ('Effects/model-pbr', '<base-color-factor type="vec4d">0.01 0.012 0.018 1</base-color-factor><metallic-factor type="float">0.0</metallic-factor><roughness-factor type="float">0.25</roughness-factor>')}


def alert_tone(path):
    """the cockpit's alert: a double tone. The aircraft's six alerts are spoken messages [ICD slide 40], and their words aren't published"""
    import wave
    r = 22050
    t = np.arange(int(0.16 * r)) / r
    beep = (0.9 * np.sin(2 * math.pi * 880 * t) * np.minimum(1, np.minimum(t, t[::-1]) / 0.01))
    y = np.r_[beep, np.zeros(int(0.08 * r)), beep]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(r)
        w.writeframes((y * 32000).astype('<i2').tobytes())


def effects(out):
    """HDR's materials. Anything else still draws, but the sun and the panel's lights go through these. The Tecnam's own for the other compositors, and its HDR switch"""
    d = os.path.join(out, 'Models', 'Effects')
    shutil.rmtree(d, ignore_errors=True)
    shutil.copytree(os.path.join(SRC, 'Models', 'Effects', 'Glass'), os.path.join(d, 'Glass'))
    for n, (base, par) in EFF.items():
        with open(os.path.join(d, n + '.eff'), 'w') as f:
            f.write('<?xml version="1.0"?>\n<PropertyList><name>%s</name><inherits-from>%s</inherits-from><parameters>%s</parameters></PropertyList>\n' % (n, base, par))
    # the instruments' glass, as the Bo 105's: clear in HDR, the dial dimmed by the faces' reflection and the sun's glint on it. Legacy transparency doesn't reach HDR, where it's an opaque grey
    with open(os.path.join(d, 'Glass', 'instrument-glass.eff'), 'w') as f:
        f.write('''<?xml version="1.0"?>
<PropertyList>
  <name>instrument-glass</name>
  <inherits-from>Aircraft/X-57/Models/Effects/Glass/glass</inherits-from>
  <technique n="128"><pass><blend><source-rgb>one</source-rgb></blend><define n="300"><name>BLEND_PREMULTIPLIED</name></define>
    <program><fragment-shader n="0">Aircraft/X-57/Shaders/HDR/instrument_glass.frag</fragment-shader></program></pass></technique>
</PropertyList>
''')
    sh = os.path.join(out, 'Shaders', 'HDR')
    os.makedirs(sh, exist_ok=True)
    shutil.copy(os.path.join(HERE, 'shaders', 'instrument_glass.frag'), sh)
    shutil.copy(os.path.join(HERE, 'shaders', 'lit.frag'), sh)
    # self-lit parts, as the MV-22B's and the Bo 105's: the texture emits lit-color x lit-level x lit-gain in cd/m2 and reflects the light on it as a lamp's dead front
    os.makedirs(os.path.join(d, 'Lit'))
    with open(os.path.join(d, 'Lit', 'lit.eff'), 'w') as f:
        f.write('''<?xml version="1.0"?>
<PropertyList>
  <name>lit</name>
  <inherits-from>Effects/model-default</inherits-from>
  <parameters><lit-color type="vec3d">1.0 1.0 1.0</lit-color><lit-level type="float">0.0</lit-level><lit-gain type="float">1.0</lit-gain><lit-reflectance type="float">1.0</lit-reflectance></parameters>
  <technique n="109"><pass><program><fragment-shader n="0">Aircraft/X-57/Shaders/HDR/lit.frag</fragment-shader></program>
    <uniform n="20"><name>lit_color</name><type>float-vec3</type><value><use>lit-color</use></value></uniform>
    <uniform n="21"><name>lit_level</name><type>float</type><value><use>lit-level</use></value></uniform>
    <uniform n="22"><name>lit_gain</name><type>float</type><value><use>lit-gain</use></value></uniform>
    <uniform n="23"><name>lit_reflectance</name><type>float</type><value><use>lit-reflectance</use></value></uniform>
  </pass></technique>
</PropertyList>
''')
    os.makedirs(os.path.join(out, 'Nasal'), exist_ok=True)
    shutil.copy(os.path.join(SRC, 'Nasal', 'hdr.nas'), os.path.join(out, 'Nasal', 'hdr.nas'))
    for n in ('displays.nas', 'cockpit.nas', 'doors.nas', 'lights.nas'):
        shutil.copy(os.path.join(HERE, 'nasal', n), os.path.join(out, 'Nasal', n))
    alert_tone(os.path.join(out, 'Sounds', 'x57-alert.wav'))
    os.makedirs(os.path.join(out, 'Docs'), exist_ok=True)
    shutil.copy(os.path.join(HERE, 'cockpit-guide.md'), os.path.join(out, 'Docs', 'X-57-cockpit-guide.md'))
    shutil.copy(os.path.join(HERE, '..', 'COPYING'), os.path.join(out, 'COPYING'))


def effect(name, objs):
    return '<effect><inherits-from>Aircraft/X-57/Models/Effects/%s</inherits-from>%s</effect>' % (name, ''.join('<object-name>%s</object-name>' % o for o in objs))


def split(sec, h):
    """an airfoil, Selig order, into the part ahead of the hinge and the part behind, each closed"""
    x, z = sec[:, 0], sec[:, 1]
    le = int(np.argmin(x))
    up, lo = sec[:le + 1][::-1], sec[le:]
    zu, zl = np.interp(h, up[:, 0], up[:, 1]), np.interp(h, lo[:, 0], lo[:, 1])
    fu, fl = up[up[:, 0] < h], lo[lo[:, 0] < h]
    front = np.r_[[[h, zu]], fu[::-1], fl[1:], [[h, zl]]]
    ru, rl = up[up[:, 0] > h], lo[lo[:, 0] > h]
    rear = np.r_[ru[::-1], [[h, zu], [h, zl]], rl]
    return front, rear


def resample(a, n):
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(a, axis=0).T))]
    t = np.linspace(0, s[-1], n)
    return np.c_[np.interp(t, s, a[:, 0]), np.interp(t, s, a[:, 1])]


def wing_parts(Mesh, wing, sec, p, x0, x1, z0):
    """(mesh, y of its middle, kind) along the span. Twist 2 deg at the root, none at the tip"""
    tip = p['y1']
    c = lambda y: p['c0'] + (p['c1'] - p['c0']) * abs(y) / tip
    le = lambda y: x0 + (x1 - x0) * abs(y) / tip
    tw = lambda y: 2.0 * (1 - abs(y) / tip)
    front, rear = (resample(a, 26) for a in split(sec, HINGE))
    out = []
    m = Mesh('wing-c')
    for sg in (1, -1):
        wing(m, 'c', sec, 0, sg * SEG[0], c(0), c(SEG[0]), le(0), le(SEG[0]), z0, tw(0), tw(SEG[0]))
    out.append((m, 0.0, 'fixed'))
    for sd, sg in (('l', -1), ('r', 1)):
        for i, (a, b) in enumerate(zip(SEG, SEG[1:])):
            ya, yb = sg * a, sg * b
            mid = a < Y_AIL[1] + 0.5
            m = Mesh('wing-%s-%d' % (sd, i))
            wing(m, 'w', front if mid else sec, ya, yb, c(ya), c(yb), le(ya), le(yb), z0, tw(ya), tw(yb))
            out.append((m, sg * 0.5 * (a + b), 'wing'))
            if mid:
                kind = 'flap' if b <= Y_FLAP[1] + 0.5 else 'ail'
                m = Mesh('%s-%s-%d' % (kind, sd, i))
                wing(m, 'w', rear, ya, yb, c(ya), c(yb), le(ya), le(yb), z0, tw(ya), tw(yb))
                y = 0.5 * (ya + yb)
                out.append((m, sg * 0.5 * (a + b), kind, (le(y) + HINGE * c(y), z0)))
    return out


def ride(an, obj, y_in, z_in, D):
    """an object on the bending mode: up with the mode, leaning with its slope. Listed before anything of its own, so that its own moves happen in the moved frame"""
    y = y_in / 12.0
    ph, sl = FX.phi_at(D, y), FX.slope_at(D, y)
    if not ph:
        return
    q = 'fdm/jsbsim/flex/q%s-ft' % ('L' if y_in < 0 else 'R')
    names = ''.join('<object-name>%s</object-name>' % o for o in (obj if isinstance(obj, (list, tuple)) else [obj]))
    an.append('<animation><type>translate</type>%s<property>%s</property><factor>%.5f</factor><axis><x>0</x><y>0</y><z>1</z></axis></animation>' % (names, q, ph * 0.3048))
    an.append('<animation><type>rotate</type>%s<property>%s</property><factor>%.4f</factor><center><x-m>0</x-m><y-m>%.4f</y-m><z-m>%.4f</z-m></center><axis><x>%d</x><y>0</y><z>0</z></axis></animation>'
              % (names, q, math.degrees(sl), y_in * 0.0254, z_in * 0.0254, 1 if y_in > 0 else -1))


def surface(an, obj, prop, x_in, z_in, sign):
    an.append('<animation><type>rotate</type><object-name>%s</object-name><property>%s</property><factor>%d</factor><center><x-m>%.4f</x-m><y-m>0</y-m><z-m>%.4f</z-m></center><axis><x>0</x><y>1</y><z>0</z></axis></animation>'
              % (obj, prop, sign, x_in * 0.0254, z_in * 0.0254))


# The X-57's panel, on the P2006T's. Sources: 20240006496 figs. 7-10, 20230005312 (the D175), 20240010539 (crew interface)
DROP = ('carbtemp', 'chrono', 'engines/fflow', 'engines/fuelp', 'engines/man', 'engines/oil', 'engines/tach', 'fuelgauge', 'kap140', 'KI204', 'kns80', 'kr87-adf', 'kt70', 'kx165', 'lights', 'warning', 'plaques')

def quad(name, y0, z0, y1, z1, x=0.0, tex=None, mat=0):
    v = [(y0, z0), (y1, z0), (y1, z1), (y0, z1)]
    uv = [(0, 0), (1, 0), (1, 1), (0, 1)]
    t = ['OBJECT poly', 'name "%s"' % name] + (['texture "%s"' % tex] if tex else []) + ['numvert 4'] + ['%.5f %.5f %.5f' % (x, z, -y) for y, z in v] + ['numsurf 1', 'SURF 0x20', 'mat %d' % mat, 'refs 4']
    return '\n'.join(t + ['%d %d %d' % (i, *uv[i]) for i in range(4)] + ['kids 0'])


def disc(name, r, x, tex=None, mat=0, n=40):
    """a round face: its picture's middle at the centre, nothing outside the circle"""
    a = [2 * math.pi * k / n for k in range(n)]
    t = ['OBJECT poly', 'name "%s"' % name] + (['texture "%s"' % tex] if tex else []) + ['numvert %d' % n] + ['%.5f %.5f %.5f' % (x, r * math.sin(q), -r * math.cos(q)) for q in a] + ['numsurf 1', 'SURF 0x20', 'mat %d' % mat, 'refs %d' % n]
    return '\n'.join(t + ['%d %.4f %.4f' % (k, 0.5 + 0.5 * math.cos(a[k]), 0.5 + 0.5 * math.sin(a[k])) for k in range(n)] + ['kids 0'])


def case(name, r, x0, x1, mat=5, n=40, lip=0.0025):
    """the instrument's round case and its bezel: a tube, and a flat ring round the face"""
    a = [2 * math.pi * k / n for k in range(n)]
    ring = lambda x, rr: ['%.5f %.5f %.5f' % (x, rr * math.sin(q), -rr * math.cos(q)) for q in a]
    v = ring(x0, r + lip) + ring(x1, r + lip) + ring(x1, r)
    s = []
    for k in range(n):
        j = (k + 1) % n
        for b0, b1 in ((0, n), (n, 2 * n)):
            s += ['SURF 0x20', 'mat %d' % mat, 'refs 4'] + ['%d 0 0' % i for i in (b0 + k, b0 + j, b1 + j, b1 + k)]
    return '\n'.join(['OBJECT poly', 'name "%s"' % name, 'numvert %d' % len(v)] + v + ['numsurf %d' % (2 * n)] + s + ['kids 0'])


def tri(name, r, mat=0, x=0.0):
    pts = [(-0.0016, -0.15 * r), (0.0016, -0.15 * r), (0.0, 0.85 * r)]
    t = ['OBJECT poly', 'name "%s"' % name, 'numvert 3'] + ['%.5f %.5f %.5f' % (x, z, -y) for y, z in pts] + ['numsurf 1', 'SURF 0x20', 'mat %d' % mat, 'refs 3']
    return '\n'.join(t + ['%d 0 0' % i for i in range(3)] + ['kids 0'])


MATS = ('MATERIAL "white" rgb 1 1 1 amb 1 1 1 emis 0.25 0.25 0.25 spec 0 0 0 shi 0 trans 0\n'
        'MATERIAL "needle" rgb 1 1 1 amb 1 1 1 emis 0.8 0.8 0.8 spec 0 0 0 shi 0 trans 0\n'
        'MATERIAL "green" rgb 0 1 0.2 amb 0 1 0.2 emis 0 1 0.2 spec 0 0 0 shi 0 trans 0\n'
        'MATERIAL "red" rgb 1 0.1 0 amb 1 0.1 0 emis 1 0.1 0 spec 0 0 0 shi 0 trans 0\n'
        'MATERIAL "amber" rgb 1 0.7 0 amb 1 0.7 0 emis 1 0.7 0 spec 0 0 0 shi 0 trans 0\n'
        'MATERIAL "screen" rgb 0.02 0.03 0.05 amb 0.02 0.03 0.05 emis 0.02 0.03 0.05 spec 0 0 0 shi 0 trans 0\n')


def ac(objs):
    return 'AC3Db\n' + MATS + 'OBJECT world\nkids %d\n%s\n' % (len(objs), '\n'.join(objs))


def face(path, label, low, high, step):
    from PIL import Image, ImageDraw, ImageFont
    S = 512
    im = Image.new('RGB', (S, S), (10, 10, 12))              # drawn on a round face, so its corners never show
    d = ImageDraw.Draw(im)
    f = ImageFont.load_default(size=52)
    g = ImageFont.load_default(size=50)
    c = S / 2
    n = int(round((high - low) / step))
    for i in range(n + 1):
        a = math.radians(-135 + 270 * i / n)
        s_, co = math.sin(a), math.cos(a)
        d.line((c + 205 * s_, c - 205 * co, c + 250 * s_, c - 250 * co), fill=(255, 255, 255), width=8)
        t = '%g' % ((low + i * step) / (100 if high >= 1000 else 1))
        w = d.textlength(t, font=g)
        d.text((c + 160 * s_ - w / 2, c - 160 * co - 28), t, font=g, fill=(255, 255, 255))
    w = d.textlength(label, font=f)
    d.text((c - w / 2, c + 80), label, font=f, fill=(255, 255, 255))
    im.save(path)


def plate(path, w, h, lines):
    """a panel's face with its labels: (x, y, text, size) in pixels"""
    from PIL import Image, ImageDraw, ImageFont
    im = Image.new('RGB', (w, h), (58, 62, 68))
    d = ImageDraw.Draw(im)
    d.rectangle((1, 1, w - 2, h - 2), outline=(120, 124, 130), width=2)
    for x, y, t, s in lines:
        d.text((x, y), t, font=ImageFont.load_default(size=s), fill=(240, 240, 240))
    im.save(path)


def text_block(name, y, z, size, fmt, prop=None):
    kind = ('<type type="string">number-value</type><format type="string">%s</format><property>%s</property><scale>1</scale><truncate>false</truncate>' % (fmt.replace('%%', '%'), prop)
            if prop else '<type type="string">literal</type><text type="string">%s</text>' % fmt)
    return ('<text><name>%s</name><offsets><x-m>0.007</x-m><y-m>%.4f</y-m><z-m>%.4f</z-m></offsets><alignment>left-baseline</alignment><axis-alignment>yz-plane</axis-alignment>%s'
            '<layout>left-to-right</layout><draw-text type="bool">true</draw-text><font>Helvetica.txf</font><character-size type="double">%.4f</character-size>'
            '<character-aspect-ratio type="double">1.0</character-aspect-ratio><font-resolution><width type="int">32</width><height type="int">32</height></font-resolution></text>' % (name, y, z, kind, size))


def full_colour(out):
    """HDR samples a one-channel texture as red and a two-channel one as red and green: every such texture under Models goes to RGB or RGBA, as on the Bo 105"""
    from PIL import Image
    n = 0
    for r, _, fs in os.walk(os.path.join(out, 'Models')):
        for f in fs:
            if f.lower().endswith(('.png', '.rgb', '.rgba', '.sgi')):
                p = os.path.join(r, f)
                try:
                    im = Image.open(p)
                except Exception:
                    continue
                if im.mode in ('L', 'LA', 'P', '1', 'I;16'):
                    im.convert('RGBA' if im.mode == 'LA' or 'transparency' in im.info else 'RGB').save(p, format=im.format)
                    n += 1
    print('  %d textures made full colour' % n)


# the P2006T's instruments stay in the P2006T's places, with their screws
TACH = os.path.join(SRC, 'Models', 'Interior', 'Panel', 'Instruments', 'engines', 'tach')


def sharpen(base):
    """the P2006T's faces are 256 pixels: four times that, resampled and sharpened, so the mipmaps don't blur them. The same drawings"""
    from PIL import Image, ImageFilter
    n = 0
    for r, _, fs in os.walk(os.path.join(base, 'Instruments')):
        for f in fs:
            if f.lower().endswith('.png'):
                p = os.path.join(r, f)
                im = Image.open(p)
                if max(im.size) <= 512:
                    k = 4 if max(im.size) <= 256 else 2
                    im = im.convert('RGBA' if 'A' in im.mode else 'RGB').resize((im.size[0] * k, im.size[1] * k), Image.LANCZOS).filter(ImageFilter.UnsharpMask(radius=3, percent=130, threshold=2))
                    im.save(p)
                    n += 1
    print('  %d instrument textures sharpened' % n)


def light_alike(base):
    """HDR's material on the instruments' solid parts, so that they take the light as the rest of the panel; the glass has its own"""
    for r, _, fs in os.walk(os.path.join(base, 'Instruments')):
        for f in fs:
            if not f.endswith('.xml'):
                continue
            p = os.path.join(r, f)
            s = open(p, errors='replace').read()
            m = re.search(r'<path>([^<]+\.ac)</path>', s)
            if not m or not os.path.exists(os.path.join(r, m.group(1))) or 'x57-face' in s:
                continue
            names = re.findall(r'^name "([^"]+)"', open(os.path.join(r, m.group(1)), errors='replace').read(), re.M)
            solid = [n for n in names if 'vitre' not in n.lower() and 'glass' not in n.lower()]
            if solid:
                open(p, 'w', errors='replace').write(s.replace('</PropertyList>', effect('x57-face', solid) + '\n</PropertyList>'))


def four(y, z, at, r, skip=()):
    """an instrument's corners, less those with a knob in them"""
    return [(y + a * at, z + b * at, r) for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)) if (a, b) not in skip]


LL, LR = (-1, -1), (1, -1)
SCREWS = (four(-0.436, -0.032, 0.0356, 0.0037) + four(-0.341, -0.032, 0.0356, 0.0037, (LR,)) + four(-0.245, -0.032, 0.0356, 0.0037, (LL,)) + four(-0.146, -0.032, 0.0356, 0.0037)    # airspeed, attitude, altitude, radio
          + four(-0.436, -0.128, 0.0356, 0.0037) + four(-0.341, -0.128, 0.0356, 0.0037, (LL, LR)) + four(-0.245, -0.128, 0.0356, 0.0037) + four(-0.146, -0.128, 0.0356, 0.0037))                  # turn, heading, climb, G


def screw_picture(path, level=92):
    """a cross-head, seen from above, black oxide by default: drawn large and shrunk"""
    from PIL import Image, ImageDraw
    S = 1024
    y, x = np.mgrid[0:S, 0:S] / (S / 2.0) - 1
    r2 = x * x + y * y
    g = level * (1 - 0.37 * r2 - 0.17 * (x + y))             # light from the upper left, falling off the dome
    im = Image.fromarray(np.clip(np.dstack((g, g + 1, g + 4)), 0, 255).astype('uint8'), 'RGB')
    d = ImageDraw.Draw(im)
    c = S / 2
    for a in (math.radians(30), math.radians(120)):
        u, v = 300 * math.cos(a), 300 * math.sin(a)
        n, m = 46 * math.sin(a), -46 * math.cos(a)
        d.polygon(((c - u + n, c - v + m), (c + u + n, c + v + m), (c + u - n, c + v - m), (c - u - n, c - v - m)), fill=(14, 14, 16))
    im.resize((256, 256), 1).save(path)


def screws(name, tex, heads, n=20, x0=0.0):
    """round domed heads, (y, z, radius) on a face at x0, as one object"""
    prof = ((1.0, 0.0), (1.0, 0.0005), (0.84, 0.0012), (0.5, 0.0014))       # the share of the radius, and the height
    v, s = [], []
    for y, z, r in heads:
        b = len(v)
        for f, x in prof:
            v += [(x + x0, y + f * r * math.cos(2 * math.pi * k / n), z + f * r * math.sin(2 * math.pi * k / n), 0.5 + 0.5 * f * math.cos(2 * math.pi * k / n), 0.5 + 0.5 * f * math.sin(2 * math.pi * k / n)) for k in range(n)]
        v.append((prof[-1][1] + x0, y, z, 0.5, 0.5))
        for k in range(n):
            j = (k + 1) % n
            for i in range(len(prof) - 1):
                s.append((b + i * n + k, b + i * n + j, b + (i + 1) * n + j, b + (i + 1) * n + k))
            s.append((b + (len(prof) - 1) * n + k, b + (len(prof) - 1) * n + j, b + len(prof) * n))
    t = ['OBJECT poly', 'name "%s"' % name, 'texture "%s"' % tex, 'numvert %d' % len(v)] + ['%.5f %.5f %.5f' % (q[0], q[2], -q[1]) for q in v] + ['numsurf %d' % len(s)]
    for q in s:
        t += ['SURF 0x30', 'mat 0', 'refs %d' % len(q)] + ['%d %.4f %.4f' % (i, v[i][3], v[i][4]) for i in q]
    return '\n'.join(t + ['kids 0'])


def unscrew(path):
    """take an instrument's own screws off, out at the corners of its bezel"""
    s = open(path, errors='replace').read()
    parts = re.split(r'(?=^OBJECT )', s, flags=re.M)
    for w, o in enumerate(parts):
        if not re.search(r'^name "fond"', o, re.M):
            continue
        l = o.split('\n')
        k = next(i for i, q in enumerate(l) if q.startswith('numvert'))
        n = int(l[k].split()[1])
        p = np.array([q.split()[1:3] for q in l[k + 1:k + 1 + n]], float)
        out = np.hypot(p[:, 0], p[:, 1]) > 0.0445
        near = np.zeros(n, bool)
        for a in (-1, 1):
            for b in (-1, 1):
                c = out & (np.sign(p[:, 0]) == a) & (np.sign(p[:, 1]) == b)
                if c.any():
                    near |= c & (np.hypot(*(p - np.median(p[c], axis=0)).T) < 0.0045)
        i, kept = k + 2 + n, []
        while l[i].startswith('SURF'):
            c = int(l[i + 2].split()[1])
            if not all(near[int(q.split()[0])] for q in l[i + 3:i + 3 + c]):
                kept += l[i:i + 3 + c]
            i += 3 + c
        parts[w] = '\n'.join(l[:k + 1 + n] + ['numsurf %d' % (len(kept) and sum(q.startswith('SURF') for q in kept))] + kept + l[i:])
    open(path, 'w', errors='replace').write(''.join(parts))


def plug(path, y, z, r, n=48):
    """fill a round hole in the panel: a disc of the panel's own surface, its picture carried on from the face around it"""
    s = open(path, errors='replace').read()
    parts = re.split(r'(?=^OBJECT )', s, flags=re.M)
    best = None
    for w, o in enumerate(parts):
        l = o.split('\n')
        k = next((i for i, q in enumerate(l) if q.startswith('numvert')), None)
        if k is None:
            continue
        c = int(l[k].split()[1])
        v = np.array([q.split()[:3] for q in l[k + 1:k + 1 + c]], float)
        d = np.array((re.findall(r'^loc (.*)', o, re.M) or ['0 0 0'])[0].split(), float)
        m = v + d
        i = k + 2 + c
        while l[i].startswith('SURF'):
            e = int(l[i + 2].split()[1])
            ref = [q.split() for q in l[i + 3:i + 3 + e]]
            idx = [int(q[0]) for q in ref]
            if np.abs(m[idx, 0]).max() < 0.0015:
                yz = np.c_[-m[idx, 2], m[idx, 1]]
                area = abs(np.linalg.det(np.c_[yz[:3], np.ones(3)]))
                gap = np.hypot(*(yz.mean(0) - (y, z)))
                if area > 1e-5 and (best is None or gap < best[0]):
                    best = (gap, w, k, c, d, l[i], l[i + 1], yz[:3], np.array(ref[:3])[:, 1:].astype(float))
            i += 3 + e
    _, w, k, c, d, flag, mat, yz, uv = best
    A = np.linalg.solve(np.c_[yz, np.ones(3)], uv)                # (y, z, 1) to the picture
    a = [2 * math.pi * q / n for q in range(n)]
    ring = [(y + r * math.cos(q), z + r * math.sin(q)) for q in a]
    l = parts[w].split('\n')
    ns = next(i for i, q in enumerate(l) if q.startswith('numsurf'))
    kids = next(i for i in range(len(l) - 1, -1, -1) if l[i].startswith('kids'))
    surf = [flag, mat, 'refs %d' % n] + ['%d %.5f %.5f' % (c + q, *(np.array((p_[0], p_[1], 1.0)) @ A)) for q, p_ in enumerate(ring)]
    new = ['%.6f %.6f %.6f' % (0.0004 - d[0], p_[1] - d[1], -p_[0] - d[2]) for p_ in ring]
    l = l[:k] + ['numvert %d' % (c + n)] + l[k + 1:k + 1 + c] + new + ['numsurf %d' % (int(l[ns].split()[1]) + 1)] + l[ns + 1:kids] + surf + l[kids:]
    parts[w] = '\n'.join(l)
    open(path, 'w', errors='replace').write(''.join(parts))


LEVER_SHIFT = 0.0215          # the four levers to the middle of the quadrant, with the mixture's two gone


def levers(base):
    """the pedestal: a torque lever and a propeller lever for each motor [20240006496, 20240010539]. The torque levers show the flight model's lever, regeneration and overdrive included"""
    p = os.path.join(base, 'Pedestal', 'pedestal.ac')
    s = open(p, errors='replace').read()
    head, *objs = re.split(r'(?=^OBJECT poly)', s, flags=re.M)
    objs = [o for o in objs if not re.search(r'^name "mixture-knob', o, re.M)]
    moved = lambda o: re.sub(r'^loc (\S+) (\S+) (\S+)$', lambda m: 'loc %s %s %.6f' % (m.group(1), m.group(2), float(m.group(3)) - LEVER_SHIFT), o, flags=re.M)
    objs = [moved(o) if re.search(r'^name "(throttle|rpm)-knob', o, re.M) else o for o in objs]
    open(p, 'w', errors='replace').write(re.sub(r'kids \d+', 'kids %d' % len(objs), head) + ''.join(objs))
    p = os.path.join(base, 'Pedestal', 'pedestal.xml')
    x = open(p, errors='replace').read()
    x = re.sub(r'<animation>(?:(?!</animation>).)*?engine\[\d\]/mixture</property>(?:(?!</animation>).)*?</animation>\s*', '', x, flags=re.S)
    x = re.sub(r'\s*<object-name>mixture-knob\.[LR]</object-name>', '', x)
    table = ''.join('<entry><ind>%g</ind><dep>%g</dep></entry>' % e for e in ((-0.174, 0), (0, 6), (1, 40), (1.66, 50)))
    x = re.sub(r'(<object-name>throttle-knob\.([LR])</object-name>\s*)<property>controls/engines/engine\[\d\]/throttle</property>\s*<factor> 50 </factor>',
               lambda m: '%s<property>fdm/jsbsim/prop/%s/lever</property><interpolation>%s</interpolation>' % (m.group(1), m.group(2).lower(), table), x)
    open(p, 'w', errors='replace').write(x)
    import hires
    hires.quadrant(base, LEVER_SHIFT * 1.875)


def needle_zero(ac):
    """which way the needle points unturned, deg clockwise from up"""
    s = open(ac, errors='replace').read()
    o = next(x for x in re.split(r'(?=^OBJECT )', s, flags=re.M) if re.search(r'^name "needle"', x, re.M))
    l = o.split('\n')
    i = next(k for k, q in enumerate(l) if q.startswith('numvert'))
    v = np.array([q.split()[:3] for q in l[i + 1:i + 1 + int(l[i].split()[1])]], float)
    tip = v[np.argmax(v[:, 1] ** 2 + v[:, 2] ** 2)]
    return math.degrees(math.atan2(-tip[2], tip[1]))


def dial(dst, label, low, high, step, unit=1, colour=(255, 255, 255)):
    """an X-57 dial on the P2006T's gauge: its picture four times the size, the face repainted, 270 deg from the lower left"""
    from PIL import Image, ImageDraw, ImageFont
    im = Image.open(os.path.join(TACH, 'tach.png')).convert('RGB')
    im = im.resize((1024, 1024), Image.LANCZOS)
    ImageDraw.Draw(im).rectangle((1000, 1000, 1023, 1023), fill=colour)     # for the needle: it samples here
    S = int(0.949 * 1024)
    d = Image.new('RGB', (S, S), (12, 12, 13))
    g = ImageDraw.Draw(d)
    c = S / 2
    n = int(round((high - low) / step))
    big = ImageFont.load_default(size=92)
    for i in range(n + 1):
        a = math.radians(-135 + 270 * i / n)
        s_, co = math.sin(a), math.cos(a)
        g.line((c + 372 * s_, c - 372 * co, c + 450 * s_, c - 450 * co), fill=(255, 255, 255), width=15)
        if i < n:
            b = math.radians(-135 + 270 * (i + 0.5) / n)
            g.line((c + 408 * math.sin(b), c - 408 * math.cos(b), c + 450 * math.sin(b), c - 450 * math.cos(b)), fill=(255, 255, 255), width=8)
        tx = '%g' % ((low + i * step) / unit)
        w = g.textlength(tx, font=big)
        g.text((c + 290 * s_ - w / 2, c - 290 * co - 54), tx, font=big, fill=(255, 255, 255))
    for i, part in enumerate(label.split('|')):
        fs = ImageFont.load_default(size=70 if i == 0 else 52)
        w = g.textlength(part, font=fs)
        g.text((c - w / 2, c + 30 + 76 * i), part, font=fs, fill=(255, 255, 255))
    m = Image.new('L', (S, S), 0)
    ImageDraw.Draw(m).ellipse((0, 0, S - 1, S - 1), fill=255)
    im.paste(d, (int(0.023 * 1024), int((1 - 0.973) * 1024)), m)
    im.save(dst)


def gauge(d, n, label, prop, low, high, step, k=1.0, colour=(255, 255, 255)):
    """the P2006T's gauge with a new face and property; k its size"""
    s = open(os.path.join(TACH, 'tach.ac'), errors='replace').read().replace('texture "tach.png"', 'texture "%s.png"' % n)
    if k != 1.0:
        out, c = [], 0
        for l in s.split('\n'):
            q = l.split()
            if c:
                out.append('%s %.6f %.6f' % (q[0], k * float(q[1]), k * float(q[2])))
                c -= 1
                continue
            if l.startswith('numvert'):
                c = int(q[1])
            out.append(l)
        s = '\n'.join(out)
    open(os.path.join(d, n + '.ac'), 'w', errors='replace').write(s)
    import hires
    hires.flat_uv(os.path.join(d, n + '.ac'), ('needle',))
    dial(os.path.join(d, n + '.png'), label, low, high, step, 100 if high >= 1000 else 1, colour)
    z = needle_zero(os.path.join(d, n + '.ac'))
    open(os.path.join(d, n + '.xml'), 'w').write('<?xml version="1.0"?>\n<PropertyList><path>%s.ac</path><animation><type>rotate</type><object-name>needle</object-name><property>%s</property>'
        '<interpolation><entry><ind>%g</ind><dep>%g</dep></entry><entry><ind>%g</ind><dep>%g</dep></entry></interpolation><axis><x>-1</x><y>0</y><z>0</z></axis><center><x-m>0</x-m><y-m>0</y-m><z-m>0</z-m></center></animation>%s%s</PropertyList>\n'
        % (n, prop, low, -135 - z, high, 135 - z, effect('x57-face', ('face', 'fond', 'needle')), effect('Glass/instrument-glass', ('vitre',))))


def panel(out):
    base = os.path.join(out, 'Models', 'Interior', 'Panel')
    p = os.path.join(base, 'panel.xml')
    t = open(p, errors='replace').read()
    t = re.sub(r'<model>(?:(?!</model>).)*?<path>Instruments/(?:%s)[^<]*</path>.*?</model>\s*' % '|'.join(DROP), '', t, flags=re.S)
    import x57panel
    t = x57panel.place(t, base)
    t = t.replace('</PropertyList>', '<model><path>X57/x57-panel.xml</path></model>\n</PropertyList>')
    # the P2006T has its panel and glare shield cast no shadow: under HDR the sun through the windshield then lit everything under
    # the glare shield, the inside of its ends too, as if it weren't there. They cast shadows
    t = re.sub(r'<animation>\s*<type>noshadow</type>.*?</animation>\s*', '', t, flags=re.S)
    open(p, 'w', errors='replace').write(t)
    sharpen(base)
    import hires
    hires.paint(base)
    for n in ('ai', 'alt', 'asi'):
        unscrew(os.path.join(base, 'Instruments', n, n + '.ac'))
    plug(os.path.join(base, 'panel.ac'), -0.146, -0.032, 0.0425)                  # the Tecnam's ADF hole
    levers(base)
    y = os.path.join(base, 'Instruments', 'yokes', 'all-yokes.xml')                # the real right panel has no yoke [20240010539 fig. 4]
    t = open(y, errors='replace').read()
    open(y, 'w', errors='replace').write(re.sub(r'<model>(?:(?!</model>).)*?<y-m>\s*0\.28442\s*</y-m>(?:(?!</model>).)*?</model>\s*', '', t, flags=re.S))
    light_alike(base)
    d = os.path.join(base, 'X57')
    os.makedirs(d, exist_ok=True)
    import x57panel
    x57panel.build(base, d)
    x57panel.clicks(base)


FAILS = (('Motor power', 'motor-on-%s', 1), ('Controller A', 'cmc-%sa', 1), ('Controller B', 'cmc-%sb', 1), ('Propeller forces gone', 'no-prop-forces-%s', 0), ('Hub frozen', 'hub-fixed-%s', 0),
         ('Instant feather', 'feather-now-%s', 0), ('Torque command stuck', 'cmd-stuck-%s', 0), ('Runaway to fine', 'hub-fine-%s', 0))


def lever_extra(e):
    """the property-rule expression for ctl/lever-extra: the torque lever less the throttle (Systems/x57-rules.xml's header)"""
    c = lambda n: '<property>/controls/engines/engine[%d]/%s</property>' % (e, n)
    val = lambda x: '<value>%g</value>' % x
    tag = lambda t, *a: '<%s>%s</%s>' % (t, ''.join(a), t)
    neg = lambda x: tag('product', val(-1), x)
    det = '<property>/sim/model/x57/regen-detent[%d]</property>' % e
    normal = tag('sum', tag('product', '<property>/fdm/jsbsim/ctl/overdrive</property>', tag('table', c('throttle'), '<entry><ind>0.9</ind><dep>0</dep></entry><entry><ind>1</ind><dep>0.66</dep></entry>')),
                 tag('product', val(-1), '<property>/fdm/jsbsim/ctl/regen</property>', tag('table', c('throttle'), '<entry><ind>0</ind><dep>0.174</dep></entry><entry><ind>0.1</ind><dep>0</dep></entry>')))
    # below the detent, 0 at it to 1 at the stop; 1 throughout when the detent sits at the axis's end
    frac = tag('max', val(0), tag('min', val(1), tag('max', tag('div', tag('sum', det, neg(c('throttle'))), tag('max', det, val(0.02))), tag('sum', val(1), tag('product', val(-50), det)))))
    off = lambda n: tag('sum', val(1), neg(c(n)))
    return tag('sum', tag('product', off('reverser'), off('cutoff'), normal),
               tag('product', c('reverser'), off('cutoff'), val(-1.174), c('throttle')),
               tag('product', c('cutoff'), tag('sum', neg(c('throttle')), tag('product', val(-0.174), frac))))


def dialog(out):
    """NASA's failure list and the systems the overhead holds: a window from the X-57 menu"""
    d = os.path.join(out, 'gui', 'dialogs')
    os.makedirs(d, exist_ok=True)
    cb = lambda r, c, pr, lab: ('<checkbox><row>%d</row><col>%d</col><halign>left</halign><label>%s</label><property>/fdm/jsbsim/ctl/%s</property><live>true</live><binding><command>dialog-apply</command></binding></checkbox>' % (r, c, lab, pr))
    tx = lambda r, c, lab: '<text><row>%d</row><col>%d</col><halign>left</halign><label>%s</label></text>' % (r, c, lab)
    rows = [tx(0, 0, 'Ticked = working where it says so, else failed'), tx(0, 1, 'Left'), tx(0, 2, 'Right')]
    for i, (lab, pr, ok) in enumerate(FAILS, 1):
        rows += [tx(i, 0, lab + (' on' if ok else '')), cb(i, 1, pr % 'l', ''), cb(i, 2, pr % 'r', '')]
    n = len(FAILS) + 1
    for j, (lab, l_, r_) in enumerate((('Contactor A', 'contactor-al', 'contactor-ar'), ('Contactor B', 'contactor-bl', 'contactor-br')), n):
        rows += [tx(j, 0, lab + ' closed (Mod IV)'), cb(j, 1, l_, ''), cb(j, 2, r_, '')]
    n += 2
    rows += [tx(n, 0, 'MAIN BATT A, B closed'), cb(n, 1, 'bus-a', ''), cb(n, 2, 'bus-b', '')]
    rows += [tx(n + 1, 0, 'Overdrive, regeneration: lever ends'), cb(n + 1, 1, 'overdrive', ''), cb(n + 1, 2, 'regen', '')]
    rows += [tx(n + 2, 0, 'DC converter A, B failed'), cb(n + 2, 1, 'fail/conv-a', ''), cb(n + 2, 2, 'fail/conv-b', '')]
    rows += [tx(n + 3, 0, 'Essential battery failed'), cb(n + 3, 1, 'fail/ess-batt', '')]
    with open(os.path.join(d, 'x57-systems.xml'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<PropertyList><name>x57-systems</name><layout>vbox</layout><text><label>X-57 systems and NASA\'s failures</label></text><hrule/><group><layout>table</layout>%s</group><hrule/>'
                '<text><label>Cruise rpm: the propeller lever. Feather: the same lever, back through its gate.</label></text>'
                '<text><label>Regeneration over the whole throttle: Delete or Backspace, or a joystick\'s reverser. A Warthog throttle: each lever past its idle detent.</label></text><button><legend>Close</legend><default>true</default><key>Esc</key><binding><command>dialog-close</command></binding></button></PropertyList>\n' % ''.join(rows))
    with open(os.path.join(out, 'Systems', 'x57-rules.xml'), 'w') as f:
        f.write('''<?xml version="1.0"?>
<!-- the cockpit's levers to the flight model: written by fdm/mkmodel.py -->
<!-- The torque lever is the throttle plus lever-extra. Normally: overdrive in the throttle's last tenth, and regeneration in its first when the menu
     ticks it. With the engine's reverser on (Delete, or a joystick's reverser button: Nasal/cockpit.nas), the whole throttle runs through the regeneration
     region, idle at 0 to full regeneration, -17.4 percent of the lever [SIM], at 1. With its cutoff on (a Warthog throttle's lever lifted past its idle
     detent), regeneration runs from where the lever crossed the detent, /sim/model/x57/regen-detent, to full at the stop; full at once if the axis
     doesn't move past the detent -->
<PropertyList>
%s
</PropertyList>
''' % ('\n'.join('''<filter><type>gain</type><gain>1</gain><input><expression><table><property>/controls/engines/engine[%(e)d]/propeller-pitch</property><entry><ind>0.3</ind><dep>1700</dep></entry><entry><ind>1</ind><dep>2700</dep></entry></table></expression></input><output>/fdm/jsbsim/ctl/cruise-rpm-%(s)s</output></filter>
<filter><type>gain</type><gain>1</gain><input><expression><table><property>/controls/engines/engine[%(e)d]/propeller-pitch</property><entry><ind>0.14</ind><dep>1</dep></entry><entry><ind>0.16</ind><dep>0</dep></entry></table></expression></input><output>/fdm/jsbsim/ctl/feather-%(s)s</output></filter>
<filter><type>gain</type><gain>1</gain><input><expression>%(extra)s</expression></input><output>/fdm/jsbsim/ctl/lever-extra-%(s)s</output></filter>''' % dict(e=e, s=s, extra=lever_extra(e)) for e, s in ((0, 'l'), (1, 'r')))))
