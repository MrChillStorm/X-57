"""The X-57's pilot: built in Blender 4.3 with MPFB 2 and its CC0 system assets, posed in the cockpit as the model has it, cut at his joints
for FlightGear, with the arm and leg angles that keep his hands on the yoke and torque levers and his feet on the rudder pedals.

    "/Applications/Blender 4.3.app/Contents/MacOS/Blender" -b --python fdm/mkpilot.py [-- '{"render": true}']

writes data/pilot/: pilot.ac (his parts, interior frame), its textures and pilot.json (joints, parts, animation tables), which
x57pilot.py turns into the aircraft's Models/Pilot. Run it again only to change him: the build uses its output.

Sources and licences: MPFB's base mesh ("explicitly released as CC0 in september 2020"), its game-engine rig and weights (CC0),
the makehuman system assets' middle-aged skin, low-poly brown eyes and the black sock of shoes03, all CC0. The flight suit, its
pockets and zipper, his hair and eyebrows (painted), the headset, the sunglasses and the seatbelt are mine; his work shoes a
photogrammetry scan of a pair of this aircraft's author's (mkshoes.py). No NASA name or insignia.

Fit. He is a median man, 1.75 m (MPFB height 0.52). The P2006T model's seat was high for its cabin, so x57cabin.py lowers it 8 cm
(the P2006T's seats adjust up and down); the hip is placed on it. Leaning back 17 deg (the seat's own back is 15 deg), his head is
3.2 cm under the roof's lining, the headset's band 1.9 cm, his eyes 4 cm above and 3 cm aft of FlightGear's cockpit eye point (he is
hidden in that view). Feet on x57pedals.py's bars, which are placed for him: his knees 6.8 cm under the yoke at neutral; at full
aileron its horns pass them by 4 to 9 mm. His left shoulder comes 28 deg forward, reaching for the yoke: wherever FlightGear puts it
(stab-norm to 0.33 forward on the ground) his hand is on its grip; only beyond, full forward with full left aileron, would he fall
2.5 cm short.
"""
import bpy, bmesh, math, json, os, re, shutil, sys
from mathutils import Vector, Matrix, Euler
from mathutils.bvhtree import BVHTree
from types import SimpleNamespace
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import x57pedals as PD

ARGS = json.loads(sys.argv[sys.argv.index('--') + 1]) if '--' in sys.argv and len(sys.argv) > sys.argv.index('--') + 1 else {}
OUT = os.path.join(HERE, 'data', 'pilot')
CHECK = ARGS.get('check', os.path.join(HERE, 'data', 'pilot', 'check'))
MODELS = os.path.join(HERE, '..', 'X-57', 'Models')
ASSETS = os.path.expanduser('~/Library/Application Support/Blender/4.3/extensions/.user/blender_org/mpfb/data')

# the cockpit, interior frame (x aft, y right, z up)
PANEL = Vector((-2.74446, 0.0, -0.13289))
YOKE_ORIGIN = PANEL + Vector((-0.00024, -0.28442, -0.21115))          # the yoke's own frame: it slides along x, turns about x here
PEDESTAL = PANEL + Vector((0.03825, 0.0, -0.47427))
LEVER_PIVOT = PEDESTAL + Vector((0.01796, 0.0, -0.10735))             # the torque levers turn about -y here
CONTEXT = [('Interior/Seats/seat.ac', Vector((-2.08, -0.29, -0.39))), ('Interior/Panel/Instruments/yokes/yoke.ac', YOKE_ORIGIN),
           ('Interior/Panel/Pedestal/pedestal.ac', PEDESTAL), ('Interior/interior.ac', Vector((0, 0, 0))),
           ('Interior/Panel/X57/x57-panel.ac', PANEL)]
HIP = Vector((-2.20, PD.PILOT_Y, -0.555))       # the hip joints' midpoint: 2 cm into the cushion, the buttocks against the backrest
RECLINE = math.radians(17.0)
PROTRACT = math.radians(28.0)                   # his left shoulder forward, reaching for the yoke: full forward, his arm reaches it
HEAD_TILT = math.radians(8.0)
HEIGHT = 0.52
GRIP_L = Vector((-2.624, -0.4225, -0.275))      # the yoke's left grip's axis (x -2.624, y -0.4225, z -0.36..-0.20), its height mid-grip
GRIP_R = 0.015                                  # its radius (yoke.ac)
KNOBS = Vector((-2.470, -0.026, -0.490))        # the right palm over both torque knobs (their tops at z -0.517)
SOLE = 45.0                                     # the soles along the pedals
PALM = 0.085                                    # wrist to palm centre
SUIT_OFF = 0.012                                # the flight suit stands this far off the skin
SAGE = np.array((101, 110, 86), float)          # sage green, as flight suits are


def clear():
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o)
    for c in (bpy.data.meshes, bpy.data.materials, bpy.data.images):
        for d in list(c):
            if d.users == 0:
                c.remove(d)


def context():
    objs = []
    for path, off in CONTEXT:
        before = set(bpy.data.objects)
        bpy.ops.import_scene.import_ac3d(filepath=os.path.join(MODELS, path))
        for o in set(bpy.data.objects) - before:
            if o.parent is None:
                o.location += off
            o['ctx'] = True
            o.color = (0.62, 0.66, 0.7, 1)
            if o.type == 'MESH' and ('vitre' in o.name.lower() or o.name.startswith('HDR')):
                o.hide_render = True
            objs.append(o)
    return objs


def human():
    from bl_ext.blender_org.mpfb.services import HumanService
    macro = {"gender": 1.0, "age": 0.62, "muscle": 0.55, "weight": 0.5, "proportions": 0.5, "height": HEIGHT, "cupsize": 0.5, "firmness": 0.5,
             "race": {"asian": 0.2, "caucasian": 0.6, "african": 0.2}}
    body = HumanService.create_human(macro_detail_dict=macro, feet_on_ground=True, scale=0.1)
    rig = HumanService.add_builtin_rig(body, 'game_engine', import_weights=True)
    eyes = HumanService.add_mhclo_asset(os.path.join(ASSETS, 'eyes', 'low-poly', 'low-poly.mhclo'), body, asset_type='Eyes', subdiv_levels=0)
    boots = HumanService.add_mhclo_asset(os.path.join(ASSETS, 'clothes', 'shoes03', 'shoes03.mhclo'), body, asset_type='Clothes', subdiv_levels=0)
    rig.rotation_euler = (0, 0, math.radians(-90))
    bpy.context.view_layer.update()
    hip = (rig.matrix_world @ rig.data.bones['thigh_l'].head_local + rig.matrix_world @ rig.data.bones['thigh_r'].head_local) / 2
    rig.location += HIP - hip
    bpy.context.view_layer.update()
    return body, rig, eyes, boots


# ------------------------------------------------------------------------------------------------- his shoes
# Black leather work shoes with two hook-and-loop straps, as a flight suit is worn with: a photogrammetry scan of a pair, UK 7.5,
# made by mkshoes.py (data/pilot/shoes.obj and shoes.png). Each is fitted to his foot in place of shoes03's shoe, in its rest shape
# before he is posed: its heel on shoes03's heel, its sole on shoes03's sole, along shoes03's footprint, so his feet sit on the
# pedals as before; each of its vertices takes the bone weights of shoes03's nearest, so it rides his foot as shoes03 did. Of
# shoes03 only the black sock above the shoe stays, its picture a plain black corner of the shoes' atlas
SOCK_UV = (0.75, 0.06)


def islands(bm):
    """the mesh's loose pieces, each a list of its vertices"""
    seen, out = set(), []
    for v in bm.verts:
        if v in seen:
            continue
        comp, stack = [], [v]
        seen.add(v)
        while stack:
            u = stack.pop()
            comp.append(u)
            for e in u.link_edges:
                w = e.other_vert(u)
                if w not in seen:
                    seen.add(w)
                    stack.append(w)
        out.append(comp)
    return out


def scan_shoes(boots):
    """the scanned shoes into the boots' mesh, shoes03's shoes out of it; the shoes' atlas as pilot-boots.png"""
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=os.path.join(OUT, 'shoes.obj'), forward_axis='Y', up_axis='Z')
    scans = {o.name.split('.')[0]: o for o in set(bpy.data.objects) - before if o.type == 'MESH'}
    bm = bmesh.new()
    bm.from_mesh(boots.data)
    uv = bm.loops.layers.uv.active
    dl = bm.verts.layers.deform.active
    z0 = min(v.co.z for v in bm.verts)
    pieces = islands(bm)
    sock = [p for p in pieces if min(v.co.z for v in p) > z0 + 0.04 and max(v.co.z for v in p) > z0 + 0.15]
    shoe = [v for p in pieces if p not in sock for v in p]
    for s, tag in ((1, 'l'), (-1, 'r')):
        own = [v for v in shoe if v.co.x * s > 0]
        low = np.array([v.co[:2] for v in own if v.co.z < z0 + 0.006])
        c = low.mean(0)
        w, U = np.linalg.eigh(np.cov((low - c).T))
        ax = U[:, 1] if U[1, 1] > 0 else -U[:, 1]                      # along the footprint, toward the heel (+y)
        heel = max((v.co.x - c[0]) * ax[0] + (v.co.y - c[1]) * ax[1] for v in own)
        y_ = Vector((ax[0], ax[1], 0.0))
        x_ = y_.cross(Vector((0, 0, 1)))
        sc = scans['shoe-' + tag]
        sm = sc.data
        y_heel = max(v.co.y for v in sm.vertices)
        O = Vector((c[0], c[1], z0 - 0.001)) + y_ * (heel - y_heel)
        src = [(v.co.copy(), dict(v[dl])) for v in own]
        new = []
        for v in sm.vertices:
            p = O + x_ * v.co.x + y_ * v.co.y + Vector((0, 0, v.co.z))
            nv = bm.verts.new(p)
            co, wt = min(src, key=lambda q: (q[0] - p).length_squared)
            for g, x in wt.items():
                nv[dl][g] = x
            new.append(nv)
        suv = sm.uv_layers.active.data
        for f in sm.polygons:
            try:
                nf = bm.faces.new([new[i] for i in f.vertices])
            except ValueError:
                continue
            for l, li in zip(nf.loops, f.loop_indices):
                l[uv].uv = suv[li].uv
        print("MKPILOT shoe %s: scanned, %d faces, fitted to shoes03's footprint (%.1f deg off his axis), its heel on shoes03's" % (
            tag, len(sm.polygons), math.degrees(math.atan2(ax[0], ax[1]))))
    bmesh.ops.delete(bm, geom=shoe, context='VERTS')
    for p in sock:
        for v in p:
            for l in v.link_loops:
                l[uv].uv = SOCK_UV
    bm.normal_update()
    bm.to_mesh(boots.data)
    bm.free()
    boots.data.update()
    for o in scans.values():
        bpy.data.objects.remove(o)
    shutil.copy(os.path.join(OUT, 'shoes.png'), os.path.join(OUT, 'pilot-boots.png'))


def store_rest(obj):
    """each vertex's rest position (MPFB's own frame: x his left, -y his front, z up) as an attribute, before posing"""
    dg = bpy.context.evaluated_depsgraph_get()
    masks = [m for m in obj.modifiers if m.type == 'MASK']
    for m in masks:
        m.show_viewport = False
    bpy.context.view_layer.update()
    ev = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    co = np.array([v.co[:] for v in ev.data.vertices])
    for m in masks:
        m.show_viewport = True
    if len(co) != len(obj.data.vertices):
        raise SystemExit('mkpilot: %s changed its vertex count when evaluated' % obj.name)
    a = obj.data.attributes.new('rest', 'FLOAT_VECTOR', 'POINT')
    a.data.foreach_set('vector', co.ravel())
    return co


# ------------------------------------------------------------------------------------------------- the pose
def W(rig, name, end='head'):
    pb = rig.pose.bones[name]
    return rig.matrix_world @ (pb.head if end == 'head' else pb.tail)


def orient(rig, name, y, z):
    """the bone along y (world), its Z toward z as nearly as it can, its head where its parent put it"""
    y = y.normalized()
    z = (z - y * z.dot(y)).normalized()
    R = Matrix((y.cross(z), y, z)).transposed()
    pb = rig.pose.bones[name]
    m = (rig.matrix_world.to_3x3().inverted() @ R).to_4x4()
    m.translation = pb.matrix.translation
    pb.matrix = m
    bpy.context.view_layer.update()


def rest_axes(rig, name):
    return rig.matrix_world.to_3x3() @ rig.data.bones[name].matrix_local.to_3x3()


def ik(P, a, b, T, pole):
    d = T - P
    L = min(d.length, (a + b) * 0.999)
    u = d.normalized()
    ca = (a * a + L * L - b * b) / (2 * a * L)
    v = (pole - u * pole.dot(u)).normalized()
    return P + a * (ca * u + math.sqrt(max(0.0, 1 - ca * ca)) * v), P + u * L


def foot_target(side, lean):
    """the ball of the foot on its pedal's bar, the sole along the pedal, at a lean"""
    s = -1 if side == 'l' else 1
    bx, bz = PD.bar(lean)
    a = math.radians(lean)
    sole = Vector((-math.sin(a), 0, math.cos(a)))       # up the pedal, heel to ball
    return Vector((bx, PD.PILOT_Y + s * PD.SPREAD, bz)), sole


def arm_target(side, T=Matrix.Identity(4)):
    """the palm's centre, the hand's direction and its palm normal, moved by T (the yoke's or levers' motion)"""
    if side == 'l':
        g, d, n = HOLD['palm'], HOLD['y'], HOLD['z']
    else:
        g, d, n = KNOBS, Vector((-1, -0.10, -0.45)), Vector((0, 0, -1))
    R = T.to_3x3()
    return T @ g, (R @ d).normalized(), R @ n


HOLD = {}
POLE = {'l': Vector((0.3, -0.5, -1)), 'r': Vector((0.4, 0.4, -1))}


PHALANX = (0.0085, 0.0075, 0.0065)              # his fingers' half thickness about their bones, proximal to tip, less 1 mm of squeeze (estimate)
FLEX_MAX = (90, 110, 80)                        # their joints' flexion at most, knuckle to tip, degrees
GRIP_AT = -0.018                                # the grip across his palm this far behind his knuckles' joints
SQUEEZE = 0.010                                 # and this far into his palm's pad, as a firm grip presses it: MPFB's palm stands 22 mm
                                                # below the knuckles' joints, too thick for his fingers to close round a bar resting on it


def skin_points(rig, body):
    """his left hand's and fingers' skin, now, in the hand bone's axes from his wrist (MPFB's helper geometry left out)"""
    masks = [m for m in body.modifiers if m.type == 'MASK']
    for m in masks:
        m.show_viewport = False
    bpy.context.view_layer.update()
    ev = body.evaluated_get(bpy.context.evaluated_depsgraph_get())
    P = np.array([(body.matrix_world @ v.co)[:] for v in ev.data.vertices])
    if 'keep' not in HOLD:
        keep = np.ones(len(P), bool)                          # what MPFB's masks show, not its helpers (joint cubes, tights)
        for m in masks:
            gi = body.vertex_groups[m.vertex_group].index
            inside = np.array([any(g.group == gi and g.weight > m.threshold for g in v.groups) for v in body.data.vertices])
            keep &= inside != m.invert_vertex_group
        HOLD['keep'], HOLD['cls'] = keep, bone_of(body, body.data)
    for m in masks:
        m.show_viewport = True
    bpy.context.view_layer.update()
    keep, cls = HOLD['keep'], HOLD['cls']
    fist = keep & np.array([n.startswith(('hand_', 'index_', 'middle_', 'ring_', 'pinky_', 'thumb_')) and n.endswith('_l') for n in cls])
    Rh, w = rot(rig, 'hand_l'), W(rig, 'hand_l')
    return (P[fist] - np.array(w[:])) @ np.array(Rh), cls[fist]


def to_bar(x, B, k):
    """a point's distance from the grip's axis (through B along k)"""
    d = x - B
    return (d - k * d.dot(k)).length


def seg_to_bar(a, b, B, k):
    """a segment's least distance from the grip's axis"""
    a2, b2 = a - B - k * (a - B).dot(k), b - B - k * (b - B).dot(k)
    e = b2 - a2
    t = 0.0 if e.length_squared < 1e-12 else max(0.0, min(1.0, -a2.dot(e) / e.length_squared))
    return (a2 + e * t).length


def phalanges(rig, finger, angles):
    """a left finger's (or thumb's) three joints and tip, in the hand bone's axes from the wrist, at these flexions: degrees about
    each bone's x, or (x, y, z) Euler angles"""
    H = rig.pose.bones['hand_l']
    Hi = H.matrix.inverted()
    M, out = H.matrix, []
    parent = H.bone
    for i, ang in enumerate(angles):
        b = rig.data.bones['%s_%02d_l' % (finger, i + 1)]
        e = (ang, 0, 0) if isinstance(ang, (int, float)) else ang
        M = M @ (parent.matrix_local.inverted() @ b.matrix_local) @ Euler([math.radians(a) for a in e], 'XYZ').to_matrix().to_4x4()
        out.append(Hi @ M.translation)
        parent = b
    out.append(Hi @ (M @ Vector((0, parent.length, 0))))
    s = rig.matrix_world.to_scale()[0]                       # the rig's own scale, if any: world metres
    return [x * s for x in out]


THUMB = 0.0095                                  # his thumb's half thickness about its bones (estimate)


def wrap_thumb(rig, B, k, f, p, curls, palm):
    """his thumb round the grip's other side and over his index finger, as the user's photograph of a bar held like a yoke has it:
    its tip toward the back of the index's middle bone, as far round as it reaches, its bones clear of the grip, of his fingers and
    of his palm (palm: its skin along p from the grip's axis). Joint angles by a plain descent from a loose fist's"""
    J = phalanges(rig, 'index', curls['index'])
    mid = J[1].lerp(J[2], 0.7)                      # on the index's middle bone, near its last joint: as far round as his thumb reaches
    r = mid - B - k * (mid - B).dot(k)
    target = mid + r.normalized() * (PHALANX[1] + THUMB) + k * 0.006
    bones = [phalanges(rig, n, curls[n]) for n in ('index', 'middle', 'ring', 'pinky')]

    def cost(v):
        T = phalanges(rig, 'thumb', [v[0:3], (v[3], 0, v[4]), (v[5], 0, 0)])
        c = 100 * (T[3] - target).length_squared / 1e-4      # cm squared
        for i in (1, 2):                                      # its two outer bones clear of the grip
            c += 1e4 * max(0.0, GRIP_R + THUMB - seg_to_bar(T[i], T[i + 1], B, k)) ** 2 / 1e-6 * 1e-2
        for i in (1, 2):                                      # and of his fingers' bones, and out of his palm
            for t in np.linspace(0.3 if i == 1 else 0.0, 1, 5):
                x = T[i].lerp(T[i + 1], float(t))
                for F in bones:
                    for jn in range(3):
                        a2, b2 = F[jn], F[jn + 1]
                        e = b2 - a2
                        u = max(0.0, min(1.0, (x - a2).dot(e) / e.length_squared))
                        d = (x - a2 - e * u).length
                        c += 1e2 * max(0.0, THUMB + PHALANX[jn] - 0.002 - d) ** 2 / 1e-6
                if (x - B).dot(f) < 0.010:
                    c += 1e2 * max(0.0, palm + THUMB - (x - B).dot(p)) ** 2 / 1e-6
        return c

    lo = np.array((-60, -60, -60, -10, -30, -10.0))
    hi = np.array((90, 90, 90, 80, 30, 90.0))
    v = np.array((20, 35, 0, 40, 0, 50.0))
    best = cost(v)
    for step in (16, 8, 4, 2, 1, 0.5):
        better = True
        while better:
            better = False
            for i in range(6):
                for sgn in (1, -1):
                    w = v.copy()
                    w[i] = min(hi[i], max(lo[i], w[i] + sgn * step))
                    cw = cost(w)
                    if cw < best - 1e-9:
                        v, best, better = w, cw, True
    T = phalanges(rig, 'thumb', [v[0:3], (v[3], 0, v[4]), (v[5], 0, 0)])
    print('MKPILOT his left thumb: %s; its tip %.1f mm from its place on his index finger; its bones %.1f, %.1f mm off the grip' % (
        [round(float(a)) for a in v], 1000 * (T[3] - target).length, *(1000 * (seg_to_bar(T[i], T[i + 1], B, k) - GRIP_R) for i in (1, 2))))
    return [[float(a) for a in v[0:3]], [float(v[3]), 0.0, float(v[4])], [float(v[5]), 0.0, 0.0]]


def grasp(rig, B, k):
    """each finger closed round the grip, as a hand closes on a bar: its knuckle bends until its first bone meets the bar, then its
    middle joint until the second bone does, then its last; each joint no further than FLEX_MAX"""
    curls = {}
    for finger in ('index', 'middle', 'ring', 'pinky'):
        ang = [0.0, 0.0, 0.0]
        for j in range(3):
            for a in np.arange(0.0, FLEX_MAX[j] + 0.1, 0.5):
                ang[j] = float(a)
                J = phalanges(rig, finger, ang)
                if seg_to_bar(J[j], J[j + 1], B, k) <= GRIP_R + PHALANX[j]:
                    break
        curls[finger] = tuple(ang)
    return curls


def hold(rig, body):
    """his left hand round the upright grip, as a fist holds a bar (the user's photograph of a bar held like a yoke): the joints of
    his four knuckles in a line up it, the grip across his palm just behind them, each finger closed round it (grasp()), his thumb
    round its other side over his index finger (wrap_thumb()); the whole turned about the grip to where his wrist bends least. The
    hand's frame from his own knuckles, in the hand bone's axes (the bone's own roll is not the palm's)"""
    Rh, w = rot(rig, 'hand_l'), W(rig, 'hand_l')
    K = [Rh.transposed() @ (W(rig, f + '_01_l') - w) for f in ('index', 'middle', 'ring', 'pinky')]
    c = sum(K, Vector()) / 4                        # the knuckles' centre, from the wrist
    k = (K[0] - K[3]).normalized()                  # little finger to index
    p = c.cross(k).normalized()                     # the palm's normal: out of a left palm with the fingers ahead and the index up
    f = k.cross(p)
    pose_fingers(rig)                               # straight enough to read his palm: where its skin is below the knuckles
    Q, cls = skin_points(rig, body)
    q = Q - np.array(c[:])
    near = (np.abs(q @ np.array(k[:])) < 0.022) & (np.abs(q @ np.array(f[:]) - GRIP_AT) < 0.005) & np.array([n == 'hand_l' for n in cls])
    palm = float((q[near] @ np.array(p[:])).max())
    at, sq = GRIP_AT, SQUEEZE
    X = c + f * at + p * (palm - sq + GRIP_R)       # the grip's axis in his palm
    curls = grasp(rig, X, k)
    curls['thumb'] = wrap_thumb(rig, X, k, f, p, curls, sq - GRIP_R)
    pose_fingers(rig, curls)
    Q, cls = skin_points(rig, body)
    d = np.array([to_bar(Vector(x), X, k) for x in Q])
    inside = (d < GRIP_R - 0.002) & (np.abs((Q - np.array(X[:])) @ np.array(k[:])) < 0.045)
    print('MKPILOT his left fist: palm skin %.1f mm below his knuckles\' joints; the grip %.1f mm behind them, %.1f mm into it; fingers %s; %d skin points more than 2 mm into the grip (%s)' % (
        1000 * palm, -1000 * at, 1000 * sq, {n: tuple(int(round(a)) for a in v) for n, v in curls.items() if n != 'thumb'}, int(inside.sum()),
        sorted(set(cls[inside]))))
    HOLD['curls'] = curls
    L = Matrix((k, p, f)).transposed()
    Rh0, Rf0 = rest_axes(rig, 'hand_l'), rest_axes(rig, 'lowerarm_l')
    fore = Rh0.transposed() @ Rf0.col[1]            # the forearm's line in the hand's axes with his wrist straight
    sh = W(rig, 'upperarm_l')
    a = (W(rig, 'lowerarm_l') - sh).length
    b = (w - W(rig, 'lowerarm_l')).length
    up = Vector((0, 0, 1))
    best = None
    for deg in range(-60, 61, 2):                   # 0: his palm square to the yoke's wheel, facing inboard
        pw = Matrix.Rotation(math.radians(deg), 3, 'Z') @ Vector((0, 1, 0))
        R = Matrix((up, pw, up.cross(pw))).transposed() @ L.transposed()
        wr = GRIP_L - R @ X
        el, got = ik(sh, a, b, wr, POLE['l'])
        bend = math.degrees((R @ fore).angle(got - el))
        short = max(max(0.0, ((yoke_T(st, ai) @ wr) - sh).length - (a + b) * 0.999) for st in (0, STAB_FWD) for ai in (-1, 0, 1))
        cost = bend + 1000 * short                  # and his arm reaching the grip over the yoke's whole travel: 1 deg a mm
        if best is None or cost < best[0]:
            best = (cost, deg, R, wr, bend)
    _, deg, R, wr, bend = best
    HOLD.update(y=R @ Vector((0, 1, 0)), z=R @ Vector((0, 0, 1)), turn=deg, bend=bend)
    HOLD['palm'] = wr + HOLD['y'] * PALM
    print('MKPILOT left hand round the grip: turned %+d deg from square, his wrist bent %.1f deg' % (deg, bend))


def pose_torso(rig):
    up = Vector((math.sin(RECLINE), 0, math.cos(RECLINE)))
    face = Vector((-math.cos(RECLINE), 0, math.sin(RECLINE)))
    for n in ('pelvis', 'spine_01', 'spine_02', 'spine_03'):
        orient(rig, n, up, face)
    R = Matrix.Rotation(-PROTRACT, 3, up)                 # the left clavicle swung forward about his spine
    C = rot(rig, 'clavicle_l')
    orient(rig, 'clavicle_l', R @ C.col[1], R @ C.col[2])
    orient(rig, 'neck_01', Vector((math.sin(RECLINE) * 0.4, 0, 1)), Vector((-1, 0, 0)))
    orient(rig, 'head', Vector((math.sin(HEAD_TILT), 0, math.cos(HEAD_TILT))), Vector((-math.cos(HEAD_TILT), 0, math.sin(HEAD_TILT))))


def pose_leg(rig, side, lean=PD.LEAN):
    s = -1 if side == 'l' else 1
    hip = W(rig, 'thigh_' + side)
    a = (W(rig, 'calf_' + side) - hip).length
    b = (W(rig, 'foot_' + side) - W(rig, 'calf_' + side)).length
    flen = (W(rig, 'ball_' + side) - W(rig, 'foot_' + side)).length
    ball, sole = foot_target(side, lean)
    ankle = ball - sole * flen
    knee, ankle = ik(hip, a, b, ankle, Vector((0, s * 0.15, 1)))
    orient(rig, 'thigh_' + side, knee - hip, Vector((1, 0, -1)))
    orient(rig, 'calf_' + side, ankle - knee, Vector((1, 0, -1)))
    normal = Vector((-math.cos(math.radians(lean)), 0, -math.sin(math.radians(lean))))     # the sole's outward normal, into the pedal
    orient(rig, 'foot_' + side, sole, normal)
    orient(rig, 'ball_' + side, sole, normal)


def pose_arm(rig, side, T=Matrix.Identity(4)):
    grip, hdir, palm = arm_target(side, T)
    pole = POLE[side]
    sh = W(rig, 'upperarm_' + side)
    a = (W(rig, 'lowerarm_' + side) - sh).length
    b = (W(rig, 'hand_' + side) - W(rig, 'lowerarm_' + side)).length
    wrist = grip - hdir * PALM
    elbow, wrist = ik(sh, a, b, wrist, pole)
    Rh0, Rf0 = rest_axes(rig, 'hand_' + side), rest_axes(rig, 'lowerarm_' + side)
    zf_in_hand = Rh0.transposed() @ Rf0.col[2]
    pn = (palm - hdir * palm.dot(hdir)).normalized()
    Rh = Matrix((hdir.cross(pn), hdir, pn)).transposed()
    orient(rig, 'upperarm_' + side, elbow - sh, wrist - sh)
    orient(rig, 'lowerarm_' + side, wrist - elbow, Rh @ zf_in_hand)
    orient(rig, 'hand_' + side, hdir, pn)


def pose_fingers(rig, left=None):
    """left: each left finger's flexions, as grasp() closed them round the grip (none: straight); the right hand rests on the knobs"""
    for side in ('l', 'r'):
        for f in ('index', 'middle', 'ring', 'pinky'):
            curl = ((left or {}).get(f, (0, 0, 0)) if side == 'l' else (35, 40, 30))
            for i, ang in enumerate(curl):
                pb = rig.pose.bones['%s_%02d_%s' % (f, i + 1, side)]
                pb.rotation_mode = 'XYZ'
                pb.rotation_euler = (math.radians(ang), 0, 0)
        # the left thumb round the grip's far side, its tip over his index finger, as the photograph's (wrap_thumb())
        thumb = (left or {}).get('thumb', [[20, 35, 0], [40, 0, 0], [50, 0, 0]]) if side == 'l' else [[10, 0, 0], [15, 0, 0], [10, 0, 0]]
        for i, e in enumerate(thumb):
            pb = rig.pose.bones['thumb_%02d_%s' % (i + 1, side)]
            pb.rotation_mode = 'XYZ'
            pb.rotation_euler = tuple(math.radians(a) for a in e)
    bpy.context.view_layer.update()


def pose(rig, body):
    pose_torso(rig)
    hold(rig, body)
    for side in ('l', 'r'):
        pose_leg(rig, side)
        pose_arm(rig, side)


# ------------------------------------------------------------------------------------------------- the suit, hair and brows, painted
BONES = set()


def bone_of(obj, mesh):
    """each vertex's heaviest bone (the mesh has other groups too: helpers, joints, sides)"""
    names = [g.name for g in obj.vertex_groups]
    ok = [n in BONES for n in names]
    best = np.full(len(mesh.vertices), -1)
    w = np.zeros(len(mesh.vertices))
    for v in mesh.vertices:
        for g in v.groups:
            if g.group < len(names) and ok[g.group] and g.weight > w[v.index]:
                w[v.index] = g.weight
                best[v.index] = g.group
    return np.array([names[i] if i >= 0 else '' for i in best])


def levels(rig):
    """heights in MPFB's rest frame, from the rig at rest"""
    b = rig.data.bones
    s = 1.0 / rig.scale[0] if rig.scale[0] else 1.0
    get = lambda n, e='head': Vector(getattr(b[n], e + '_local'))
    v = lambda n, e='head': np.array(get(n, e)[:])
    return dict(neck=get('neck_01').z, neck_y=get('neck_01').y, head=get('head').z, chest=get('spine_03').z, waist=get('spine_01').z,
                crotch=get('thigh_l').z, thigh_x=abs(get('thigh_l').x), knee=get('calf_l').z,
                elbow_l=v('lowerarm_l'), wrist_l=v('hand_l'), elbow_r=v('lowerarm_r'), wrist_r=v('hand_r'))


HAND = ('hand_', 'thumb_', 'index_', 'middle_', 'ring_', 'pinky_')


def suit_at(P, cls, lv):
    """1 where the flight suit covers a point (rest frame), by the point's bone: a collar line round the neck, higher at the back,
    and cuffs 3.5 cm short of the wrists. P (N, 3); cls the bone of each"""
    s = np.ones(len(P))
    near = np.isin(cls, ('head', 'neck_01', 'spine_03', 'clavicle_l', 'clavicle_r'))      # the head's bone too: its faces reach the throat
    collar = lv['neck'] + 0.032 + 0.012 * np.clip((P[:, 1] - lv['neck_y']) / 0.05, -1, 1)
    s[near & (P[:, 2] > collar)] = 0
    for side in ('l', 'r'):
        e, w = lv['elbow_' + side], lv['wrist_' + side]
        u = w - e
        L = np.linalg.norm(u)
        t = (P - e) @ (u / L) / L
        arm = (cls == 'lowerarm_' + side) | np.array([c.startswith(HAND) and c.endswith('_' + side) for c in cls])
        s[arm & (t > 1 - 0.035 / L)] = 0
    return s


def ease(P, cls, lv, s):
    """the suit's offset eased in over 2 cm below the collar line and short of the cuffs, so its edges slope like a collar's and a
    cuff's, wherever the mesh's edges run"""
    f = s.copy()
    near = np.isin(cls, ('head', 'neck_01', 'spine_03', 'clavicle_l', 'clavicle_r'))
    collar = lv['neck'] + 0.032 + 0.012 * np.clip((P[:, 1] - lv['neck_y']) / 0.05, -1, 1)
    f[near] *= np.clip((collar[near] - P[near, 2]) / 0.02, 0.15, 1)
    for side in ('l', 'r'):
        e, w = lv['elbow_' + side], lv['wrist_' + side]
        u = w - e
        L = np.linalg.norm(u)
        t = (P - e) @ (u / L)
        arm = (cls == 'lowerarm_' + side) | np.array([c.startswith(HAND) and c.endswith('_' + side) for c in cls])
        f[arm] *= np.clip((L - 0.035 - t[arm]) / 0.02, 0.15, 1)
    return f


def taper(mesh, s, rings=(0.3, 0.6, 0.85)):
    """the suit's offset eased in over its last rows, so its edges slope like a collar's and a cuff's"""
    nb = [[] for _ in mesh.vertices]
    for e in mesh.edges:
        a, b = e.vertices
        nb[a].append(b)
        nb[b].append(a)
    f = s.copy()
    front = [i for i in range(len(s)) if s[i] == 0]
    seen = set(front)
    for k in rings:
        nxt = []
        for i in front:
            for j in nb[i]:
                if j not in seen and s[j] > 0:
                    seen.add(j)
                    f[j] = k
                    nxt.append(j)
        front = nxt
    return f


def paint(body, mesh, rest, bones, lv, eye_l, eye_r, skin_png, out_png, size=2048):
    """the body's texture: MPFB's skin, the flight suit where suit, hair and brows on the head. Details placed in the rest frame"""
    img = bpy.data.images.load(skin_png)
    tex = np.array(img.pixels[:], dtype=np.float32).reshape(img.size[1], img.size[0], 4)[::-1, :, :3] * 255.0   # rows top down
    if tex.shape[0] != size:
        raise SystemExit('mkpilot: the skin is %s, not %d' % (tex.shape, size))
    out = tex.copy()
    covered = np.zeros((size, size), bool)
    rng = np.random.default_rng(57)
    noise = rng.normal(0, 1, (size, size))
    uv = mesh.uv_layers.active.data
    mesh.calc_loop_triangles()
    for tri in mesh.loop_triangles:
        vi = list(tri.vertices)
        P3 = rest[vi]
        cls = bones[vi]
        tcls = max(set(cls), key=list(cls).count)
        S = suit_at(P3, cls, lv)
        n = np.cross(P3[1] - P3[0], P3[2] - P3[0])
        nn = n / (np.linalg.norm(n) + 1e-12)
        skin_only = S.max() < 0.5 and P3[:, 2].min() < lv['head'] - 0.02          # skin below the head: as it is
        UV = np.array([uv[l].uv[:] for l in tri.loops]) * size
        UV[:, 1] = size - UV[:, 1]
        x0, y0 = np.floor(UV.min(0)).astype(int)
        x1, y1 = np.ceil(UV.max(0)).astype(int)
        x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, size - 1), min(y1, size - 1)
        if x1 < x0 or y1 < y0:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        a, b, c = UV
        den = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(den) < 1e-9:
            continue
        l1 = ((b[1] - c[1]) * (gx - c[0]) + (c[0] - b[0]) * (gy - c[1])) / den
        l2 = ((c[1] - a[1]) * (gx - c[0]) + (a[0] - c[0]) * (gy - c[1])) / den
        l3 = 1 - l1 - l2
        inside = (l1 >= -0.02) & (l2 >= -0.02) & (l3 >= -0.02)
        if not inside.any():
            continue
        covered[y0:y1 + 1, x0:x1 + 1] |= inside
        if skin_only:
            continue
        P = l1[..., None] * P3[0] + l2[..., None] * P3[1] + l3[..., None] * P3[2]
        ys, xs = np.nonzero(inside)
        Pi = P[ys, xs]
        si = suit_at(Pi, np.full(len(Pi), tcls), lv)
        X, Y, Z = Pi[:, 0], Pi[:, 1], Pi[:, 2]
        col = out[y0 + ys, x0 + xs]
        nz = noise[y0 + ys, x0 + xs]
        # the suit
        on = si >= 0.5
        if on.any():
            c = np.tile(SAGE, (len(X), 1)) * (1 + 0.035 * nz[:, None])
            front = nn[1] < -0.35
            if front:
                zip_ = (np.abs(X) < 0.0065) & (Z < lv['neck'] + 0.01) & (Z > lv['crotch'] - 0.03)
                c[zip_] = np.array((58, 60, 52)) * (1 + 0.08 * ((Z[zip_] * 900).astype(int) % 2))[:, None]
                ctop = lv['chest'] + 0.07
                for sx in (-1, 1):
                    px = sx * X
                    pocket = (px > 0.055) & (px < 0.150) & (Z < ctop) & (Z > ctop - 0.12)
                    edge = pocket & ((px < 0.059) | (px > 0.146) | (Z > ctop - 0.004) | (Z < ctop - 0.116))
                    c[pocket] *= 0.93
                    c[edge] *= 0.72
                    zp = pocket & (np.abs(Z - (ctop - 0.012)) < 0.0025)
                    c[zp] = (52, 54, 48)
                patch = (X > 0.06) & (X < 0.15) & (Z > ctop + 0.012) & (Z < ctop + 0.052)          # his left breast: a blank leather patch
                c[patch] = np.array((70, 52, 38)) * (1 + 0.05 * nz[patch, None])
                for sx in (-1, 1):
                    px = sx * X
                    tp = (px > lv['thigh_x'] - 0.005) & (Z > lv['knee'] + 0.10) & (Z < lv['knee'] + 0.27) & (Z < lv['crotch'] - 0.05)
                    tedge = tp & ((Z > lv['knee'] + 0.264) | (Z < lv['knee'] + 0.106))
                    c[tp] *= 0.92
                    c[tedge] *= 0.72
            band = np.abs(Z - lv['waist']) < 0.014
            c[band] *= 0.88
            col[on] = c[on]
        # hair and brows
        if (~on).any() and P3[:, 2].max() > lv['head'] - 0.04:
            ez = (eye_l[2] + eye_r[2]) / 2
            fy = min(eye_l[1], eye_r[1])                 # the face's front at the eyes
            hair = (Z > ez + 0.045) & ~((Y < fy + 0.035) & (Z < ez + 0.075))
            hair |= (Y > fy + 0.06) & (Z > ez - 0.035)     # sides above the ears and the back
            hair |= (Y > fy + 0.10) & (Z > lv['head'] + 0.01)
            m = hair & ~on
            if m.any():
                h = np.array((52, 40, 32)) * (1 + 0.12 * nz[m, None])
                col[m] = 0.15 * col[m] + 0.85 * h
            for e in (eye_l, eye_r):
                dx = (X - e[0]) / 0.024
                dz = (Z - (e[2] + 0.017 + 0.006 * (1 - dx * dx))) / 0.0045
                brow = (dx * dx + dz * dz < 1) & (Y < fy + 0.02) & ~on
                col[brow] = 0.25 * col[brow] + 0.75 * np.array((48, 36, 28))
        out[y0 + ys, x0 + xs] = col
    # the gutters round each UV island filled from its edge outward, so filtering at the seams finds suit or skin, not the skin picture's border
    filled = covered.copy()
    for _ in range(10):
        acc = np.zeros_like(out)
        cnt = np.zeros((size, size))
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            m = np.roll(filled, (dy, dx), axis=(0, 1))
            acc += np.roll(out, (dy, dx), axis=(0, 1)) * m[..., None]
            cnt += m
        new = ~filled & (cnt > 0)
        out[new] = acc[new] / cnt[new][:, None]
        filled |= new
    im = bpy.data.images.new('pilot-body', size, size)
    rgba = np.concatenate([np.clip(out, 0, 255) / 255.0, np.ones((size, size, 1))], axis=2)[::-1]
    im.pixels.foreach_set(rgba.astype(np.float32).ravel())
    im.filepath_raw = out_png
    im.file_format = 'PNG'
    im.save()
    return im


# ------------------------------------------------------------------------------------------------- the headset
def tube(name, pts, r, n=10, col=(0.05, 0.05, 0.05)):
    bm = bmesh.new()
    rings = []
    a, t0 = None, None
    for i, p in enumerate(pts):
        t = (pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]).normalized()
        if a is None:
            a = t.orthogonal().normalized()
        else:                                    # each ring's frame carried on from the last one's: a fresh one each ring twists the tube thin where it bends
            a = t0.rotation_difference(t) @ a
            a = (a - t * a.dot(t)).normalized()
        b = t.cross(a)
        t0 = t
        rings.append([bm.verts.new(p + r * (math.cos(2 * math.pi * k / n) * a + math.sin(2 * math.pi * k / n) * b)) for k in range(n)])
    for r0, r1 in zip(rings, rings[1:]):
        for k in range(n):
            bm.faces.new((r0[k], r0[(k + 1) % n], r1[(k + 1) % n], r1[k]))
    bm.faces.new(rings[0][::-1])
    bm.faces.new(rings[-1])
    for f in bm.faces:                           # smooth-shaded: FlightGear keeps only edges sharper than the crease angle
        f.smooth = True
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    o['colour'] = col
    return o


def cup(name, c, axis, r, depth, col):
    a = axis.normalized()
    return tube(name, [c - a * depth / 2, c + a * depth / 2], r, 24, col)


def head_geometry(rig, body, eyes):
    """his posed head as the headset and glasses fit it: a frame at the midpoint of his eyes (left, up, fwd: the head bone's axes),
    the eyes' centres, and a BVH of his body's surface (the suit on) to measure against"""
    from mathutils.bvhtree import BVHTree
    Rh = rot(rig, 'head')
    up, fwd, left = Rh.col[1].normalized(), Rh.col[2].normalized(), Rh.col[0].normalized()
    E = [eyes.matrix_world @ v.co for v in eyes.data.vertices]
    c = sum(E, Vector()) / len(E)
    EL = [e for e in E if (e - c).dot(left) > 0]
    ER = [e for e in E if (e - c).dot(left) <= 0]
    eL, eR = sum(EL, Vector()) / len(EL), sum(ER, Vector()) / len(ER)
    dg = bpy.context.evaluated_depsgraph_get()
    me = body.evaluated_get(dg).data
    P = [body.matrix_world @ v.co for v in me.vertices]
    bvh = BVHTree.FromPolygons(P, [tuple(f.vertices) for f in me.polygons])
    return SimpleNamespace(mid=(eL + eR) / 2, left=left, up=up, fwd=fwd, eL=eL, eR=eR, P=P, bvh=bvh)


# The ear seals: a circumaural headset's cushions enclose the ears and press on the head round them, the temples of his glasses
# running under them. His ear (MPFB's, 48 mm tall, standing 17 mm off his head) is found on a height map of the side of his head,
# by what stands more than 5 mm out of the local median; the seal is centred on it. Its face is the plane fitted to his head
# in a ring 28 to 40 mm from that centre (his head is within 2 mm of it, by 45-deg sector), and its inner edge follows his head
# exactly, pressed 1 mm in, as a cushion gives. Sizes mine: an 80 mm cushion 20 mm thick, an 86 mm shell 30 mm deep
SEAL_R, SEAL_T = 0.040, 0.020
CUP_R, CUP_T = 0.043, 0.030


def ear_pad(G, s):
    """where the ear seal on side s (1 his left) sits: its centre on his head's plane round the ear, that plane's normal, and the ear"""
    fs, bs = np.arange(-0.15, -0.025, 0.003), np.arange(-0.085, 0.055, 0.003)
    H = np.full((len(bs), len(fs)), np.nan)
    for i, b in enumerate(bs):
        for j, f in enumerate(fs):
            loc, nrm, idx, dist = G.bvh.ray_cast(G.mid + G.left * s * 0.2 + G.fwd * float(f) + G.up * float(b), -G.left * s, 0.2)
            if loc is not None:
                H[i, j] = 0.2 - dist
    FF, BB = np.meshgrid(fs, bs)
    m = np.isfinite(H)
    med = np.full(H.shape, np.nan)
    for i in range(H.shape[0]):
        for j in range(H.shape[1]):
            w = H[max(0, i - 5):i + 6, max(0, j - 5):j + 6]
            if np.isfinite(w).sum() > 20:
                med[i, j] = np.nanmedian(w)
    with np.errstate(invalid='ignore'):
        ear = m & (H - med > 0.005) & (FF > -0.13) & (FF < -0.05) & (BB > -0.07) & (BB < 0.03)
    fe, be = (FF[ear].min() + FF[ear].max()) / 2, (BB[ear].min() + BB[ear].max()) / 2
    r = np.hypot(FF - fe, BB - be)
    ring = m & (r > 0.028) & (r < SEAL_R) & ~ear
    X = np.column_stack([np.ones(ring.sum()), FF[ring] - fe, BB[ring] - be])
    (h0, gf, gb), *_ = np.linalg.lstsq(X, H[ring], rcond=None)
    n = (G.left * s - G.fwd * float(gf) - G.up * float(gb)).normalized()
    p0 = G.mid + G.left * s * float(h0) + G.fwd * float(fe) + G.up * float(be)
    pts = [G.mid + G.left * s * float(H[i, j]) + G.fwd * float(FF[i, j]) + G.up * float(BB[i, j]) for i, j in zip(*np.nonzero(ring))]
    off = np.array([(p - p0).dot(n) for p in pts])
    tall = max((G.mid + G.left * s * float(H[i, j]) + G.fwd * float(FF[i, j]) + G.up * float(BB[i, j]) - p0).dot(n) for i, j in zip(*np.nonzero(ear)))
    print('MKPILOT ear %s: %.0f x %.0f mm, centre %.0f mm behind and %.0f below his eyes, standing %.1f mm off the plane; the seal faces %.1f deg down, %.1f back; his head off it %.1f..%.1f mm (5th..95th pct)'
          % ('lr'[s < 0], (FF[ear].max() - FF[ear].min()) * 1e3, (BB[ear].max() - BB[ear].min()) * 1e3, -fe * 1e3, -be * 1e3, tall * 1e3,
             math.degrees(math.atan(gb)), math.degrees(math.atan(gf)), *(np.percentile(off, (5, 95)) * 1e3)))
    return dict(p0=p0, n=n, fe=float(fe), be=float(be), top=float(BB[ear].max()))


def seal(name, pad, G, col, N=32):
    """the ear seal: its inner edge on his head, pressed 1 mm in, its side bulging a little, its outer face flat"""
    n, p0 = pad['n'], pad['p0']
    a = n.orthogonal().normalized()
    b = n.cross(a)
    bm = bmesh.new()
    rings = [[], ]
    for k in range(N):
        dr = a * math.cos(2 * math.pi * k / N) + b * math.sin(2 * math.pi * k / N)
        q = p0 + dr * SEAL_R
        loc, nrm, idx, dist = G.bvh.ray_cast(q + n * 0.05, -n, 0.1)
        u = min(max(0.05 - dist, -0.008), 0.006) if loc is not None else 0.0
        rings[0].append(bm.verts.new(q + n * (u - 0.001)))
    for u, rad in ((0.008, SEAL_R + 0.0015), (0.014, SEAL_R + 0.001), (0.018, SEAL_R - 0.001), (SEAL_T, SEAL_R - 0.004)):
        rings.append([bm.verts.new(p0 + n * u + (a * math.cos(2 * math.pi * k / N) + b * math.sin(2 * math.pi * k / N)) * rad) for k in range(N)])
    for r0, r1 in zip(rings, rings[1:]):
        for k in range(N):
            bm.faces.new((r0[k], r0[(k + 1) % N], r1[(k + 1) % N], r1[k]))
    bm.faces.new(rings[0][::-1])
    bm.faces.new(rings[-1])
    for f in bm.faces:                           # smooth-shaded: FlightGear keeps only edges sharper than the crease angle
        f.smooth = True
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    o['colour'] = col
    return o


def headset(G):
    """two cups over his ears, a band over his head, a boom to the left of his mouth; and where the seals sit, for his glasses"""
    pads = {s: ear_pad(G, s) for s in (1, -1)}
    objs = []
    shell = {}
    for s, tag in ((1, 'l'), (-1, 'r')):
        p = pads[s]
        objs.append(seal('hs-cup-%s-seal' % tag, p, G, (0.03, 0.03, 0.03)))
        shell[s] = p['p0'] + p['n'] * (SEAL_T + CUP_T / 2)
        objs.append(cup('hs-cup-' + tag, shell[s], p['n'], CUP_R, CUP_T, (0.13, 0.13, 0.14)))
    # the band: from the cups' centres over the top of his head, 14 mm clear of it
    M = (shell[1] + shell[-1]) / 2
    half = (shell[1] - shell[-1]) / 2
    upv = (G.up - half.normalized() * G.up.dot(half.normalized())).normalized()
    crown = max((p - M).dot(upv) for p in G.P if (p - M).length < 0.25)
    arc = [M + half * math.cos(math.pi * k / 24) + upv * (math.sin(math.pi * k / 24) * (crown + 0.014)) for k in range(25)]
    clear = min(G.bvh.find_nearest(q)[3] for q in arc[3:22])
    objs.append(tube('hs-band', arc, 0.0085, 8, (0.08, 0.08, 0.09)))
    # the boom, from the left cup's front to his mouth's left corner
    loc, nrm, idx, dist = G.bvh.ray_cast(G.mid - G.up * 0.068 + G.fwd * 0.2, -G.fwd, 0.3)
    F = loc if loc is not None else G.mid - G.up * 0.068 + G.fwd * 0.03
    start = shell[1] + pads[1]['n'] * 0.008 + G.fwd * 0.03 - upv * 0.02
    end = F + G.fwd * 0.022 + G.left * 0.022 - upv * 0.005
    boom = [start.lerp(end, t) + G.fwd * 0.035 * math.sin(math.pi * t) + G.left * 0.02 * math.sin(math.pi * t) for t in np.linspace(0, 1, 12)]
    objs.append(tube('hs-boom', boom, 0.0025, 8, (0.04, 0.04, 0.04)))
    objs.append(tube('hs-mic', [end - G.fwd * 0.01, end - G.left * 0.012], 0.0075, 12, (0.02, 0.02, 0.02)))
    print('MKPILOT headset: the band clears his head by %.0f mm at least (its centreline)' % (clear * 1e3))
    return objs, pads


# ------------------------------------------------------------------------------------------------- his parts, for FlightGear
FINGERS = ('thumb', 'index', 'middle', 'ring', 'pinky')
SEGMENTS = {'torso': ('Root', 'pelvis', 'spine_01', 'spine_02', 'spine_03', 'clavicle_l', 'clavicle_r'), 'head': ('neck_01', 'head')}
for _s in ('l', 'r'):
    SEGMENTS.update({'arm-u-' + _s: ('upperarm_' + _s,), 'arm-f-' + _s: ('lowerarm_' + _s,),
                     'hand-' + _s: ('hand_' + _s,) + tuple('%s_%02d_%s' % (f, i, _s) for f in FINGERS for i in (1, 2, 3)),
                     'leg-t-' + _s: ('thigh_' + _s,), 'leg-c-' + _s: ('calf_' + _s,), 'foot-' + _s: ('foot_' + _s, 'ball_' + _s)})
SEG_OF = {b: k for k, v in SEGMENTS.items() for b in v}


def split(obj, part, bones):
    """the object cut into one per segment, by each face's commonest bone: named SEGMENT.PART"""
    out = []
    me = obj.data
    face_seg = []
    for f in me.polygons:
        cls = [SEG_OF.get(bones[i], 'torso') for i in f.vertices]
        face_seg.append(max(set(cls), key=cls.count))
    vert_segs = [set() for _ in me.vertices]
    for f, sg in zip(me.polygons, face_seg):
        for i in f.vertices:
            vert_segs[i].add(sg)
    normals = [v.normal.copy() for v in me.vertices]
    for seg in sorted(set(face_seg)):
        # its own faces, and a skirt: the faces just across its boundary, 3 mm inside, to fill the seam when its joint turns
        skirt = {f.index for f, sg in zip(me.polygons, face_seg) if sg != seg and any(seg in vert_segs[i] for i in f.vertices)}
        o = obj.copy()
        o.data = me.copy()
        o.name = '%s.%s' % (seg, part)
        bpy.context.scene.collection.objects.link(o)
        bm = bmesh.new()
        bm.from_mesh(o.data)
        bm.faces.ensure_lookup_table()
        bm.verts.ensure_lookup_table()
        moved = {i for f in skirt for i in me.polygons[f].vertices if seg not in vert_segs[i]}
        for i in moved:
            bm.verts[i].co -= normals[i] * 0.003
        bmesh.ops.delete(bm, geom=[f for f in bm.faces if face_seg[f.index] != seg and f.index not in skirt], context='FACES')
        bm.to_mesh(o.data)
        bm.free()
        out.append(o)
    bpy.data.objects.remove(obj)
    return out


def interp(x, table):
    xs, ys = zip(*table)
    return float(np.interp(x, xs, ys))


YOKE_SLIDE = ((-1, 0.07), (0, 0.0), (1, -0.07))          # yoke.xml: along x with fdm/jsbsim/x57/stab-norm
YOKE_TURN = ((-1, 50.0), (0, 0.0), (1, -50.0))           # and about x with surface-positions/left-aileron-pos-norm
STAB_FWD = 5.0 / 15.0                                       # stab-norm at full forward: the stabilator runs -15 to +5 deg, over 15 (x57-mod*-controls.xml)
LEVER_DEG = (0.0, 12.5, 25.0, 37.5, 50.0)                # the torque knobs' travel (pedestal.xml), right-handed about -y


def yoke_T(stab, ail):
    """FlightGear's order: the slide outside, the turn inside"""
    return (Matrix.Translation((interp(stab, YOKE_SLIDE), 0, 0)) @ Matrix.Translation(YOKE_ORIGIN)
            @ Matrix.Rotation(math.radians(interp(ail, YOKE_TURN)), 4, 'X') @ Matrix.Translation(-YOKE_ORIGIN))


def lever_T(deg):
    return Matrix.Translation(LEVER_PIVOT) @ Matrix.Rotation(math.radians(deg), 4, Vector((0, -1, 0))) @ Matrix.Translation(-LEVER_PIVOT)


def rot(rig, name):
    return (rig.matrix_world.to_3x3() @ rig.pose.bones[name].matrix.to_3x3()).normalized()


def frame(d0):
    """two axes square to a bone's rest direction: its swing is a turn about the first, then the second (inside it)"""
    e1 = d0.normalized()
    e2 = e1.cross(Vector((0, 0, 1)))
    if e2.length < 0.2:
        e2 = e1.cross(Vector((1, 0, 0)))
    e2.normalize()
    return e1, e2, e1.cross(e2)


def two_angles(d0, d):
    """the turns (deg) about e2, then e3, that take d0 to d: R(e2, a) R(e3, b) d0 = d"""
    e1, e2, e3 = frame(d0)
    d = d.normalized()
    b = math.asin(max(-1.0, min(1.0, d.dot(e2))))
    a = math.atan2(-d.dot(e3), d.dot(e1))
    return math.degrees(a), math.degrees(b)


def turn(d0, a, b):
    e1, e2, e3 = frame(d0)
    return Matrix.Rotation(math.radians(a), 3, e2) @ Matrix.Rotation(math.radians(b), 3, e3)


TERMS = ((0, 0), (1, 0), (0, 1), (2, 0), (1, 1), (0, 2), (2, 1), (1, 2), (3, 0), (0, 3), (2, 2), (3, 1), (1, 3))


def polyfit(X, Y, V):
    A = np.array([[x ** i * y ** j for i, j in TERMS] for x, y in zip(X, Y)])
    c, *_ = np.linalg.lstsq(A, np.array(V), rcond=None)
    return [round(float(v), 6) for v in c]


def polyval(c, x, y):
    return sum(k * x ** i * y ** j for k, (i, j) in zip(c, TERMS))


def solve(rig):
    """the arms' and legs' joint swings over their controls' travel, as FlightGear nests them: the upper part about its root joint,
    the lower about its own (in the rest pose's frame), each a turn about two fixed axes square to the bone. The left arm's angles
    are polynomials in the yoke's two inputs; the others tables of one. The hands and feet ride their controls exactly"""
    d0 = {n: rot(rig, n).col[1].copy() for n in ('upperarm_l', 'lowerarm_l', 'upperarm_r', 'lowerarm_r', 'thigh_l', 'calf_l', 'thigh_r', 'calf_r')}

    def angles(up, lo):
        u, l = rot(rig, up).col[1].copy(), rot(rig, lo).col[1].copy()
        au, bu = two_angles(d0[up], u)
        al, bl = two_angles(d0[lo], turn(d0[up], au, bu).transposed() @ l)
        return au, bu, al, bl

    def axes(n):
        e1, e2, e3 = frame(d0[n])
        return [tuple(round(c, 6) for c in e2), tuple(round(c, 6) for c in e3)]

    out = {}
    # left arm: a grid over the yoke's travel
    g = (-1, -0.5, 0, 0.5, 1)
    S, A, R = [], [], []
    for st in g:
        for ai in g:
            pose_arm(rig, 'l', yoke_T(st, ai))
            S.append(st)
            A.append(ai)
            R.append(angles('upperarm_l', 'lowerarm_l'))
    R = np.array(R)
    polys = [polyfit(S, A, R[:, k]) for k in range(4)]
    out['arm-l'] = dict(inputs=['fdm/jsbsim/x57/stab-norm', 'surface-positions/left-aileron-pos-norm'], axes=axes('upperarm_l'), poly=polys[:2])
    out['fore-l'] = dict(inputs=out['arm-l']['inputs'], axes=axes('lowerarm_l'), poly=polys[2:])
    # how well: the wrist the polynomials put, against the solved one, over a finer grid
    pose_arm(rig, 'l')
    sh, el0, wr0 = W(rig, 'upperarm_l'), W(rig, 'lowerarm_l'), W(rig, 'hand_l')
    worst = 0.0
    for st in np.linspace(-1, 1, 9):
        for ai in np.linspace(-1, 1, 9):
            pose_arm(rig, 'l', yoke_T(st, ai))
            want = W(rig, 'hand_l')
            v = [polyval(c, st, ai) for c in polys]
            Ru, Rl = turn(d0['upperarm_l'], v[0], v[1]), turn(d0['lowerarm_l'], v[2], v[3])
            got = sh + Ru @ ((el0 - sh) + Rl @ (wr0 - el0))
            worst = max(worst, (got - want).length)
    out['left_wrist_gap_cm'] = round(worst * 100, 2)
    short = 0.0                                     # and how far short of the grip his straightened arm would fall, over the travel
    for st in (-1, -0.5, 0, STAB_FWD):             # the stabilator's travel: full aft to full forward
        for ai in (-1, 0, 1):
            pose_arm(rig, 'l', yoke_T(st, ai))
            grip, hdir, palm = arm_target('l', yoke_T(st, ai))
            short = max(short, (W(rig, 'hand_l') - (grip - hdir * PALM)).length)
            print('MKPILOT reach stab %+.1f ail %+.0f: short %.1f cm, shoulder to wrist target %.3f m' % (st, ai, (W(rig, 'hand_l') - (grip - hdir * PALM)).length * 100, ((grip - hdir * PALM) - W(rig, 'upperarm_l')).length))
    out['left_reach_short_cm'] = round(short * 100, 2)
    pose_arm(rig, 'l')

    def table(up, lo, values, pose_fn, prop):
        rows = []
        for x in values:
            pose_fn(x)
            rows.append((x,) + angles(up, lo))
        rows = np.array(rows)
        mk = lambda k: [(round(float(r[0]), 4), round(float(r[k]), 4)) for r in rows]
        return (dict(input=prop, axes=axes(up), tables=[mk(1), mk(2)]), dict(input=prop, axes=axes(lo), tables=[mk(3), mk(4)]))

    out['arm-r'], out['fore-r'] = table('upperarm_r', 'lowerarm_r', LEVER_DEG, lambda d: pose_arm(rig, 'r', lever_T(d)), '/sim/model/x57/pilot/lever-deg')
    pose_arm(rig, 'r')
    for side in ('l', 'r'):
        out['leg-' + side], out['calf-' + side] = table('thigh_' + side, 'calf_' + side, (-1, -0.5, 0, 0.5, 1),
                                                        lambda x, side=side: pose_leg(rig, side, PD.LEAN + PD.swing('p' + side, x)), 'controls/flight/rudder')
        pose_leg(rig, side)
    return out


def export(objs, extra):
    """the parts, their textures beside them, and pilot.json"""
    bpy.ops.object.select_all(action='DESELECT')
    for o in objs:
        o.select_set(True)
    path = os.path.join(OUT, 'pilot.ac')
    bpy.ops.export_scene.export_ac3d(filepath=path, use_selection=True, crease_angle=math.radians(50.0))
    t = open(path).read()
    t = re.sub(r'^texture "(?:[^"]*/)?([^"/]+)"', r'texture "\1"', t, flags=re.M)
    open(path, 'w').write(t)
    json.dump(extra, open(os.path.join(OUT, 'pilot.json'), 'w'), indent=1)


# ------------------------------------------------------------------------------------------------- his sunglasses
# Aviators of the US military's pattern, as American Optical's Original Pilot (its product photographs, front, angle and profile): a
# 23K-gold wire frame, a brow bar over the lenses' inner thirds and an arched bridge 10 mm under the lenses' tops, nose pads on arms,
# straight bayonet temples 140 mm long with clear sleeves, green lenses. Sizes 52, 55 and 57 mm lens, 20 mm bridge: the smallest
# that clears his temples, and his small face (temples 59 mm either side, pupils 58 mm apart) overhangs even that, so the frame is
# scaled to put its endpieces 2 mm outside his temples: about 48 mm lens, the temples scaled with it. Under a headset the bayonets run
# straight back over the ears' roots, under the ear seals. The lens's outline traced from the front photograph (data/pilot/aviator-lens.json: across from its inner edge,
# down from its top, by its width), smoothed. Modelled here; no maker's name or mark on them
LENS_SIZES = (0.052, 0.055, 0.057)
BRIDGE = 0.020
TEMPLE = 0.140


def lens_outline(n=48, k=7):
    """the traced outline, smoothed by a Fourier series about its centre and resampled"""
    pts = np.array(json.load(open(os.path.join(OUT, 'aviator-lens.json'))))
    c = pts.mean(0)
    ang = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    rad = np.hypot(pts[:, 1] - c[1], pts[:, 0] - c[0])
    order = np.argsort(ang)
    ang, rad = ang[order], rad[order]
    A = np.column_stack([np.ones_like(ang)] + [f(m * ang) for m in range(1, k + 1) for f in (np.cos, np.sin)])
    coef, *_ = np.linalg.lstsq(A, rad, rcond=None)
    t = np.linspace(-math.pi, math.pi, n, endpoint=False)
    T = np.column_stack([np.ones_like(t)] + [f(m * t) for m in range(1, k + 1) for f in (np.cos, np.sin)])
    r = T @ coef
    out = np.column_stack([c[0] + r * np.cos(t), c[1] + r * np.sin(t)])
    out[:, 0] -= out[:, 0].min()
    out[:, 1] -= out[:, 1].max()
    return out / (out[:, 0].max())                     # across 0..1 from the inner edge, down 0..-h from the top


def glasses(G, pads):
    """the sunglasses on his face, their temples under the headset's ear seals: from his posed head and eyes"""
    up, fwd, left, mid, eL, eR, P, bvh = G.up, G.fwd, G.left, G.mid, G.eL, G.eR, G.P, G.bvh
    rel = lambda p: ((p - mid).dot(left), (p - mid).dot(up), (p - mid).dot(fwd))
    R3 = [rel(p) for p in P]
    temple = max(abs(a) for a, b, f in R3 if abs(b) < 0.01 and -0.02 < f < -0.01)        # his temples, 10 to 20 mm behind his eyes (not his ears)
    w = next((x for x in LENS_SIZES if x + BRIDGE / 2 + 0.004 >= temple + 0.002), LENS_SIZES[-1])         # the smallest whose endpieces clear them
    scale = min(1.0, (temple + 0.002) / (w + BRIDGE / 2 + 0.004))       # and if even that overhangs his small face, the whole frame scaled to it
    w, bridge = w * scale, BRIDGE * scale
    length = TEMPLE * scale
    shape = lens_outline()
    h = -shape[:, 1].min() * w
    top = mid + up * (0.4 * h) + fwd * 0.024              # the lenses' top line, 12 mm before the corneas, the pupils 40 % down
    objs = []

    def at(s, xr, zr, dome=0.0):
        return top + left * s * (bridge / 2 + xr * w) + up * (zr * w) + fwd * dome

    def mesh(name, verts, faces, col):
        m = bpy.data.meshes.new(name)
        m.from_pydata([tuple(v) for v in verts], [], faces)
        o = bpy.data.objects.new(name, m)
        bpy.context.scene.collection.objects.link(o)
        o['colour'] = col
        objs.append(o)
        return o

    gold, green, clear = (0.83, 0.66, 0.28), (0.05, 0.09, 0.05), (0.9, 0.9, 0.88)
    cx, cz = shape[:, 0].mean(), shape[:, 1].mean()
    for s, tag in ((1, 'l'), (-1, 'r')):
        # the lens, domed 2 mm (a base curve), its rim a gold wire round it
        sag = lambda xr, zr: 0.002 * (1 - ((xr - cx) ** 2 + (zr - cz) ** 2) / 0.36)
        ring = [at(s, xr, zr, sag(xr, zr)) for xr, zr in shape]
        cen = at(s, cx, cz, sag(cx, cz))
        n = len(ring)
        lo = mesh('gl-lens-' + tag, ring + [cen], [(i, (i + 1) % n, n) for i in range(n)], green)
        lo.data.update()
        if lo.data.polygons[0].normal.dot(fwd) < 0:              # its faces toward the world: FlightGear draws one side
            bm = bmesh.new()
            bm.from_mesh(lo.data)
            bmesh.ops.reverse_faces(bm, faces=bm.faces[:])
            bm.to_mesh(lo.data)
            bm.free()
        o = tube('gl-rim-' + tag, ring + [ring[0]], 0.0007, 6, gold)
        objs.append(o)
        # the endpiece at its top outer corner; the temple from there straight back over the root of his ear, under the ear seal: aimed 1.5 mm
        # off his head over the ear's top, then turned out until, where it shows, it clears his head by 1.5 mm
        j = int(np.argmax(shape[:, 0] + 0.6 * shape[:, 1]))
        hinge = at(s, shape[j, 0], shape[j, 1]) + left * s * 0.004 - fwd * 0.003
        objs.append(tube('gl-end-' + tag, [ring[j], hinge], 0.0012, 6, gold))
        pad = pads[s]

        def hidden(q):                                        # inside the seal: between his head and its outer face, within its rim
            v = q - pad['p0']
            u = v.dot(pad['n'])
            return u < SEAL_T and (v - pad['n'] * u).length < SEAL_R - 0.002

        o = mid + left * s * 0.2 + fwd * pad['fe'] + up * (pad['top'] + 0.003)
        loc, nrm, idx, dist = bvh.ray_cast(o, -left * s, 0.2)
        d = (loc + left * s * 0.0015 - hinge).normalized()
        for _ in range(60):
            worst, gap = 1.0, 0.0
            for i in range(1, 41):
                q = hinge + d * (length * i / 40)
                if hidden(q):
                    continue
                loc, nrm, idx, dist = bvh.find_nearest(q)
                if loc is not None:
                    sd = dist if (q - loc).dot(nrm) > 0 else -dist
                    worst, gap = min(worst, sd), max(gap, sd)
            if worst >= 0.0015:
                break
            d = (d + left * s * 0.004).normalized()
        end = hinge + d * length
        into = next((i for i in range(41) if hidden(hinge + d * (length * i / 40))), None)
        print('MKPILOT temple %s: %.0f mm, under the seal from %.0f mm, its end %s; where it shows, %.1f to %.1f mm off his head'
              % (tag, length * 1e3, -1 if into is None else length * into / 40 * 1e3, 'under it' if hidden(end) else 'OUT BEHIND IT', worst * 1e3, gap * 1e3))
        objs.append(tube('gl-temple-' + tag, [hinge, end], 0.0009, 6, gold))
        objs.append(tube('gl-sleeve-' + tag, [hinge + d * (length * 0.088 / TEMPLE), end], 0.0021, 8, clear))
        # the nose pad on its arm, from the rim's inner side
        k = int(np.argmin(np.abs(shape[:, 1] + 0.5 * h / w) + 2 * shape[:, 0]))
        root = ring[k]
        pad = root + left * s * 0.002 - fwd * 0.010 - up * 0.003          # behind the lens's inner edge, on the side of his nose
        objs.append(tube('gl-padarm-' + tag, [root, root - fwd * 0.005, pad], 0.0005, 5, gold))
        objs.append(tube('gl-pad-' + tag, [pad - up * 0.005, pad + up * 0.005], 0.0024, 8, clear))
    # the brow bar over the lenses' inner thirds, at their tops; the arched bridge 10 mm under them
    bar = [at(s, 0.32, 0.0) for s in (1, -1)]
    objs.append(tube('gl-bar', [bar[0] - up * 0.0015, bar[0] + up * 0.001] + [bar[0].lerp(bar[1], t) + up * 0.001 for t in np.linspace(0.05, 0.95, 9)]
                     + [bar[1] + up * 0.001, bar[1] - up * 0.0015], 0.0008, 6, gold))
    ends = [at(s, 0.0, -0.012 / w) for s in (1, -1)]
    objs.append(tube('gl-bridge', [ends[0].lerp(ends[1], t) + up * (0.0025 * math.sin(math.pi * t)) for t in np.linspace(0, 1, 11)], 0.0009, 6, gold))
    print('MKPILOT glasses: %.1f mm lens, %.1f mm bridge (his temples %.0f mm either side), pupils %.0f mm apart' % (w * 1000, bridge * 1000, temple * 1000, (eL - eR).length * 1000))
    return objs


# ------------------------------------------------------------------------------------------------- his seatbelt
# The P2006T's seats have "three fitting points safety belts; belt adjustment is via the sliding buckle located on the belt metal
# hook" [P2006T flight manual, 7-26]: a lap belt from the seat's outboard side across his hips to a buckle at his inboard hip, the
# buckle on a short stalk to the seat's inboard side, and a shoulder strap from behind his outboard shoulder, over it and across his
# chest to the buckle. Where the upper fitting is the manual doesn't say: here on the left wall just aft of the main door's frame (its
# lining ends at x -1.944), above and behind his shoulder. The webbing 48 mm wide and dark grey, the fittings and buckle steel: mine.
# The belt lies on his suit, draped over it from fitting to buckle
BELT_W, BELT_T = 0.048, 0.0025
UPPER_FIT = Vector((-1.90, -0.485, 0.16))           # on the wall aft of the door, 16 cm over his shoulder: higher, the strap would pass under his ear cup
LAP_FIT = {'out': Vector((-2.08, -0.478, -0.60)), 'in': Vector((-2.08, -0.105, -0.60))}
WEBBING, STEEL = (0.20, 0.20, 0.21), (0.70, 0.71, 0.74)


def _ribbon(name, pts, nrms, col, w=BELT_W, t=BELT_T):
    """a strap along pts, its face toward nrms: across its width five vertices on top and five under (so it can follow him across it),
    its ends closed. Each ring: the top's five from one edge to the other, then the underside's back"""
    bm = bmesh.new()
    rings = []
    for i, (p, n) in enumerate(zip(pts, nrms)):
        tg = (pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]).normalized()
        n = (n - tg * n.dot(tg)).normalized()
        sd = tg.cross(n).normalized()
        across = [1, 0.5, 0, -0.5, -1]
        rings.append([bm.verts.new(p + sd * a * w / 2 + n * t / 2) for a in across] + [bm.verts.new(p + sd * a * w / 2 - n * t / 2) for a in across[::-1]])
    m = len(rings[0])
    for r0, r1 in zip(rings, rings[1:]):
        for k in range(m):
            bm.faces.new((r0[k], r0[(k + 1) % m], r1[(k + 1) % m], r1[k]))
    bm.faces.new(rings[0][::-1])
    bm.faces.new(rings[-1])
    for f in bm.faces:                           # smooth-shaded: FlightGear keeps only edges sharper than the crease angle
        f.smooth = True
    bm.normal_update()
    c = sum((v.co for v in bm.verts), Vector()) / len(bm.verts)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    o['colour'] = col
    return o


def _plate(name, c, n, along, size, col):
    """a small box at c: its face toward n, its length along along"""
    n = n.normalized()
    a = (along - n * along.dot(n)).normalized()
    b = n.cross(a)
    bm = bmesh.new()
    vs = [bm.verts.new(c + a * x * size[0] / 2 + b * y * size[1] / 2 + n * z * size[2] / 2) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    for q in ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)):
        bm.faces.new([vs[i] for i in q])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    o['colour'] = col
    return o


def _box(name, c, n, along, size, bevel=0.0, col=(0.03, 0.03, 0.03)):
    """a box at c, its face toward n, its length along along, its edges rounded"""
    n = n.normalized()
    a = (along - n * along.dot(n)).normalized()
    b = n.cross(a)
    bm = bmesh.new()
    vs = [bm.verts.new(c + a * x * size[0] / 2 + b * y * size[1] / 2 + n * z * size[2] / 2) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    for q in ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)):
        bm.faces.new([vs[i] for i in q])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    if bevel:
        bmesh.ops.bevel(bm, geom=bm.edges[:] + bm.verts[:], offset=bevel, segments=2, affect='EDGES')
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    o['colour'] = col
    return o


def _latch(name, c, n, along):
    """a seatbelt's latch plate, 3 mm steel: at its outer end a frame 60 mm across round a 51 x 9 mm slot the 48 mm webbing loops
    through (its centre 18 mm out from c), then its body, then a 24 mm tongue 26 mm long toward along, into the buckle"""
    n = n.normalized()
    a = (along - n * along.dot(n)).normalized()
    b = n.cross(a)
    parts = []
    for k, (x, y, lx, ly) in enumerate(((-0.0265, 0.0, 0.0050, 0.060), (-0.0090, 0.0, 0.0040, 0.060), (-0.0180, 0.0278, 0.0130, 0.0044),
                                         (-0.0180, -0.0278, 0.0130, 0.0044), (0.0010, 0.0, 0.0160, 0.044), (0.0220, 0.0, 0.0260, 0.024))):
        parts.append(_box('%s-%d' % (name, k), c + a * x + b * y, n, a, (lx, ly, 0.003), 0.0, STEEL))
    return parts


def seatbelt(rig, body, bones):
    """the lap belt, the shoulder strap, the stalk, the buckle and the three fittings, on his posed body"""
    from mathutils.bvhtree import BVHTree
    me = body.data
    P = [body.matrix_world @ v.co for v in me.vertices]
    keep = {'torso', 'leg-t-l', 'leg-t-r'}
    polys = [tuple(f.vertices) for f in me.polygons if all(SEG_OF.get(bones[i], 'torso') in keep for i in f.vertices)]
    bvh = BVHTree.FromPolygons(P, polys)
    polys2 = [tuple(f.vertices) for f in me.polygons if all(SEG_OF.get(bones[i], 'torso') in keep | {'head'} for i in f.vertices)]
    bvh2 = BVHTree.FromPolygons(P, polys2)                  # and his neck, which is his head's part: the strap crosses its base
    hip_l, hip_r = W(rig, 'thigh_l'), W(rig, 'thigh_r')
    pelvis = (hip_l + hip_r) / 2 + Vector((0.02, 0, 0.06))
    neck, shoulder = W(rig, 'neck_01'), W(rig, 'upperarm_l')
    spine = (pelvis, neck)

    def drape(guide, centre, n_out=90, off=BELT_T / 2 + 0.0015):
        """points along the guide polyline, each pushed out onto his body along the ray from its centre, if inside it"""
        L = [0.0]
        for a, b in zip(guide, guide[1:]):
            L.append(L[-1] + (b - a).length)
        pts, nrm = [], []
        for t in np.linspace(0, L[-1], n_out):
            i = max(k for k in range(len(L) - 1) if L[k] <= t + 1e-9) if t < L[-1] else len(L) - 2
            q = guide[i].lerp(guide[i + 1], (t - L[i]) / max(L[i + 1] - L[i], 1e-9))
            c = centre(q)
            d = (q - c)
            dist = d.length
            d.normalize()
            loc, n, idx, hd = bvh.ray_cast(c + d * 0.5, -d, 0.5)
            if loc is not None and (loc - c).length + off > dist:
                pts.append(loc + n * off)
                nrm.append(n)
            else:
                pts.append(q)
                nrm.append(d)
        for _ in range(3):                                   # smoothed, the ends held
            pts = [pts[0]] + [(pts[k - 1] + pts[k] * 2 + pts[k + 1]) / 4 for k in range(1, len(pts) - 1)] + [pts[-1]]
            nrm = [nrm[0]] + [(nrm[k - 1] + nrm[k] * 2 + nrm[k + 1]).normalized() for k in range(1, len(nrm) - 1)] + [nrm[-1]]
        return pts, nrm

    def on_spine(q):
        a, b = spine
        ab = b - a
        t = min(max((q - a).dot(ab) / ab.length_squared, 0.0), 1.0)
        return a + ab * t

    # the buckle: at his inboard hip, low on his pelvis
    o = Vector((hip_r.x - 0.06, hip_r.y + 0.03, hip_r.z + 0.35))          # from above, 6 cm ahead of his hip joint: onto his hip's crease
    loc, n, idx, hd = bvh.ray_cast(o, Vector((0, 0, -1)), 0.6)
    B, nB = loc + n * 0.008, n.copy()
    objs = []
    allp = BVHTree.FromPolygons(P, [tuple(f.vertices) for f in me.polygons])          # all of him, his arms and head too

    def lie(o, gap=0.002):                             # 2 mm: closer, FlightGear's shadows speckle the webbing with the suit's
        """the strap's every vertex out of him: its underside at least gap off him, its top a thickness more, so it follows him across its width"""
        me_ = o.data
        nv = len(me_.vertices)
        for _ in range(10):
            if _:                                         # smoothed along its length, each of its ten lines of vertices, so its top isn't lumpy
                co = [v.co.copy() for v in me_.vertices]
                for i in range(10, nv - 10):
                    me_.vertices[i].co = (co[i - 10] + co[i] * 2 + co[i + 10]) / 4
            for i, v in enumerate(me_.vertices):
                loc, n, idx, d = allp.find_nearest(v.co)
                if loc is None:
                    continue
                want = gap + (BELT_T if i % 10 < 5 else 0.0)
                if (v.co - loc).dot(n) < want:
                    v.co = loc + n * want
            me_.update()
            moved = 0
            for f in me_.polygons:                       # and where a face's middle is still in him, the face out
                loc, n, idx, d = allp.find_nearest(f.center)
                if loc is not None and (f.center - loc).dot(n) < gap:
                    push = n * (gap - (f.center - loc).dot(n))
                    for vi in f.vertices:
                        me_.vertices[vi].co += push
                    moved += 1
            me_.update()
            if not moved and _ >= 4:
                break
        return o
    # the buckle's line: down its stalk toward the seat's inboard side; the latch plate on it, the webbing's loop through its slot at J
    g1 = Vector((hip_r.x + 0.02, LAP_FIT['in'].y, hip_r.z - 0.02))
    sd = (g1 - B) - nB * (g1 - B).dot(nB)
    sd.normalize()
    cp = B - nB * 0.004 + sd * 0.010                     # the latch plate's centre
    J = cp - sd * 0.018                                  # its slot's
    # the lap belt: from the outboard fitting round his hip, across the front of his pelvis, to the buckle
    front = Vector((pelvis.x - 0.05, PD.PILOT_Y, hip_l.z + 0.07))          # the guide's points inside him: drape() pushes them out onto him
    side_l = Vector((hip_l.x - 0.02, hip_l.y - 0.06, hip_l.z + 0.05))
    lap, lap_n = drape([LAP_FIT['out'], side_l, front, J], lambda q: Vector((pelvis.x + 0.06, pelvis.y, q.z - 0.10)))
    lap[-1], lap_n[-1] = J, nB
    objs.append(lie(_ribbon('belt-lap', lap, lap_n, WEBBING)))
    # the shoulder strap: from the upper fitting down onto his left shoulder mid-collarbone, then laid from point to point on him: the
    # top of his shoulder found from above, his chest from in front on the line from there to the buckle; between them each point
    # pushed onto him along the blend of the two points' normals. Over the free span its face stays as on his shoulder
    def onto(p, d, tree=bvh2):
        loc, n, idx, hd = tree.ray_cast(p + d * 0.3, -d, 0.6)
        return (loc, n) if loc is not None else (p, d)

    off = BELT_T / 2 + 0.0015
    S, nS = onto(neck.lerp(shoulder, 0.45), Vector((0, 0, 1)), bvh)       # his shoulder only: from above, his head is in the way
    C1, n1 = onto(S.lerp(B, 0.33), Vector((-1, 0, 0.4)).normalized())
    C2, n2 = onto(S.lerp(B, 0.66), Vector((-1, 0, 0)))
    marks = [(S, nS), (C1, n1), (C2, n2), (J - nB * 0.004, nB)]
    sh, sh_n = [], []
    clear = 1.0
    for t in np.linspace(0, 1, 14)[:-1]:                 # in the air from the fitting to his shoulder, out of him wherever it would touch
        q = UPPER_FIT.lerp(S + nS * off, t)
        loc, n, idx, d = allp.find_nearest(q)
        if loc is not None:
            dist = d if (q - loc).dot(n) > 0 else -d
            clear = min(clear, dist)
            if dist < off:
                q = loc + n * off
        sh.append(q)
        sh_n.append(nS.copy())
    print('MKPILOT seatbelt: its free span %.1f mm off him at least, before any push' % (clear * 1000))
    for (a, na), (b, nb) in zip(marks, marks[1:]):
        for t in np.linspace(0, 1, 28, endpoint=False):
            loc, n = onto(a.lerp(b, t), na.lerp(nb, t).normalized())
            sh.append(loc + n * off)
            sh_n.append(n)
    sh.append(J)
    sh_n.append(nB)
    for _ in range(3):
        sh = [sh[0]] + [(sh[k - 1] + sh[k] * 2 + sh[k + 1]) / 4 for k in range(1, len(sh) - 1)] + [sh[-1]]
        sh_n = [sh_n[0]] + [(sh_n[k - 1] + sh_n[k] * 2 + sh_n[k + 1]).normalized() for k in range(1, len(sh_n) - 1)] + [sh_n[-1]]
    objs.append(lie(_ribbon('belt-shoulder', sh, sh_n, WEBBING)))
    seg_of_face = [SEG_OF.get(bones[f.vertices[0]], 'torso') for f in me.polygons]
    for o in objs[-2:]:                                  # the lap belt and shoulder strap: any of their vertices inside him, and where
        bad = []
        pts_ = [v.co.copy() for v in o.data.vertices] + [f.center.copy() for f in o.data.polygons]
        for q in pts_:
            loc, n, idx, d = allp.find_nearest(q)
            if loc is not None and (q - loc).dot(n) < 0:
                bad.append((d, seg_of_face[idx], tuple(round(c, 3) for c in q)))
        bad.sort(reverse=True)
        print('MKPILOT belt check %s: %d of %d vertices and face centres inside him%s' % (o.name, len(bad), len(pts_),
              ''.join('; %.1f mm in his %s at %s' % (d * 1000, sg, p) for d, sg, p in bad[:6])))
    # the buckle's stalk, from the buckle down past his hip to the seat's inboard side
    # the buckle on its stalk: a black housing with a red release button, the latch plate's tongue plugged into its end (sizes mine,
    # as such buckles are); the stalk from the housing's far end down past his hip to the seat's inboard side
    hc = B - nB * 0.001 + sd * 0.062
    objs.append(_box('belt-buckle', hc, nB, sd, (0.064, 0.040, 0.018), 0.004))
    objs.append(_box('belt-button', hc + nB * 0.010 - sd * 0.012, nB, sd, (0.024, 0.022, 0.003), 0.001))
    objs += _latch('belt-tongue', cp, nB, sd)
    stalk, stalk_n = drape([hc + sd * 0.028, g1, LAP_FIT['in']], lambda q: Vector((hip_r.x, hip_r.y, q.z)), 30)
    objs.append(lie(_ribbon('belt-stalk', stalk, stalk_n, WEBBING, 0.034)))
    objs.append(_plate('belt-fit-upper', UPPER_FIT, Vector((0, 1, 0)), Vector((0, 0, 1)), (0.05, 0.035, 0.008), STEEL))
    for k, p in LAP_FIT.items():
        objs.append(_plate('belt-fit-' + k, p, Vector((0, 1 if k == 'out' else -1, 0)), Vector((1, 0, 0)), (0.05, 0.03, 0.008), STEEL))
    lengths = [sum((b - a).length for a, b in zip(q, q[1:])) for q in (lap, sh, stalk)]
    print('MKPILOT seatbelt: lap %.2f m, shoulder %.2f m, stalk %.2f m; the buckle at %s' % (*lengths, tuple(round(c, 3) for c in B)))
    return objs


# ------------------------------------------------------------------------------------------------- checks
def render(name, cam_loc, look, lens=30):
    sc = bpy.context.scene
    sc.render.engine = 'BLENDER_WORKBENCH'
    sc.display.shading.light = 'STUDIO'
    sc.display.shading.color_type = 'TEXTURE'
    sc.render.resolution_x, sc.render.resolution_y = 1200, 800
    cam = bpy.data.objects.get('cam') or bpy.data.objects.new('cam', bpy.data.cameras.new('cam'))
    if cam.name not in sc.collection.objects:
        sc.collection.objects.link(cam)
    cam.data.lens = lens
    cam.data.clip_start = 0.01
    cam.location = cam_loc
    cam.rotation_euler = (Vector(look) - Vector(cam_loc)).to_track_quat('-Z', 'Y').to_euler()
    sc.camera = cam
    os.makedirs(CHECK, exist_ok=True)
    sc.render.filepath = os.path.join(CHECK, name + '.png')
    bpy.ops.render.render(write_still=True)


def simple_material(name, image=None, colour=(0.5, 0.5, 0.5)):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes['Principled BSDF']
    bsdf.inputs['Base Color'].default_value = (*colour, 1)
    bsdf.inputs['Roughness'].default_value = 0.8
    if image is not None:
        t = m.node_tree.nodes.new('ShaderNodeTexImage')
        t.image = image
        m.node_tree.links.new(t.outputs['Color'], bsdf.inputs['Base Color'])
    return m


def main():
    os.makedirs(OUT, exist_ok=True)
    clear()
    ctx = context() if ARGS.get('render', True) else []
    body, rig, eyes, boots = human()
    scan_shoes(boots)
    BONES.update(b.name for b in rig.data.bones)
    rest = store_rest(body)
    lv = levels(rig)
    pose(rig, body)
    # everything to plain meshes in the posed shape
    meshes = {}
    for o in (body, eyes, boots):
        bpy.ops.object.select_all(action='DESELECT')
        o.select_set(True)
        bpy.context.view_layer.objects.active = o
        bpy.ops.object.convert(target='MESH')
        meshes[o.name] = o
    print('MKPILOT converted', {k: len(o.data.vertices) for k, o in meshes.items()})
    # the suit off the skin
    me = body.data
    R = np.array([0.0] * (3 * len(me.vertices)))
    me.attributes['rest'].data.foreach_get('vector', R)
    R = R.reshape(-1, 3)
    bones = bone_of(body, me)
    suit = ease(R, bones, lv, suit_at(R, bones, lv))
    nrm = [v.normal.copy() for v in me.vertices]
    for v in me.vertices:
        if suit[v.index] > 0:
            v.co += nrm[v.index] * SUIT_OFF * suit[v.index]
    me.update()
    eyes_R = []
    em = eyes.data
    if 'rest' not in em.attributes:
        # the eyes' rest positions: from where they are now, back through the rig's head bone
        pass
    # the eyes' centres in MPFB's rest frame, from the body's eye sockets: the head's front, either side
    ev = np.array([v.co[:] for v in em.vertices])
    mw = body.matrix_world
    head = bones == 'head'
    H = R[head]
    ez = np.percentile(H[:, 2], 62)
    fy = H[:, 1].min()
    eye_l, eye_r = np.array((0.032, fy + 0.02, ez)), np.array((-0.032, fy + 0.02, ez))
    skin = os.path.join(ASSETS, 'skins', 'middleage_caucasian_male', 'middleage_lightskinned_male_diffuse.png')
    im = paint(body, me, R, bones, lv, eye_l, eye_r, skin, os.path.join(OUT, 'pilot-body.png'))
    body.data.materials.clear()
    body.data.materials.append(simple_material('pilot-body', im))
    G = head_geometry(rig, body, eyes)
    hs, pads = headset(G)
    hs += glasses(G, pads)
    belt = seatbelt(rig, body, bones)
    for o in hs + belt:
        o.color = (*o['colour'], 1)
        o.data.materials.append(simple_material(o.name, None, o['colour']))
    print('MKPILOT faces', {o.name: len(o.data.polygons) for o in (body, eyes, boots)}, 'headset', sum(len(o.data.polygons) for o in hs))
    # textures for the boots and eyes, as files beside the model
    for o, src, name in ((boots, None, 'pilot-boots.png'),                       # scan_shoes() put it there
                         (eyes, os.path.join(ASSETS, 'eyes', 'materials', 'brown_eye.png'), 'pilot-eyes.png')):
        if src:
            shutil.copy(src, os.path.join(OUT, name))
        o.data.materials.clear()
        o.data.materials.append(simple_material(name[:-4], bpy.data.images.load(os.path.join(OUT, name))))
    if ARGS.get('hand_check'):                             # his left knuckles' line against the upright grip
        K = [W(rig, f + '_01_l') for f in ('index', 'middle', 'ring', 'pinky')]
        line = (K[0] - K[3]).normalized()
        print('MKPILOT knuckles: index to pinky %.1f mm, %.1f deg off the upright; their distances from the grip axis %s mm; the line %s' % (
            (K[0] - K[3]).length * 1000, math.degrees(math.acos(abs(line.z))),
            [round(math.hypot(k.x - GRIP_L.x, k.y - GRIP_L.y) * 1000) for k in K], tuple(round(c, 3) for c in line)))
        print('MKPILOT knuckles at', [tuple(round(c, 3) for c in k) for k in K])
    tables = solve(rig)
    print('MKPILOT left wrist gap, cm', tables['left_wrist_gap_cm'], '; short of the grip at worst, cm', tables['left_reach_short_cm'])
    joints = {'neck': W(rig, 'neck_01'), 'shoulder_l': W(rig, 'upperarm_l'), 'elbow_l': W(rig, 'lowerarm_l'), 'wrist_l': W(rig, 'hand_l'),
              'shoulder_r': W(rig, 'upperarm_r'), 'elbow_r': W(rig, 'lowerarm_r'), 'wrist_r': W(rig, 'hand_r'),
              'hip_l': W(rig, 'thigh_l'), 'knee_l': W(rig, 'calf_l'), 'ankle_l': W(rig, 'foot_l'),
              'hip_r': W(rig, 'thigh_r'), 'knee_r': W(rig, 'calf_r'), 'ankle_r': W(rig, 'foot_r')}
    if ARGS.get('render', True):
        for o in ctx:
            if o.name in ('interieur', 'int-roof', 'intporteG', 'inttourporteG', 'inttour', 'int-bulk'):
                o.hide_render = True
        render('side', (-2.35, -2.2, -0.15), (-2.45, -0.28, -0.35), 30)
        render('face', (-2.75, -0.40, 0.10), (-2.13, -0.285, 0.12), 45)
        render('rear', (-1.55, 0.25, 0.20), (-2.40, -0.30, -0.40), 24)
        render('shoes', (-2.62, -0.80, -0.42), (-2.93, -0.29, -0.68), 34)
        render('belt', (-2.80, -0.62, 0.05), (-2.12, -0.28, -0.32), 30)
    for o in ctx:
        bpy.data.objects.remove(o)
    # off the rig, where the rig put them: it goes, and they would fall back to MPFB's standing frame
    for o in (body, eyes, boots):
        mw = o.matrix_world.copy()
        o.parent = None
        o.matrix_world = mw
    bpy.context.view_layer.update()
    # cut him at the joints, the boots and eyes too, the headset with his head
    parts = []
    for o, part in ((body, 'body'), (boots, 'boots'), (eyes, 'eyes')):
        parts += split(o, part, bone_of(o, o.data))
    for o in hs:
        o.name = 'head.' + o.name
        parts.append(o)
    for o in belt:
        o.name = 'torso.' + o.name
        parts.append(o)
    for o in [o for o in bpy.data.objects if o not in parts]:
        bpy.data.objects.remove(o)
    objects = {}
    for o in parts:
        objects.setdefault(o.name.split('.')[0], []).append(o.name)
    export(parts, dict(objects=objects, joints={k: tuple(round(c, 5) for c in v) for k, v in joints.items()}, tables=tables,
                       yoke=dict(origin=tuple(YOKE_ORIGIN), slide=YOKE_SLIDE, turn=YOKE_TURN), lever=dict(pivot=tuple(LEVER_PIVOT)),
                       faces=sum(len(o.data.polygons) for o in parts)))
    print('MKPILOT exported', len(parts), 'parts,', sum(len(o.data.polygons) for o in parts), 'faces')


main()
