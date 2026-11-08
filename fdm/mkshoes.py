"""The pilot's work shoes, from a photogrammetry scan of a pair of black leather work shoes with two hook-and-loop straps (this
aircraft's author's own, filmed turning on a glass plate; each frame's shoe masked from the floor, then Apple's Object Capture).
The scans saw the shoes' sides down to the ground, their black soles too, and never under them: below that Object Capture
guessed, in grey. So each scan is cut where it turns black, its bottom closed flat there, turned into the foot's frame (toe toward
-y, up z, x the wearer's left) and scaled to the pair's size, UK 7.5: a last of (7.5 + 25) / 3 in = 27.5 cm, the shoe's outside
about 2 cm more for its toe cap, 29.5 cm (an estimate). Each is cut to about FACES faces. Underneath, lugs drawn as the user's
photograph of the sole shows them (black chevrons, a light shank and heel insert), mine. Their textures: both scans' and the
underside's in one atlas, the maker's yellow heel label painted out, the floor the scans saw through the soles' clear rubber
darkened; no maker's name or mark.

Run once in Blender 4.3, with the scans' folders (Object Capture's OBJ output) and the heights where each upper turns black:
  Blender -b --python mkshoes.py -- '{"left": DIR, "right": DIR, "cut_left": 0.20, "cut_right": 0.105}'
It writes data/pilot/shoes.obj (shoe-l, shoe-r, metres, in that frame, their soles at z 0) and data/pilot/shoes.png, which
mkpilot.py fits to his feet.
"""
import bpy, bmesh, glob, json, math, os, sys
import numpy as np
from mathutils import Vector, Matrix

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'data', 'pilot')
ARGS = json.loads(sys.argv[sys.argv.index('--') + 1])
LENGTH = 0.295                         # the shoe's outside, heel to toe: UK 7.5
FACES = 3500                           # each scan's upper cut to about this many faces
LOW = 0.035                            # the lowest 3.5 cm: the scans saw the floor through the soles' clear rubber there
ATLAS = (2048, 1024)                   # the two scans side by side over the top 896 rows; the underside's strip below
SCAN_ROWS = 896


def load(path):
    before = set(bpy.data.objects)
    bpy.ops.wm.obj_import(filepath=glob.glob(os.path.join(path, '*.obj'))[0], forward_axis='NEGATIVE_Z', up_axis='Y')
    ob = [o for o in set(bpy.data.objects) - before if o.type == 'MESH'][0]
    tex = glob.glob(os.path.join(path, '*_tex0.png'))[0]
    return ob, tex


def frame(ob, cut):
    """the shoe's frame from its upper: the long axis from the top of it, the heel the end nearer its collar"""
    P = np.array([(ob.matrix_world @ v.co)[:] for v in ob.data.vertices])
    up = P[P[:, 2] > cut]
    top = up[up[:, 2] > np.percentile(up[:, 2], 70)]
    c = up[:, :2].mean(0)
    w, U = np.linalg.eigh(np.cov((up[:, :2] - c).T))
    ax = U[:, 1]
    along = (up[:, :2] - c) @ ax
    collar = ((top[:, :2] - c) @ ax).mean()
    if collar < (along.min() + along.max()) / 2:               # the heel toward +y: the collar is nearer the heel
        ax = -ax
    along = (up[:, :2] - c) @ ax
    y = Vector((ax[0], ax[1], 0.0))                            # toe to heel
    z = Vector((0, 0, 1))
    x = y.cross(z)                                             # the wearer's left
    k = LENGTH / float(np.ptp(along))
    mid = Vector((c[0], c[1], 0)) + y * float((along.min() + along.max()) / 2)
    R = Matrix((x, y, z))
    return Matrix.Scale(k, 4) @ R.to_4x4() @ Matrix.Translation(-mid - Vector((0, 0, cut))), k


def cut_and_place(ob, cut):
    M, k = frame(ob, cut)
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bm.transform(ob.matrix_world)
    bm.transform(M)
    bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], plane_co=(0, 0, 0), plane_no=(0, 0, 1), clear_inner=True)
    bm.to_mesh(ob.data)
    bm.free()
    ob.matrix_world = Matrix.Identity(4)
    ob.data.update()
    # fewer faces: the upper's shape and its UVs kept
    mod = ob.modifiers.new('fewer', 'DECIMATE')
    mod.ratio = min(1.0, FACES / max(1, len(ob.data.polygons)))
    bpy.context.view_layer.objects.active = ob
    bpy.ops.object.modifier_apply(modifier=mod.name)
    return k


def close_sole(ob, side):
    """its bottom closed flat at the cut, the lugs' strip under it; the scan's own picture into its half of the atlas"""
    me = ob.data
    bm = bmesh.new()
    bm.from_mesh(me)
    uv = bm.loops.layers.uv.active
    low = [[tuple(l[uv].uv) for l in f.loops] for f in bm.faces if f.calc_center_median().z < LOW]
    for f in bm.faces:
        for l in f.loops:
            l[uv].uv = ((l[uv].uv.x + (0 if side == 'l' else 1)) * 0.5, 1 - (1 - l[uv].uv.y) * SCAN_ROWS / ATLAS[1])
    edge = [e for e in bm.edges if e.is_boundary and all(v.co.z < 0.002 for v in e.verts)]
    made = bmesh.ops.holes_fill(bm, edges=edge, sides=0)['faces']
    made = bmesh.ops.triangulate(bm, faces=made)['faces']
    ys = np.array([v.co.y for v in bm.verts])
    toe, heel = ys.min(), ys.max()
    v0 = 1 - SCAN_ROWS / ATLAS[1]
    bm.normal_update()
    for f in made:
        if f.normal.z > 0:
            f.normal_flip()
        for l in f.loops:
            p = l.vert.co
            l[uv].uv = (0.5 * (p.y - toe) / (heel - toe), v0 * (0.5 + p.x / 0.14))      # the strip's left half
    bm.to_mesh(me)
    bm.free()
    me.update()
    return toe, heel, low, len(made)


def fill(mask, polys):
    """the polygons (pixel coordinates) filled into the mask"""
    H, W = mask.shape
    for P in polys:
        P = np.array(P)
        for k in range(1, len(P) - 1):
            T = P[[0, k, k + 1]]
            x0, y0 = np.floor(T.min(0)).astype(int)
            x1, y1 = np.ceil(T.max(0)).astype(int) + 1
            x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, W), min(y1, H)
            if x1 <= x0 or y1 <= y0:
                continue
            X, Y = np.meshgrid(np.arange(x0, x1) + 0.5, np.arange(y0, y1) + 0.5)
            (ax, ay), (bx, by), (qx, qy) = T
            den = (by - qy) * (ax - qx) + (qx - bx) * (ay - qy)
            if abs(den) < 1e-9:
                continue
            l1 = ((by - qy) * (X - qx) + (qx - bx) * (Y - qy)) / den
            l2 = ((qy - ay) * (X - qx) + (ax - qx) * (Y - qy)) / den
            mask[y0:y1, x0:x1] |= (l1 >= -0.05) & (l2 >= -0.05) & (1 - l1 - l2 >= -0.05)


def atlas(tex_l, tex_r, lows):
    """the two scans' pictures side by side, their yellow labels painted out; the outsole's strip drawn below"""
    W, H = ATLAS
    img = np.zeros((H, W, 3), float)
    for k, path in enumerate((tex_l, tex_r)):
        im = bpy.data.images.load(path)
        im.scale(W // 2, SCAN_ROWS)
        a = np.array(im.pixels[:], np.float32).reshape(SCAN_ROWS, W // 2, 4)[::-1, :, :3] * 255
        bpy.data.images.remove(im)
        r, g, b = a[..., 0], a[..., 1], a[..., 2]
        yellow = (r > 70) & (g > 60) & (b < 0.6 * g) & (r - b > 35)
        grown = yellow.copy()
        for _ in range(4):                                     # and a few texels round them
            grown |= np.roll(grown, 1, 0) | np.roll(grown, -1, 0) | np.roll(grown, 1, 1) | np.roll(grown, -1, 1)
        dark = np.median(a[(a.max(2) < 70) & ~grown], axis=0) if ((a.max(2) < 70) & ~grown).any() else np.array((28, 28, 30))
        a[grown] = dark
        print('MKSHOES %s: %d yellow texels painted out' % ('lr'[k], int(yellow.sum())))
        m = np.zeros(a.shape[:2], bool)                        # the upper's lowest band: dark rubber and leather, not the floor
        fill(m, [[(u * W // 2, (1 - v) * SCAN_ROWS) for u, v in poly] for poly in lows['lr'[k]]])
        light = m & (a.max(2) > 45)
        a[light] *= 45.0 / a[light].max(1, keepdims=True)
        print('MKSHOES %s: %d light texels in the lowest band darkened' % ('lr'[k], int(light.sum())))
        img[:SCAN_ROWS, k * W // 2:(k + 1) * W // 2] = a
    # the strip: under the sole (left half) chevron lugs, a groove down the middle, a light shank and heel insert
    rows = H - SCAN_ROWS
    grain = np.random.default_rng(3).normal(0, 1, (rows, W))
    v = np.linspace(1, 0, rows)[:, None]                       # 1 at the strip's top
    u = np.linspace(0, 1, W // 2)[None, :]
    b = u                                                      # along the sole, toe 0 .. heel 1
    A = (v - 0.5) / 0.4                                        # across it
    lug = np.mod((b + 0.10 * np.abs(A)) / 0.034, 1.0) < 0.62
    s = np.where(lug, 36.0, 12.0)
    s = np.where(np.abs(A) < 0.06, 12, s)
    shank = (np.abs(b - 0.53) < 0.13) & (np.abs(A) < 0.22 + 0.75 * ((b - 0.53) / 0.13) ** 2)
    heel = (np.abs(b - 0.88) < 0.04) & (np.abs(A) < 0.40 - 0.18 * (1 - np.abs(b - 0.88) / 0.04))
    s = np.where(shank | heel, 168, s)
    img[SCAN_ROWS:, :W // 2] = (s + 2 * grain[:, :W // 2])[..., None]
    img[SCAN_ROWS:, W // 2:] = (24 + grain[:, W // 2:])[..., None]          # its right half plain black: mkpilot's socks
    img[SCAN_ROWS:, :, 2] += 2
    im = bpy.data.images.new('shoes', W, H)
    im.pixels.foreach_set(np.concatenate([np.clip(img[::-1], 0, 255) / 255.0, np.ones((H, W, 1))], -1).astype(np.float32).ravel())
    im.filepath_raw = os.path.join(OUT, 'shoes.png')
    im.file_format = 'PNG'
    im.save()


def main():
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o)
    texs, lows = {}, {}
    for side in ('l', 'r'):
        key = 'left' if side == 'l' else 'right'
        ob, tex = load(ARGS[key])
        n0 = len(ob.data.polygons)
        k = cut_and_place(ob, ARGS['cut_' + key])
        toe, heel, lows[side], nb = close_sole(ob, side)
        ob.name = ob.data.name = 'shoe-' + side
        texs[side] = tex
        xs = [v.co.x for v in ob.data.vertices]
        print('MKSHOES %s: %d faces scanned, %d kept, its bottom closed; scale %.3f m per scan unit; %.3f long, %.3f wide, %.3f high' % (
            side, n0, len(ob.data.polygons), k, heel - toe, max(xs) - min(xs), max(v.co.z for v in ob.data.vertices)))
    atlas(texs['l'], texs['r'], lows)
    bpy.ops.object.select_all(action='DESELECT')
    for side in ('l', 'r'):
        bpy.data.objects['shoe-' + side].select_set(True)
    bpy.ops.wm.obj_export(filepath=os.path.join(OUT, 'shoes.obj'), export_selected_objects=True, export_materials=False,
                          forward_axis='Y', up_axis='Z', export_normals=True, export_uv=True)
    print('MKSHOES wrote', os.path.join(OUT, 'shoes.obj'))


main()
