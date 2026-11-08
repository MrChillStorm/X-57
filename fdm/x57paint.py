#!/usr/bin/env python3
"""The aircraft's paint, baked. The paint is drawn in the aircraft's own frame, as a side view, and each texel of the Tecnam's white
texture takes the colour of the point of the fuselage it lands on, so the lines run true across every panel, door and side.

    python3 x57paint.py [--map out.png]       writes ../X-57/Models/texture.png, or a side view of the fuselage for drawing on

The scheme is the X-57's own: a white body, a navy belly rising to the fin, and red pinstripes that start as one line at the nose and fan
out behind the wing. No agency's insignia or lettering, and no registration: the fin says X-57.
"""
import os, re, sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy import ndimage
from scipy.interpolate import PchipInterpolator

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'X-57', 'Models')
BASE = os.path.join(HERE, '..', 'refs', 'fgaddon-p2006t', 'Tecnam-P2006T', 'Models', 'Liveries', '2048x2048', 'Full_White.png')
PAINTED = ('fuselage', 'direction', 'porteD', 'porteG', 'porteA', 'porteAD', 'porteAG', 'portetrainD', 'portetrainG')
NAVY = np.array((16, 30, 84)) / 255.0
RED = np.array((196, 22, 28)) / 255.0
BOLD = '/System/Library/Fonts/Supplemental/Arial Bold.ttf'


def objects(path):
    """the model's objects: name, vertices in the model's frame (x aft, y up, z left), polygons of (vertex, u, v)"""
    out = []
    for o in re.split(r'(?=^OBJECT )', open(path, errors='replace').read(), flags=re.M):
        n = re.search(r'^name "([^"]+)"', o, re.M)
        c = re.search(r'^numvert (\d+)', o, re.M)
        if not (n and c):
            continue
        loc = re.search(r'^loc (.*)', o, re.M)
        L = o[c.end():].lstrip('\n').split('\n')
        V = np.array([[float(q) for q in l.split()] for l in L[:int(c.group(1))]])
        V = V + (np.array([float(q) for q in loc.group(1).split()]) if loc else 0.0)
        polys = []
        for m in re.finditer(r'^refs (\d+)\n((?:\d+ \S+ \S+\n?)+)', o, re.M):
            polys.append([(int(a), float(b), float(c_)) for a, b, c_ in (l.split() for l in m.group(2).strip().split('\n'))])
        out.append((n.group(1), V, polys))
    return out


def position_map(objs, size):
    """the point of the model behind each texel, and which way it faces: NaN where no painted part uses the texel"""
    P = np.full((size, size, 3), np.nan)
    N = np.zeros((size, size, 3))
    for name, V, polys in objs:
        if name not in PAINTED:
            continue
        for poly in polys:
            for k in range(1, len(poly) - 1):
                tri = (poly[0], poly[k], poly[k + 1])
                uv = np.array([(t[1] * size, (1.0 - t[2]) * size) for t in tri])
                p = V[[t[0] for t in tri]]
                n = np.cross(p[1] - p[0], p[2] - p[0])
                n = n / (np.linalg.norm(n) + 1e-12)
                x0, y0 = np.maximum(np.floor(uv.min(0)).astype(int) - 1, 0)
                x1, y1 = np.minimum(np.ceil(uv.max(0)).astype(int) + 2, size)
                if x1 <= x0 or y1 <= y0:
                    continue
                gx, gy = np.meshgrid(np.arange(x0, x1) + 0.5, np.arange(y0, y1) + 0.5)
                d = (uv[1, 1] - uv[2, 1]) * (uv[0, 0] - uv[2, 0]) + (uv[2, 0] - uv[1, 0]) * (uv[0, 1] - uv[2, 1])
                if abs(d) < 1e-9:
                    continue
                a = ((uv[1, 1] - uv[2, 1]) * (gx - uv[2, 0]) + (uv[2, 0] - uv[1, 0]) * (gy - uv[2, 1])) / d
                b = ((uv[2, 1] - uv[0, 1]) * (gx - uv[2, 0]) + (uv[0, 0] - uv[2, 0]) * (gy - uv[2, 1])) / d
                c = 1.0 - a - b
                m = (a >= -0.02) & (b >= -0.02) & (c >= -0.02)
                if m.any():
                    q = a[m, None] * p[0] + b[m, None] * p[1] + c[m, None] * p[2]
                    P[y0:y1, x0:x1][m] = q
                    N[y0:y1, x0:x1][m] = n
    return P, N


def grow(P, N, px=5):
    """past the edges of each part, so that filtering the texture doesn't bring in the white beside it"""
    hole = np.isnan(P[..., 0])
    d, (iy, ix) = ndimage.distance_transform_edt(hole, return_indices=True)
    near = hole & (d <= px)
    P[near], N[near] = P[iy[near], ix[near]], N[iy[near], ix[near]]
    return P, N


# the lines, in the side view: x aft, y up, metres, from the fuselage's own shape (--map). The belly's top edge, and how far the stripes have fanned out
BELLY = PchipInterpolator([-4.40, -4.25, -3.70, -2.60, -1.20, 0.30, 1.50, 2.40, 3.00, 3.45, 3.80, 4.40],
                          [-0.62, -0.70, -0.76, -0.76, -0.72, -0.60, -0.40, -0.14, 0.07, 0.23, 0.34, 0.55])
FAN = PchipInterpolator([-4.40, -2.70, -1.40, 0.00, 3.00, 4.40], [0.0, 0.0, 0.45, 1.0, 1.0, 1.0])
STRIPES, GAP, WIDTH = 4, 0.082, 0.034


def scheme(x, y):
    """colour weights at side-view points: navy below the belly line, then the pinstripes above it, closing to one line at the nose"""
    b, f = BELLY(x), FAN(x)
    navy = np.clip((b - y) / 0.008 + 0.5, 0.0, 1.0)
    red = np.zeros_like(x)
    for i in range(STRIPES):
        c = b + 0.055 + i * GAP * f
        w = WIDTH * (0.65 + 0.35 * f) / 2
        red = np.maximum(red, np.clip((w - np.abs(y - c)) / 0.006 + 0.5, 0.0, 1.0))
    return navy, red


def fin_words(size=(1400, 500)):
    """X-57 for the fin, as a mask over WORDS, leaning with the fin's sweep"""
    im = Image.new('L', size, 0)
    g = ImageDraw.Draw(im)
    f = ImageFont.truetype(BOLD, 400)
    box = g.textbbox((0, 0), 'X-57', font=f)
    g.text(((size[0] - box[2] - box[0]) / 2, (size[1] - box[3] - box[1]) / 2), 'X-57', font=f, fill=255)
    return np.asarray(im.transform(size, Image.AFFINE, (1, 0.34, -60, 0, 1, 0), Image.BICUBIC), float) / 255.0


WORDS = dict(x0=3.42, x1=4.26, y0=0.90, y1=1.20)


def stamp(mask, x, y, z, nz):
    """the words on each side of the fin, reading forward to aft on the left and aft to forward on the right"""
    H, W = mask.shape
    u = (x - WORDS['x0']) / (WORDS['x1'] - WORDS['x0'])
    u = np.where(z > 0, u, 1.0 - u)
    v = (WORDS['y1'] - y) / (WORDS['y1'] - WORDS['y0'])
    on = (u >= 0) & (u < 1) & (v >= 0) & (v < 1) & (np.abs(nz) > 0.5)
    out = np.zeros_like(x)
    out[on] = ndimage.map_coordinates(mask, [v[on] * (H - 1), u[on] * (W - 1)], order=1)
    return out


def paint(ac=None, out=None):
    ac = ac or os.path.join(OUT, 'fuselage.ac')
    out = out or os.path.join(OUT, 'texture.png')
    base = np.asarray(Image.open(BASE).convert('RGB'), float) / 255.0
    P, N = position_map(objects(ac), base.shape[0])
    P, N = grow(P, N)
    k = ~np.isnan(P[..., 0])
    x, y, z = P[k, 0], P[k, 1], P[k, 2]
    navy, red = scheme(x, y)
    words = stamp(fin_words(), x, y, z, N[k, 2])
    shade = base[k] / np.maximum(np.percentile(base[k], 90, axis=0), 1e-6)         # the white's own panel lines and shading stay
    shade = np.clip(shade, 0.0, 1.0)
    c = base[k].copy()
    for w, colour in ((red, RED), (navy, NAVY), (words, NAVY)):
        c = c * (1 - w[:, None]) + shade * colour * w[:, None]
    img = base.copy()
    img[k] = c
    Image.fromarray((np.clip(img, 0, 1) * 255 + 0.5).astype(np.uint8)).save(out)
    print('painted %d texels of %s' % (k.sum(), os.path.basename(out)))


def side_map(path):
    """the fuselage from the left, x aft to the right, with the lines on it: for drawing them"""
    P, N = position_map(objects(os.path.join(OUT, 'fuselage.ac')), 2048)
    k = ~np.isnan(P[..., 0]) & (P[..., 2] > 0)
    s = 160.0
    W, H = int(9.0 * s), int(2.7 * s)
    x, y = P[k, 0], P[k, 1]
    navy, red = scheme(x, y)
    words = stamp(fin_words(), x, y, P[k, 2], N[k, 2])
    img = np.full((H, W, 3), 1.0)
    img[..., :] = (0.75, 0.85, 0.95)
    c = np.ones((len(x), 3)) * 0.97
    for w, col in ((red, RED), (navy, NAVY), (words, NAVY)):
        c = c * (1 - w[:, None]) + col * w[:, None]
    px = ((x + 4.5) * s).astype(int).clip(0, W - 1)
    py = ((1.6 - y) * s).astype(int).clip(0, H - 1)
    img[py, px] = c
    im = Image.fromarray((img * 255).astype(np.uint8))
    g = ImageDraw.Draw(im)
    for xx in np.arange(-4, 5):
        g.line([((xx + 4.5) * s, 0), ((xx + 4.5) * s, H)], fill=(120, 120, 120))
        g.text(((xx + 4.5) * s + 3, 3), '%d' % xx, fill=(0, 0, 0))
    for yy in np.arange(-1, 2, 0.5):
        g.line([(0, (1.6 - yy) * s), (W, (1.6 - yy) * s)], fill=(150, 150, 150))
        g.text((3, (1.6 - yy) * s + 2), '%.1f' % yy, fill=(0, 0, 0))
    im.save(path)
    print('side view', im.size)


if __name__ == '__main__':
    if len(sys.argv) > 2 and sys.argv[1] == '--map':
        side_map(sys.argv[2])
    else:
        paint()
