#!/usr/bin/env python3
"""Digitise NASA's predicted flying qualities for Mod III and Mod IV [20240006496, figs. 18-24] into
data/digitised/flying_qualities.json.

    python3 digitise_fq.py

The figures are stored in the PDF as lossless images, so markers are found by colour and shape, and scaled by the
axes' own limits: Mod III a blue dot, Mod IV in fixed-rpm mode a red plus, Mod IV in airspeed mode a black cross. All
are in the takeoff configuration. These are NASA's simulator's predictions, not measurements.

The eigenvalue plots label their imaginary axis Hz. It is rad/s: the same plots' damping lines read true with both
axes in one unit, the phugoid would otherwise have a period of 4 s, and the damping ratios worked out from them match
figs. 19, 21 and 24 (checked below).
"""
import json, math, os, subprocess, sys
import numpy as np
from PIL import Image
from scipy import ndimage as ndi
from scipy.signal import fftconvolve

HERE = os.path.dirname(os.path.abspath(__file__))
PDF = os.path.join(HERE, '..', 'refs', '20240006496-ModIII-IV-piloted-sim.pdf')
TMP = os.path.join(HERE, 'out', 'fq')
KCAS = (75, 80, 88, 96, 104, 112)


def images():
    os.makedirs(TMP, exist_ok=True)
    subprocess.run(['pdfimages', '-png', '-f', '17', '-l', '21', PDF, os.path.join(TMP, 'f')], check=True)
    return {name: np.asarray(Image.open(os.path.join(TMP, 'f-%03d.png' % i)).convert('RGB')).astype(int)
            for i, name in enumerate(('fig18', 'fig19', 'fig20', 'fig21', 'fig22', 'fig23', 'fig24'))}


def bands(frac, thresh):
    """runs of True in frac > thresh, as (first, last)"""
    idx = np.where(frac > thresh)[0]
    if not len(idx):
        return []
    cut = np.where(np.diff(idx) > 1)[0]
    return [(int(a[0]), int(a[-1])) for a in np.split(idx, cut + 1)]


def frames(img):
    """the plot frames: where most of a row and column isn't white. Returns [(x0, y0, x1, y1)] top to bottom"""
    nonwhite = (img.sum(axis=2) < 740)
    rows = bands(nonwhite.mean(axis=1), 0.45)
    out = []
    for y0, y1 in rows:
        if y1 - y0 < 40:
            continue
        cols = bands(nonwhite[y0:y1 + 1].mean(axis=0), 0.45)
        x0, x1 = cols[0][0], cols[-1][1]
        out.append((x0, y0, x1, y1))
    return out


def lines(img):
    """the frame of an unshaded plot: its long dark lines"""
    dark = img.sum(axis=2) < 300
    h = np.where(dark.mean(axis=1) > 0.55)[0]
    v = np.where(dark.mean(axis=0) > 0.45)[0]
    return int(v.min()), int(h.min()), int(v.max()), int(h.max())


class Plot:
    def __init__(s, img, box, xlim, ylim):
        s.img, s.box, s.xlim, s.ylim = img, box, xlim, ylim
        x0, y0, x1, y1 = box
        s.crop = img[y0:y1 + 1, x0:x1 + 1]
        s.mask = np.ones(s.crop.shape[:2], bool)

    def hide(s, x0, y0, x1, y1):
        """leave out a rectangle, in fractions of the frame from its top left"""
        H, W = s.mask.shape
        s.mask[int(y0 * H):int(y1 * H), int(x0 * W):int(x1 * W)] = False

    def hide_legend(s):
        """a shaded plot's legend is its one white box"""
        c = s.crop
        white = (c.min(axis=2) > 245)
        lab, n = ndi.label(ndi.binary_opening(white, iterations=2))
        if n:
            big = 1 + int(np.argmax(ndi.sum(white, lab, range(1, n + 1))))
            sl = ndi.find_objects(lab)[big - 1]
            s.mask[max(sl[0].start - 3, 0):sl[0].stop + 3, max(sl[1].start - 3, 0):sl[1].stop + 3] = False

    def value(s, px, py):
        x0, y0, x1, y1 = s.box
        return (s.xlim[0] + (px / (x1 - x0)) * (s.xlim[1] - s.xlim[0]),
                s.ylim[1] - (py / (y1 - y0)) * (s.ylim[1] - s.ylim[0]))

    def colour(s, kind):
        r, g, b = s.crop[..., 0], s.crop[..., 1], s.crop[..., 2]
        m = dict(blue=(b > 150) & (r < 110) & (g < 110), red=(r > 170) & (g < 120) & (b < 120),
                 black=(r < 90) & (g < 90) & (b < 90))[kind]
        return m & s.mask

    def dots(s):
        """blue dots: compact blobs. Text and arrows in blue aren't compact"""
        lab, n = ndi.label(s.colour('blue'))
        out = []
        for i, sl in enumerate(ndi.find_objects(lab)):
            h, w = sl[0].stop - sl[0].start, sl[1].stop - sl[1].start
            a = (lab[sl] == i + 1).sum()
            if h >= 4 and w >= 4 and 0.65 < h / w < 1.5 and a / (h * w) > 0.6:
                cy, cx = ndi.center_of_mass(lab == i + 1)
                out.append((cx, cy))
        return out

    def template(s, kind, arm):
        """the score of a plus or a cross of that arm at each pixel, as a fraction of its length"""
        n = 2 * arm + 1
        t = np.zeros((n, n))
        i = np.arange(n)
        if kind == 'plus':
            t[arm, :] = 1
            t[:, arm] = 1
            m = s.colour('red')
        else:
            t[i, i] = 1
            t[i, n - 1 - i] = 1
            m = s.colour('black')
        m = ndi.binary_dilation(m, iterations=1).astype(float)
        return fftconvolve(m, t[::-1, ::-1], mode='same') / t.sum()

    def peaks(s, kind, arm, thresh, sep=None):
        sc = s.template(kind, arm)
        sep = sep or arm
        mx = ndi.maximum_filter(sc, size=2 * sep + 1)
        ys, xs = np.where((sc == mx) & (sc >= thresh))
        pts = sorted(zip(xs.tolist(), ys.tolist(), sc[ys, xs].tolist()), key=lambda p: -p[2])
        keep = []
        for x, y, v in pts:
            if all(math.hypot(x - a, y - b) > sep for a, b, _ in keep):
                keep.append((x, y, v))
        return keep


def on_grid(p, pts, tol=0.8):
    """blue dots against KCAS: each must fall on a speed of NASA's grid. Returns {KCAS: y} and the worst miss"""
    out, worst = {}, 0.0
    for q in pts:
        x, y = p.value(q[0], q[1])
        k = min(KCAS, key=lambda v: abs(v - x))
        worst = max(worst, abs(k - x))
        if abs(k - x) <= tol and k not in out:
            out[k] = float(y)
    return out, worst


def in_column(p, kind, arm, speeds, floor=0.45):
    """Markers often overlap, but their speeds are known: at each speed's column, the height where the shape fits
    best. Returns {KCAS: y}, and the fit, a fraction of the shape found; below the floor nothing is taken."""
    sc = p.template(kind, arm)
    x0, _, x1, _ = p.box
    out, fit = {}, {}
    for k in speeds:
        px = int(round((k - p.xlim[0]) / (p.xlim[1] - p.xlim[0]) * (x1 - x0)))
        band = sc[:, max(px - 2, 0):px + 3]
        y, c = np.unravel_index(np.argmax(band), band.shape)
        fit[k] = round(float(band[y, c]), 2)
        if band[y, c] >= floor:
            out[k] = float(p.value(px, y)[1])
    return out, fit


def speed_plot(img, box, xlim, ylim, arm):
    p = Plot(img, box, xlim, ylim)
    p.hide_legend()
    d, worst = on_grid(p, p.dots())
    r, fr = in_column(p, 'plus', arm, KCAS)
    b, fb = in_column(p, 'cross', arm, KCAS[:4])
    return dict(mod3=d, mod4_fixed=r, mod4_airspeed=b), dict(worst_kcas=round(float(worst), 2), plus=fr, cross=fb)


def whole_markers(p, kind, arm):
    """Markers that overlap hide each other's arms, so an arm may be any marker's colour, but enough of the shape must
    be this marker's own. The whole shape has to be there: a part of one, or a line, doesn't fit."""
    n = 2 * arm + 1
    t = np.zeros((n, n))
    i = np.arange(n)
    if kind == 'plus':
        t[arm, :] = t[:, arm] = 1
        own = p.colour('red')
    else:
        t[i, i] = t[i, n - 1 - i] = 1
        own = p.colour('black')
    fat = lambda m: ndi.binary_dilation(m, iterations=1).astype(float)
    conv = lambda m: fftconvolve(fat(m), t[::-1, ::-1], mode='same') / t.sum()
    every = conv(p.colour('red') | p.colour('blue') | p.colour('black'))
    mine = conv(own)
    sc = np.where((every >= 0.9) & (mine >= 0.35), every + 0.1 * mine, 0)
    mx = ndi.maximum_filter(sc, size=9)
    ys, xs = np.where((sc == mx) & (sc > 0))
    pts = sorted(zip(xs.tolist(), ys.tolist(), sc[ys, xs].tolist()), key=lambda q: -q[2])
    keep = []
    for x, y, v in pts:
        if all(math.hypot(x - a, y - b) > 4 for a, b, _ in keep):
            keep.append((x, y, v))
    return keep


def eig_plot(img, xlim, ylim, hide):
    """every marker on an eigenvalue plot. A plus spans 51 pixels here; a cross has the same reach along its diagonals"""
    p = Plot(img, lines(img), xlim, ylim)
    for h in hide:
        p.hide(*h)
    val = lambda pts: [tuple(round(float(v), 4) for v in p.value(q[0], q[1])) for q in pts]
    return dict(mod3=val(p.dots()), mod4_fixed=val(whole_markers(p, 'plus', 25)), mod4_airspeed=val(whole_markers(p, 'cross', 18)))


def assign(points, damping, rising, near, tol=0.012):
    """Eigenvalues to speeds: the frequency runs one way with speed (rising or falling), and each must show the damping
    the speed's own plot gives. Damping alone can't catch a stray point, since it is the same all along a radial line,
    so a candidate must also lie near the Mod III frequencies. Speeds with no such point are left out, not guessed."""
    import itertools
    speeds = sorted(damping)
    pts = [(q[0], q[1], zeta(q[0], q[1])) for q in points if near[0] <= math.hypot(q[0], q[1]) <= near[1]]
    best, cost = {}, None
    ordered = sorted(pts, key=lambda q: math.hypot(q[0], q[1]), reverse=not rising)
    for n in range(len(speeds), 0, -1):
        for sp in itertools.combinations(speeds, n):
            for combo in (itertools.permutations(ordered, n) if rising is None else itertools.combinations(ordered, n)):
                e = [abs(c[2] - damping[k]) for c, k in zip(combo, sp)]
                if max(e) <= tol and (cost is None or sum(e) < cost):
                    best, cost = dict(zip(sp, combo)), sum(e)
        if best:
            break
    return best


def zeta(re, im):
    return -re / math.hypot(re, im)


def main():
    I = images()
    res, fit = {}, {}
    res['zeta_phugoid'], fit['fig19'] = speed_plot(I['fig19'], frames(I['fig19'])[0], (60, 135), (0, 0.35), 15)
    res['zeta_short_period'], fit['fig21'] = speed_plot(I['fig21'], frames(I['fig21'])[-1], (60, 135), (0.25, 0.5), 13)
    res['tau_roll_s'], fit['fig22'] = speed_plot(I['fig22'], frames(I['fig22'])[0], (60, 135), (0.5, 1.6), 15)
    res['zeta_dutch_roll'], fit['fig24'] = speed_plot(I['fig24'], frames(I['fig24'])[0], (60, 135), (0.02, 0.25), 15)

    raw = dict(phugoid=eig_plot(I['fig18'], (-0.2, 0), (0, 0.4), [(0, 0, .42, .3)]),
               short_period=eig_plot(I['fig20'], (-5, 0), (0, 5), [(0, 0, .42, .3)]),
               dutch_roll=eig_plot(I['fig23'], (-1.6, 0), (0, 1.6), [(0, 0, .42, .3)]))
    # frequency against speed [the figures' arrows]: short period and Dutch roll rising, Mod III's phugoid falling.
    # Mod IV's phugoid bends back on itself (fig. 18), so there only the damping places it (rising None); its two
    # highest speeds have almost the same damping, so which is which is uncertain
    eig = {}
    for mode, zk, rising in (('phugoid', 'zeta_phugoid', False), ('short_period', 'zeta_short_period', True),
                             ('dutch_roll', 'zeta_dutch_roll', True)):
        w = [math.hypot(*q) for q in raw[mode]['mod3']]
        near = (0.75 * min(w), 1.25 * max(w))
        eig[mode] = {}
        for s in ('mod3', 'mod4_fixed', 'mod4_airspeed'):
            a = assign(raw[mode][s], res[zk][s], None if mode == 'phugoid' and s != 'mod3' else rising, near)
            eig[mode][s] = {str(k): dict(re=v[0], im=v[1], zeta=round(v[2], 4), omega=round(math.hypot(v[0], v[1]), 4))
                            for k, v in sorted(a.items())}
    out = dict(source='NTRS 20240006496, figs. 18-24; takeoff configuration; NASA simulator predictions, not measurements',
               units=dict(eigenvalues='rad/s (the figures label the imaginary axis Hz)', tau_roll_s='s'),
               vs_kcas={k: {s: {str(a): round(b, 4) for a, b in sorted(v.items())} for s, v in d.items()} for k, d in res.items()},
               eigenvalues=eig)
    os.makedirs(os.path.join(HERE, 'data', 'digitised'), exist_ok=True)
    with open(os.path.join(HERE, 'data', 'digitised', 'flying_qualities.json'), 'w') as fo:
        json.dump(out, fo, indent=1)

    for k, d in out['vs_kcas'].items():
        print(k)
        for s, v in d.items():
            print('   %-14s %s' % (s, v))
    print('fit of each marker (fraction of its shape found):')
    for k, v in fit.items():
        print('  ', k, v)
    print('\neigenvalues kept: each one agrees with its speed\'s damping')
    for k, d in eig.items():
        print(k)
        for s, v in d.items():
            print('   %-14s %s' % (s, {a: (b['re'], b['im'], b['zeta'], b['omega']) for a, b in v.items()}))


if __name__ == '__main__':
    main()
