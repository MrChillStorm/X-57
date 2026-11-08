#!/usr/bin/env python3
"""Digitise the 2022 tunnel retest of the high-lift propeller [NTRS 20220006506, slides 6 and 7] into data/digitised/hlp_tunnel.json.

    python3 digitise_hlp.py

Full-scale propellers in the Langley LSAWT, new blades with tight tolerances: right-handed (red plus), left-handed (blue
circle), the 2020 right-handed set (magenta triangle, left out: its blades were soft), and NASA's blade-element model (black
cross). The markers are found by colour. The black crosses should fall on NASA's own tables, which tests this reading.
"""
import json, os, subprocess
import numpy as np
from PIL import Image
from scipy import ndimage as ndi

HERE = os.path.dirname(os.path.abspath(__file__))
PDF = os.path.join(HERE, '..', 'refs', '20220006506-HLP-tunnel-retest.pdf')
TMP = os.path.join(HERE, 'out', 'hlp')
DPI = 300


def page(p):
    os.makedirs(TMP, exist_ok=True)
    base = os.path.join(TMP, 'p%d' % p)
    subprocess.run(['pdftoppm', '-r', str(DPI), '-f', str(p), '-l', str(p), '-png', '-singlefile', PDF, base], check=True)
    return np.asarray(Image.open(base + '.png').convert('RGB')).astype(int)


def frame(a):
    """the plot's left and bottom axes: the longest dark lines in the plot's part of the slide"""
    H, W, _ = a.shape
    dark = a.sum(axis=2) < 330
    win = dark[int(H * .3):int(H * .95), int(W * .1):int(W * .8)]
    y = int(np.argmax(win.sum(axis=1))) + int(H * .3)
    x = int(np.argmax(win.sum(axis=0))) + int(W * .1)
    def run(line, at, gap=30):
        """the stretch of the line through at, over gaps of a few pixels where a marker or a label breaks it"""
        idx = np.where(line)[0]
        cut = np.where(np.diff(idx) > gap)[0]
        for part in np.split(idx, cut + 1):
            if part[0] <= at <= part[-1]:
                return int(part[0]), int(part[-1])
        return at, at

    # (the bottom axis runs on a little to the left, into the zero's tick)
    a, b = run(dark[y - 1:y + 2].any(axis=0), x + 5)
    c, d = run(dark[:, x - 1:x + 2].any(axis=1), y - 5)
    return x, b, c, d          # x at J 0.5, x at J 1.4, y at the top value, y at 0


def marks(mask, lo=6, hi=60, square=None):
    """the middles of a marker colour's blobs; square: about square as well, which leaves black text and arrows out"""
    lab, n = ndi.label(ndi.binary_dilation(mask, iterations=1))
    out = []
    for i, sl in enumerate(ndi.find_objects(lab)):
        h, w = sl[0].stop - sl[0].start, sl[1].stop - sl[1].start
        if h < lo or w < lo or max(h, w) > hi or (square and abs(h - w) > square):
            continue
        out.append(((sl[1].start + sl[1].stop - 1) / 2, (sl[0].start + sl[0].stop - 1) / 2))
    return out


def read(p, top):
    a = page(p)
    x0, x1, y1, y0 = frame(a)
    r, g, b = a[..., 0], a[..., 1], a[..., 2]
    box = np.zeros(a.shape[:2], bool)
    box[y1 + 2:y0 - 2, x0 + 2:x1 - 2] = True
    # the legend, at the plot's lower left
    W, H = x1 - x0, y0 - y1
    box[y0 - int(H * .34):y0, x0:x0 + int(W * .17)] = False
    if top > 0.1:              # the thrust slide's note, upper right
        box[y1:y1 + int(H * .45), x0 + int(W * .68):x1] = False
    conv = lambda m: [(round(0.5 + (u - x0) / (x1 - x0) * 0.9, 4), round((y0 - v) / (y0 - y1) * top, 5)) for u, v in m]
    red = (r > 190) & (g < 90) & (b < 90) & box
    blue = (b > 180) & (r < 90) & (g < 110) & box
    black = (r < 70) & (g < 70) & (b < 70) & box
    return dict(frame=[int(x0), int(x1), int(y1), int(y0)], RH=sorted(conv(marks(red))), LH=sorted(conv(marks(blue))), BEMT=sorted(conv(marks(black, 10, 34, 7))))


def main():
    q, t = read(6, 0.05), read(7, 0.30)
    out = dict(source='NTRS 20220006506 slides 6, 7: 2022 LSAWT test, new blades', Cq=q, Ct=t)
    with open(os.path.join(HERE, 'data', 'digitised', 'hlp_tunnel.json'), 'w') as f:
        json.dump(out, f, indent=0)
    return out


if __name__ == '__main__':
    o = main()
    hl = json.load(open(os.path.join(HERE, 'data', 'hlp_tables.json')))
    P = sorted(hl['airspeed'] + hl['fixed'], key=lambda p: p['J'])
    J = np.array([p['J'] for p in P])
    fit = {c: np.polyfit(J[(J > .5) & (J < 1.45)], np.array([p[c] for p in P])[(J > .5) & (J < 1.45)], 4) for c in ('Ct', 'Cq')}
    for c in ('Ct', 'Cq'):
        d = o[c]
        print(c, 'frame', d['frame'], 'points RH %d LH %d BEMT %d' % (len(d['RH']), len(d['LH']), len(d['BEMT'])))
        for k in ('BEMT', 'LH', 'RH'):
            x = np.array(d[k])
            if len(x):
                e = x[:, 1] - np.polyval(fit[c], x[:, 0])
                print('   %-5s J %.2f-%.2f; against NASA\'s tables: mean %+.4f, rms %.4f' % (k, x[:, 0].min(), x[:, 0].max(), e.mean(), np.sqrt((e ** 2).mean())))
    print('   J:      ', ' '.join('%6.2f' % j for j in (.6, .7, .8, .9, 1.0, 1.1, 1.2, 1.3)))
    print('   Ct NASA:', ' '.join('%6.3f' % np.polyval(fit['Ct'], j) for j in (.6, .7, .8, .9, 1.0, 1.1, 1.2, 1.3)))
    print('   Cq NASA:', ' '.join('%6.4f' % np.polyval(fit['Cq'], j) for j in (.6, .7, .8, .9, 1.0, 1.1, 1.2, 1.3)))
