#!/usr/bin/env python3
"""Digitise the wind tunnel's rate derivatives [TM-20250001715, figs. 58-62, flaps 0 panels] into
data/digitised/tunnel_rates.json.

    python3 digitise_tunnel.py

NASA forced-oscillated a 19 percent scale model in the Langley 12-ft tunnel, flaps up, at a chord Reynolds number of
150,000, and the TM plots the result as a dashed black line over the CFD. It is the only measured dynamic data for this
airframe, uncorrected for installation effects [TM 5.2.1]. The line is black and everything else on the plot coloured or
grey, so each column's black pixels give it; the gaps between dashes are interpolated. The axes are calibrated from each
figure's tick marks, and the legend is left out.
"""
import json, os, subprocess
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
PDF = os.path.join(HERE, '..', 'refs', 'NASA-TM-20250001715-aero-database.pdf')
TMP = os.path.join(HERE, 'out', 'tm')
# figure: image, name, (pixel of alpha -5, of alpha 15), (pixel row, value) twice, legend box (x0, y0, x1, y1)
FIGS = ((1, 'Cmq', (79, 406), ((24, 0.0), (385, -300.0)), (88, 32, 246, 124)),
        (2, 'Clp', (75, 402), ((24, 0.0), (385, -1.2)), (83, 32, 241, 124)),
        (3, 'Cnp', (74, 402), ((24, 0.1), (386, -0.5)), (83, 286, 241, 378)),
        (4, 'Cnr', (75, 402), ((24, 0.0), (386, -0.8)), (83, 33, 241, 124)),
        (5, 'Clr', (68, 397), ((24, 1.5), (388, 0.0)), (76, 32, 236, 124)))
ALPHA = np.arange(-4.0, 19.0, 1.0)


def line(img, xs, ys, legend):
    a = np.asarray(img).astype(int)
    black = (a.max(axis=2) < 70)
    x0, y0, x1, y1 = legend
    black[y0:y1, x0:x1] = False
    px = lambda al: xs[0] + (al + 5) / 20 * (xs[1] - xs[0])
    (r0, v0), (r1, v1) = ys
    cols, vals = [], []
    for c in range(int(px(-5)) + 3, min(int(px(20)) - 3, 475)):
        rows = np.where(black[r0 + 3:r1 - 3, c])[0]
        if len(rows) and rows.max() - rows.min() < 12:
            r = rows.mean() + r0 + 3
            cols.append(-5 + (c - xs[0]) / (xs[1] - xs[0]) * 20)
            vals.append(v0 + (r - r0) / (r1 - r0) * (v1 - v0))
    return np.array(cols), np.array(vals)


def main():
    os.makedirs(TMP, exist_ok=True)
    subprocess.run(['pdfimages', '-png', '-f', '61', '-l', '62', PDF, os.path.join(TMP, 't')], check=True)
    out = dict(source='TM-20250001715 figs. 58-62: Langley 12-ft tunnel, 19 % scale, flaps 0, uncorrected for installation',
               alpha=ALPHA.tolist())
    for i, name, xs, ys, leg in FIGS:
        img = Image.open(os.path.join(TMP, 't-%03d.png' % i)).convert('RGB')
        c, v = line(img, xs, ys, leg)
        o = np.argsort(c)
        # a light smoothing over each degree: the line is a few pixels thick
        s = [float(np.median(v[o][np.abs(c[o] - a) < 0.6])) if (np.abs(c[o] - a) < 0.6).any() else np.nan for a in ALPHA]
        s = np.array(s)
        ok = np.isfinite(s)
        out[name] = [round(float(x), 4) for x in np.interp(ALPHA, ALPHA[ok], s[ok])]
        print('%-4s %d columns; %s' % (name, len(c), ' '.join('%g:%.3g' % (a, x) for a, x in zip(ALPHA[::3], out[name][::3]))))
    with open(os.path.join(HERE, 'data', 'digitised', 'tunnel_rates.json'), 'w') as f:
        json.dump(out, f, indent=1)


if __name__ == '__main__':
    main()
