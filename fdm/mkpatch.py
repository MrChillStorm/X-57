#!/usr/bin/env python3
"""The pilot's chest patch: an embroidered round patch, drawn here, into data/pilot/patch.png, which x57pilot.py puts on his chest.

    python3 mkpatch.py [preview.png]          the preview at twice the texture's size, on the suit's colour

FlightGear's checkerboard runs round it, and its logo, the checker strip and the blue F of fgdata's Docs/img/FlightGear_logo.svg (GPL,
as fgdata), sits under the tail. The aircraft is Mod IV from above, to scale, from NASA's OpenVSP geometry in data/geometry.json: the
wing's root and tip chords on its straight 70 % chord line, the fuselage's stations, the stabilator, the tip nacelles and all fourteen
propellers, drawn as their discs' blur. A wave runs behind it, an electric field and a magnetic one, for the aircraft's name. Lettering
in DejaVu Sans Bold, as matplotlib ships it. No NASA name, logo or insignia.
"""
import json, math, os, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont
import matplotlib
from scipy.interpolate import PchipInterpolator

HERE = os.path.dirname(os.path.abspath(__file__))
GEO = json.load(open(os.path.join(HERE, 'data', 'geometry.json')))
OUT = os.path.join(HERE, 'data', 'pilot', 'patch.png')
TEX = 512                     # 54 texels a centimetre on a 9.5 cm patch
FONT = os.path.join(matplotlib.get_data_path(), 'fonts', 'ttf', 'DejaVuSans-Bold.ttf')

N = 2048                      # drawn at four times the texture's size
C = N / 2
SS = 2
FG_YELLOW, FG_BLUE, BLACK = (251, 252, 70), (66, 126, 191), (17, 17, 17)
GOLD, GOLD_DARK = (201, 162, 39), (140, 108, 18)
NAVY, SKY_IN, SKY_OUT = (13, 31, 60), (44, 96, 156), (18, 50, 92)
WHITE, OUTLINE, CYAN = (242, 244, 247), (120, 132, 146), (127, 224, 255)

R_OUT, R_CHK_O, R_CHK_I, R_TXT_O, R_TXT_I = 1000, 962, 862, 850, 700


def polar():
    y, x = np.mgrid[0:N, 0:N] + 0.5
    return np.hypot(x - C, y - C), np.arctan2(x - C, -(y - C)) % (2 * math.pi)       # angle from the top, clockwise


def stitches(img, period=7.0, amp=0.045, angle=35.0):
    """the satin stitch's sheen: fine diagonal bands"""
    y, x = np.mgrid[0:N, 0:N]
    a = math.radians(angle)
    s = 1 + amp * np.sin(2 * math.pi * (x * math.cos(a) + y * math.sin(a)) / period)
    arr = np.asarray(img).astype(np.float32)
    arr[..., :3] *= s[..., None]
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8), 'RGBA')


def ring_layers():
    r, th = polar()
    img = np.zeros((N, N, 4), np.uint8)
    # the merrowed edge, gold
    edge = (r <= R_OUT) & (r > R_CHK_O)
    img[edge] = GOLD + (255,)
    twist = np.sin((th * 260) + (r - R_CHK_O) * 0.25) > 0.55                          # the overlock's slanted stitches
    img[edge & twist] = GOLD_DARK + (255,)
    # FlightGear's checkerboard, two rows of squares round the ring
    band = (r <= R_CHK_O) & (r > R_CHK_I)
    rows = 2
    h = (R_CHK_O - R_CHK_I) / rows
    n = int(round(2 * math.pi * (R_CHK_O + R_CHK_I) / 2 / h / 2)) * 2
    row = np.floor((R_CHK_O - r) / h).astype(int)
    col = np.floor(th / (2 * math.pi) * n).astype(int)
    yellow = ((row + col) % 2) == 0
    img[band & yellow] = FG_YELLOW + (255,)
    img[band & ~yellow] = BLACK + (255,)
    # a thin gold line, the lettering's navy band, and the sky
    img[(r <= R_CHK_I) & (r > R_TXT_O)] = GOLD + (255,)
    img[(r <= R_TXT_O) & (r > R_TXT_I)] = NAVY + (255,)
    sky = r <= R_TXT_I
    t = np.clip(r / R_TXT_I, 0, 1)[..., None]
    img[sky, :3] = (np.array(SKY_IN) * (1 - t) + np.array(SKY_OUT) * t)[sky].astype(np.uint8)
    img[sky, 3] = 255
    img[(r <= R_TXT_I) & (r > R_TXT_I - 8)] = GOLD + (255,)
    return Image.fromarray(img, 'RGBA')


def arc_text(img, text, radius, size, fill, top=True, track=0.06):
    """letters round the circle, their feet to the centre on top and their heads to it below, reading left to right"""
    font = ImageFont.truetype(FONT, size)
    widths = [font.getlength(ch) * (1 + track) for ch in text]
    total = sum(widths) / radius
    a = -total / 2
    for ch, w in zip(text, widths):
        mid = a + w / 2 / radius
        a += w / radius
        if ch == ' ':
            continue
        box = font.getbbox(ch)
        tile = Image.new('RGBA', (int(box[2] + 40), int(size * 1.6)), (0, 0, 0, 0))
        ImageDraw.Draw(tile).text((20, size * 0.2), ch, font=font, fill=fill + (255,))
        if top:
            x, y, rot = C + radius * math.sin(mid), C - radius * math.cos(mid), -math.degrees(mid)
        else:
            x, y, rot = C + radius * math.sin(mid), C + radius * math.cos(mid), math.degrees(mid)
        tile = tile.rotate(rot, resample=Image.BICUBIC, expand=True)
        img.alpha_composite(tile, (int(x - tile.width / 2), int(y - tile.height / 2)))


def wave(img):
    """an electromagnetic wave behind the aircraft: the electric field in yellow, the magnetic a quarter turn on in cyan"""
    d = ImageDraw.Draw(img)
    y0, amp, lam = C + 250, 78, 330
    for col, phase, wdt, a in ((CYAN, math.pi / 2, 10, 150), (FG_YELLOW, 0.0, 14, 210)):
        pts = []
        for x in range(int(C - R_TXT_I), int(C + R_TXT_I) + 1, 4):
            if math.hypot(x - C, y0 - C) < R_TXT_I - 10:
                pts.append((x, y0 + amp * math.sin(2 * math.pi * (x - C) / lam + phase)))
        layer = Image.new('RGBA', (N, N), (0, 0, 0, 0))
        ImageDraw.Draw(layer).line(pts, fill=col + (a,), width=wdt, joint='curve')
        img.alpha_composite(layer)


def aircraft(img, s=2.7, wing_y=C - 150):
    """Mod IV from above, nose up, inches to pixels at s; the wing's root leading edge at wing_y. A high wing: it covers the cabin's roof,
    and the windshield shows ahead of it; the nacelles run under it and show ahead of its leading edge"""
    w = GEO['wing']
    x0 = w['origin_in'][0]
    p = w['panels'][0]
    half, c0, c1, loc = p['y1'], p['c0'], p['c1'], p['loc']
    xl = x0 + loc * c0                                    # the straight chord line
    P = lambda x, y: (C + y * s, wing_y + (x - x0) * s)
    d = ImageDraw.Draw(img)
    ow = 3 * SS
    # propellers first, under all: each disc seen edge-on from above, a blurred bar of its diameter
    blur = Image.new('RGBA', (N, N), (0, 0, 0, 0))
    bd = ImageDraw.Draw(blur)
    hubs = [(q[0], q[1], GEO['high_lift']['diameter_in']) for q in GEO['high_lift']['hubs_in']]
    hubs += [(GEO['cruise']['hub_in'][0] - 11.0, GEO['cruise']['hub_in'][1], GEO['cruise']['diameter_in'])]
    for hx, hy, D in hubs:
        for sy in (-1, 1):
            cx, cy = P(hx - 6.0, sy * hy)
            rx, ry = D / 2 * s, max(D / 2 * s * 0.2, 14)
            bd.ellipse((cx - rx, cy - ry, cx + rx, cy + ry), fill=CYAN + (175,))
            bd.ellipse((cx - rx * 0.94, cy - ry * 0.38, cx + rx * 0.94, cy + ry * 0.38), fill=(240, 253, 255, 245))
    img.alpha_composite(blur.filter(ImageFilter.GaussianBlur(4)))
    # nacelles: the tips' and the twelve high lift's, their backs under the wing
    def capsule(xa, xb, y, dia, fill):
        a, b = P(xa, y), P(xb, y)
        r = dia / 2 * s
        d.rounded_rectangle((a[0] - r, a[1], b[0] + r, b[1]), radius=r, fill=fill, outline=OUTLINE, width=ow)
    for sy in (-1, 1):
        capsule(126.4, 166.0, sy * GEO['cruise']['hub_in'][1], 13.8, (226, 230, 236))
        for q in GEO['high_lift']['hubs_in']:
            capsule(q[0] - 8.0, 168.0, sy * q[1], 6.5, (214, 219, 226))
    # the fuselage, its stations smoothed, and its fin seen edge-on
    fs = GEO['fuselage']
    fx = np.array([q['x'] for q in fs])
    fw = np.array([q['w'] for q in fs]) / 2
    width = PchipInterpolator(fx, fw)
    xx = np.linspace(fx[0], fx[-1], 120)
    ww = np.clip(width(xx), 0, None)
    # the stabilator, under the fuselage's end
    st = GEO['stabilator']
    sx, sh, sc = st['le_in'][0], st['half_span_in'], st['chord_in']
    d.polygon([P(sx, -sh), P(sx, sh), P(sx + sc, sh * 0.96), P(sx + sc, -sh * 0.96)], fill=WHITE, outline=OUTLINE, width=ow)
    d.polygon([P(x, -y) for x, y in zip(xx, ww)] + [P(x, y) for x, y in zip(xx[::-1], ww[::-1])], fill=WHITE, outline=OUTLINE, width=ow)
    a, b = P(292.0, 0), P(352.0, 0)
    d.rounded_rectangle((a[0] - 3.2 * s, a[1], b[0] + 3.2 * s, b[1]), radius=3.2 * s, fill=(222, 226, 232), outline=OUTLINE, width=ow)
    # the windshield, ahead of the wing
    gx = np.linspace(96.0, 150.0, 40)
    gw = width(gx) * np.clip((gx - 96.0) / 22.0, 0, 1) ** 0.5 * 0.78
    d.polygon([P(x, -y) for x, y in zip(gx, gw)] + [P(x, y) for x, y in zip(gx[::-1], gw[::-1])], fill=(64, 92, 126))
    # the wing, straight-tapered on its 70 % chord line, over all
    chord = lambda y: c0 + (c1 - c0) * abs(y) / half
    le = lambda y: xl - loc * chord(y)
    te = lambda y: xl + (1 - loc) * chord(y)
    ys = np.linspace(-half, half, 3)
    d.polygon([P(le(y), y) for y in ys] + [P(te(y), y) for y in ys[::-1]], fill=WHITE, outline=OUTLINE, width=ow)
    # the flaps' and ailerons' hinge lines, faint
    for ya, yb, f in ((24.14, 136.61, 0.75), (136.61, 181.97, 0.75)):
        for sy in (-1, 1):
            d.line([P(le(ya) + f * chord(ya), sy * ya), P(le(yb) + f * chord(yb), sy * yb)], fill=(196, 202, 210), width=2 * SS)
    # the cruise spinners' red, as the livery's
    for sy in (-1, 1):
        cx, cy = P(124.0, sy * GEO['cruise']['hub_in'][1])
        d.ellipse((cx - 14, cy - 14, cx + 14, cy + 14), fill=(206, 38, 38))
    return P(357.7, 0)[1]                                  # the tail's end


def fg_logo(img, cx, cy, size):
    """fgdata's FlightGear logo, its 200-unit drawing: the black and yellow checker strip and the blue F, on a white tile"""
    k = size / 200.0
    tile = Image.new('RGBA', (int(size + 24), int(size + 24)), (0, 0, 0, 0))
    d = ImageDraw.Draw(tile)
    o = 12
    d.rounded_rectangle((0, 0, tile.width - 1, tile.height - 1), radius=26, fill=GOLD + (255,))
    d.rounded_rectangle((o - 2, o - 2, o + size + 2, o + size + 2), radius=18, fill=(255, 255, 255, 255))
    Q = lambda x, y: (o + x * k, o + y * k)
    d.rectangle((*Q(0, 0), *Q(75, 200)), fill=BLACK)
    for col in range(3):
        for row in range(8):
            if (row + col) % 2 == 0:
                d.rectangle((*Q(25 * col, 25 * row), *Q(25 * col + 25, 25 * row + 25)), fill=FG_YELLOW)
    for a, b in (((94.418, 27.531), (129.606, 172.47)), ((94.418, 27.531), (188.54, 54.56)), ((129.6, 82.53), (184.51, 109.47))):
        d.rounded_rectangle((*Q(*a), *Q(*b)), radius=3 * k, fill=FG_BLUE)
    mask = Image.new('L', tile.size, 0)
    ImageDraw.Draw(mask).rounded_rectangle((0, 0, tile.width - 1, tile.height - 1), radius=26, fill=255)
    tile.putalpha(Image.fromarray(np.minimum(np.asarray(tile)[..., 3], np.asarray(mask))))
    img.alpha_composite(tile, (int(cx - tile.width / 2), int(cy - tile.height / 2)))


def main(preview=None):
    img = ring_layers()
    sky = Image.new('RGBA', (N, N), (0, 0, 0, 0))
    wave(sky)
    tail = aircraft(sky)
    fg_logo(sky, C, tail + 120, 180)
    r, _ = polar()
    clip = Image.fromarray(((r <= R_TXT_I - 8) * 255).astype(np.uint8), 'L')
    sky.putalpha(Image.fromarray(np.minimum(np.asarray(sky)[..., 3], np.asarray(clip))))
    img.alpha_composite(sky)
    arc_text(img, 'X-57 MAXWELL', (R_TXT_O + R_TXT_I) / 2 - 2, 104, WHITE, top=True)
    arc_text(img, 'FLIGHTGEAR', (R_TXT_O + R_TXT_I) / 2 + 2, 98, FG_YELLOW, top=False)
    img = stitches(img)
    a = np.asarray(img).copy()
    a[..., 3] = np.where(polar()[0] <= R_OUT, a[..., 3], 0)
    full = Image.fromarray(a, 'RGBA')
    img = full.resize((TEX, TEX), Image.LANCZOS)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    img.save(OUT, optimize=True)
    print('wrote', os.path.relpath(OUT, os.path.join(HERE, '..')), img.size)
    if preview:
        big = full.resize((2 * TEX, 2 * TEX), Image.LANCZOS)
        bg = Image.new('RGBA', big.size, (96, 104, 88, 255))      # on the suit's sage
        bg.alpha_composite(big)
        bg.convert('RGB').save(preview)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else None)
