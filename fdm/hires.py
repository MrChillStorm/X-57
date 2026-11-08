"""High-resolution faces for the Tecnam P2006T's instruments: the same models, their pictures redrawn at 2048 pixels in the same places on the texture.

Each painter draws one texture over the original resampled to full size, so whatever isn't redrawn (bezels, corners, knobs) stays. Needles sample a white patch rather than the picture, so
a new face can't change their colour.

    paint(base)      base: Models/Interior/Panel of the package
"""
import math, os, re
from PIL import Image, ImageDraw, ImageFont

S = 2048
W = (255, 255, 255)
INK = (14, 14, 16)


def font(size):
    return ImageFont.load_default(size=size)


def atlas(src, w=S, h=S):
    return Image.open(src).convert('RGB').resize((w, h), Image.LANCZOS)


def uv(u, v, w=S, h=S):
    return u * w, (1 - v) * h


def text(g, x, y, s, size, fill=W):
    f = font(size)
    g.text((x - g.textlength(s, font=f) / 2, y - size * 0.6), s, font=f, fill=fill)


def tick(g, cx, cy, deg, r0, r1, width, fill=W):
    a = math.radians(deg)
    g.line((cx + r0 * math.sin(a), cy - r0 * math.cos(a), cx + r1 * math.sin(a), cy - r1 * math.cos(a)), fill=fill, width=width)


def polar(cx, cy, deg, r):
    a = math.radians(deg)
    return cx + r * math.sin(a), cy - r * math.cos(a)


def white_patch(im, u=0.995, v=0.005):
    x, y = uv(u, v, *im.size)
    ImageDraw.Draw(im).rectangle((x - 14, y - 14, x + 14, y + 14), fill=W)


def flat_uv(path, names, u=0.995, v=0.005):
    """every corner of these objects on one texel"""
    s = open(path, errors='replace').read()
    out = []
    for o in re.split(r'(?=^OBJECT )', s, flags=re.M):
        m = re.search(r'^name "([^"]+)"', o, re.M)
        if m and m.group(1) in names:
            o = re.sub(r'^(\d+) [-\d.e]+ [-\d.e]+[ \t]*$', lambda q: '%s %.4f %.4f' % (q.group(1), u, v), o, flags=re.M)
        out.append(o)
    open(path, 'w', errors='replace').write(''.join(out))


def interp(x, table):
    for (a, b), (c, d) in zip(table, table[1:]):
        if a <= x <= c:
            return b + (d - b) * (x - a) / (c - a)
    return table[-1][1]


ASI = ((0, 0), (40, 28), (60, 68), (80, 120), (100, 170), (120, 205), (140, 238), (160, 269), (180, 305), (200, 340))
VSI = ((-6000, 170), (-4000, 139), (-2000, 83), (-1000, 48), (0, 0), (1000, -48), (2000, -83), (4000, -139), (6000, -170))


def asi(d):
    p = os.path.join(d, 'asi', 'asi.png')
    im = atlas(p)
    g = ImageDraw.Draw(im)
    cx, cy = uv(0.506, 0.505)
    R = 0.494 * S
    g.ellipse((cx - R, cy - R, cx + R, cy + R), fill=INK)
    # the stock dial, as the real panel's in 20240010539 fig. 4: the P2006T's flight manual's markings
    for k0, k1, col, r_, w_ in ((53, 93, (235, 235, 230), 0.865, 22), (66, 135, (30, 170, 70), 0.93, 44), (135, 167, (240, 200, 40), 0.93, 44)):
        for k in range(k0 * 4, k1 * 4):
            a = interp(k / 4, ASI) - 90
            g.arc((cx - r_ * R, cy - r_ * R, cx + r_ * R, cy + r_ * R), a, a + 2, fill=col, width=w_)
    tick(g, cx, cy, interp(167, ASI), 0.80 * R, 0.97 * R, 22, (225, 30, 30))
    tick(g, cx, cy, interp(62, ASI), 0.80 * R, 0.97 * R, 16, (225, 30, 30))       # minimum control speed
    tick(g, cx, cy, interp(80, ASI), 0.80 * R, 0.97 * R, 16, (50, 110, 235))      # best single-engine climb
    for k in range(40, 201, 5):
        a = interp(k, ASI)
        major = k % 20 == 0
        tick(g, cx, cy, a, (0.76 if major else 0.84 if k % 10 == 0 else 0.87) * R, 0.91 * R, 18 if major else 9)
        if major:
            x, y = polar(cx, cy, a, 0.60 * R)
            text(g, x, y, '%d' % k, 150)
    text(g, cx, cy - 0.20 * R, 'AIRSPEED', 130)
    text(g, cx, cy + 0.02 * R, 'KNOTS', 92)
    white_patch(im)
    im.save(p)
    flat_uv(os.path.join(d, 'asi', 'asi.ac'), ('needle',))


def vsi(d):
    p = os.path.join(d, 'vsi', 'vsi.png')
    im = atlas(p)
    g = ImageDraw.Draw(im)
    cx, cy = S / 2, S / 2
    R = 0.471 * S
    g.ellipse((cx - R, cy - R, cx + R, cy + R), fill=INK)
    # 0 at nine o'clock, as the original's dial: 500 ft/min every 45 deg, to 2,000 at three o'clock
    ang = lambda f: -90 + f * 0.09
    for f in range(-2000, 2001, 100):
        major = f % 500 == 0
        tick(g, cx, cy, ang(f), (0.78 if major else 0.85) * R, 0.94 * R, 18 if major else 9)
    for f in (-1500, -1000, -500, 0, 500, 1000, 1500, 2000):
        x, y = polar(cx, cy, ang(f), 0.60 * R)
        text(g, x, y, '%d' % (abs(f) // 100), 150)
    text(g, cx + 40, cy - 0.10 * R, 'VERTICAL SPEED', 96)
    text(g, cx + 40, cy + 0.05 * R, '100 FEET PER MINUTE', 62)
    text(g, cx - 0.40 * R, cy - 0.21 * R, 'UP', 62)
    text(g, cx - 0.40 * R, cy + 0.21 * R, 'DOWN', 62)
    white_patch(im)
    im.save(p)
    flat_uv(os.path.join(d, 'vsi', 'vsi.ac'), ('needle',))
    # the needle to match the dial: the original's table puts 1,000 ft/min on the 5
    x = os.path.join(d, 'vsi', 'vsi.xml')
    s = open(x, errors='replace').read()
    tab = ''.join('<entry><ind>%d</ind><dep>%d</dep></entry>' % (f, -f * 0.09) for f in (-2000, -1000, 0, 1000, 2000))
    open(x, 'w', errors='replace').write(re.sub(r'<interpolation>.*?</interpolation>', '<interpolation>%s</interpolation>' % tab, s, count=1, flags=re.S))


def alt(d):
    p = os.path.join(d, 'alt', 'alt.png')
    im = atlas(p)
    g = ImageDraw.Draw(im)
    cx, cy = uv(0.750, 0.7525)
    R = 0.251 * S
    g.ellipse((cx - R, cy - R, cx + R, cy + R), fill=INK)
    for k in range(50):
        a = k * 7.2
        major = k % 5 == 0
        tick(g, cx, cy, a, (0.80 if major else 0.87) * R, 0.96 * R, 12 if major else 6)
    for n in range(10):
        x, y = polar(cx, cy, n * 36, 0.64 * R)
        text(g, x, y, str(n), 84)
    text(g, cx, cy - 0.27 * R, 'ALTITUDE', 44)
    text(g, cx, cy + 0.36 * R, '100 FEET', 34)
    im.save(p)


def hi(d):
    p = os.path.join(d, 'hi', 'hi.png')
    im = atlas(p)
    g = ImageDraw.Draw(im)
    cx, cy = uv(0.2525, 0.7495)
    R = 0.2005 * S
    g.ellipse((cx - R, cy - R, cx + R, cy + R), fill=INK)
    for k in range(0, 360, 5):
        major = k % 10 == 0
        tick(g, cx, cy, k, (0.86 if major else 0.91) * R, 0.985 * R, 7 if major else 4)
    card = Image.new('RGBA', (S, S), (0, 0, 0, 0))
    for k, s in ((0, 'N'), (30, '3'), (60, '6'), (90, 'E'), (120, '12'), (150, '15'), (180, 'S'), (210, '21'), (240, '24'), (270, 'W'), (300, '30'), (330, '33')):
        t = Image.new('RGBA', (220, 140), (0, 0, 0, 0))
        q = ImageDraw.Draw(t)
        f = font(74 if s in 'NESW' else 62)
        q.text((110 - q.textlength(s, font=f) / 2, 26), s, font=f, fill=W + (255,))
        t = t.rotate(-k, expand=True, resample=Image.BICUBIC)
        x, y = polar(cx, cy, k, 0.70 * R)
        card.alpha_composite(t, (int(x - t.size[0] / 2), int(y - t.size[1] / 2)))
    im.paste(card, (0, 0), card)
    im.save(p)


def turn(d):
    p = os.path.join(d, 'turn', 'turn.png')
    im = atlas(p)
    g = ImageDraw.Draw(im)
    cx, cy = S / 2, S / 2
    R = 0.445 * S
    g.ellipse((cx - R, cy - R, cx + R, cy + R), fill=(20, 20, 21))
    text(g, cx, cy - 0.76 * R, 'D.C. ELEC', 74)
    text(g, cx, cy - 0.48 * R, 'TURN COORDINATOR', 92)
    for s, x in ((-1, cx - 0.62 * R), (1, cx + 0.62 * R)):
        g.rectangle((x - 95, cy - 60, x + 95, cy - 30), fill=(235, 235, 235))
        g.line((x - 95, cy + 130 - s * 45, x + 95, cy + 130 + s * 45), fill=(235, 235, 235), width=30)
    text(g, cx - 0.76 * R, cy + 0.42 * R, 'L', 110)
    text(g, cx + 0.76 * R, cy + 0.42 * R, 'R', 110)
    y0 = cy + 0.30 * R
    g.arc((cx - 1.3 * R, y0 - 1.4 * R, cx + 1.3 * R, y0 + 0.12 * R), 62, 118, fill=(240, 240, 240), width=120)
    for x in (cx - 95, cx + 95):
        g.line((x, y0 - 10, x, y0 + 120), fill=(30, 30, 30), width=12)
    text(g, cx, cy + 0.64 * R, '2 MIN', 88)
    text(g, cx, cy + 0.80 * R, 'NO PITCH INFORMATION', 50)
    white_patch(im)
    im.save(p)
    flat_uv(os.path.join(d, 'turn', 'turn.ac'), ('needle',))


def ai(d):
    p = os.path.join(d, 'ai', 'ai.png')
    w, h = 2048, 1024
    im = atlas(p, w, h)
    g = ImageDraw.Draw(im)
    sky, ground = (86, 170, 240), (130, 94, 44)
    # the roll ring: the left half
    cx, cy, R = 0.25 * w, 0.5 * h, 0.493 * h
    g.pieslice((cx - R, cy - R, cx + R, cy + R), 180, 360, fill=sky)
    g.pieslice((cx - R, cy - R, cx + R, cy + R), 0, 180, fill=ground)
    g.line((cx - R, cy, cx + R, cy), fill=W, width=12)
    for a, big in ((0, 2), (10, 0), (20, 0), (30, 1), (45, 1), (60, 1), (-10, 0), (-20, 0), (-30, 1), (-45, 1), (-60, 1)):
        tick(g, cx, cy, a, (0.80 if big else 0.86) * R, 0.985 * R, 22 if big == 2 else 14 if big else 9)
    # the pitch tape: the right half
    x0, x1 = 0.512 * w, 0.987 * w
    y0, y1 = (1 - 0.994) * h, (1 - 0.027) * h
    ym = (y0 + y1) / 2
    g.rectangle((x0, y0, x1, ym), fill=sky)
    g.rectangle((x0, ym, x1, y1), fill=ground)
    g.rectangle((x0, ym - 6, x1, ym + 6), fill=W)
    xm = (x0 + x1) / 2
    step = (y1 - y0) / 2 / 3.75          # the original's 10 deg
    for k, width in ((5, 0.09), (10, 0.20), (15, 0.09), (20, 0.20)):
        for sgn in (-1, 1):
            y = ym + sgn * step * k / 10
            hw = width * (x1 - x0)
            g.line((xm - hw, y, xm + hw, y), fill=W, width=8)
            if k % 10 == 0:
                text(g, xm - hw - 60, y, str(k), 58)
                text(g, xm + hw + 60, y, str(k), 58)
    g.rectangle((0.069 * w, (1 - 0.084) * h, 0.131 * w, (1 - 0.014) * h), fill=INK)        # the maker's name on the mask at the foot of the dial, stretched past reading
    im.save(p)


def flaps(d):
    """the X-57's own indicator: 0 at the top, 10 at 60 deg, 30 at 180 [ICD slide 58]. Its pivot is the dial's middle, where the Tecnam's was off to one side"""
    p = os.path.join(d, 'flaps', 'flaps.png')
    im = atlas(p)
    g = ImageDraw.Draw(im)
    cx, cy = S / 2, S / 2
    R = 0.485 * S
    g.ellipse((cx - R, cy - R, cx + R, cy + R), fill=INK)
    for k in range(0, 31, 2):
        major = k % 10 == 0
        tick(g, cx, cy, 6 * k, (0.78 if major else 0.86) * R, 0.97 * R, 30 if major else 16)
    for k in (10, 20, 30):
        x, y = polar(cx, cy, 6 * k, 0.60 * R)
        text(g, x, y, str(k), 170)
    text(g, cx - 0.42 * R, cy - 0.42 * R, 'DEGREES', 96)
    text(g, cx - 0.42 * R, cy + 0.28 * R, 'FLAP', 96)
    text(g, cx - 0.42 * R, cy + 0.42 * R, 'POSITION', 96)
    white_patch(im)
    im.save(p)
    a = os.path.join(d, 'flaps', 'flaps.ac')
    flat_uv(a, ('needle',))
    flat_uv(a, ('flapslever',), 0.5, 0.5)       # the lever's handle took a corner of the Tecnam's picture: now the dial's black middle
    t = open(a, errors='replace').read()
    parts = re.split(r'(?=^OBJECT )', t, flags=re.M)
    for w, o in enumerate(parts):
        if not re.search(r'^name "needle"', o, re.M):
            continue
        l = o.split('\n')
        k = next(i for i, q in enumerate(l) if q.startswith('numvert'))
        loc = re.search(r'^loc (\S+) (\S+) (\S+)', o, re.M)
        lz = float(loc.group(3)) if loc else 0.0
        ly = float(loc.group(2)) if loc else 0.0
        for i in range(k + 1, k + 1 + int(l[k].split()[1])):
            q = [float(x) for x in l[i].split()[:3]]
            y, z = -(q[2] + lz), q[1] + ly             # the model's y and z
            dy, dz = y + 0.00671, z - 0.012           # from the old pivot; turned to point up, on the new one
            ny, nz = -dz, dy
            l[i] = '%.6f %.6f %.6f' % (q[0], nz - ly, -ny - lz)
        parts[w] = '\n'.join(l)
    open(a, 'w', errors='replace').write(''.join(parts))


def chrono(d):
    p = os.path.join(d, 'chrono', 'chrono.png')
    im = atlas(p)
    g = ImageDraw.Draw(im)
    plate = im.getpixel((int(0.1 * S), int(0.5 * S)))
    g.rectangle((0.2 * S, 0.10 * S, 0.8 * S, 0.165 * S), fill=plate)
    g.rectangle((0.2 * S, 0.575 * S, 0.72 * S, 0.675 * S), fill=plate)
    text(g, 0.49 * S, 0.135 * S, 'CHRONOMETER', 66)
    for x, s in ((0.27, 'GMT'), (0.37, 'LT'), (0.48, 'ET')):
        text(g, x * S, 0.605 * S, s, 58)
    text(g, 0.37 * S, 0.65 * S, 'SELECT', 58)
    text(g, 0.62 * S, 0.65 * S, 'CONTROL', 58)
    # the digits: segments
    seg = {'0': 'abcdef', '1': 'bc', '2': 'abged', '3': 'abgcd', '4': 'fgbc', '5': 'afgcd', '6': 'afgedc', '7': 'abc', '8': 'abcdefg', '9': 'abcdfg'}
    g.rectangle((0, 0.885 * S, S, S), fill=(8, 24, 12))
    col = (70, 230, 110)
    for i in range(10):
        x0, y0, w_, h_ = (0.015 + 0.0985 * i) * S, 0.897 * S, 0.062 * S, 0.092 * S
        c = seg[str(i)]
        L = {'a': (x0 + 12, y0, x0 + w_ - 12, y0), 'g': (x0 + 12, y0 + h_ / 2, x0 + w_ - 12, y0 + h_ / 2), 'd': (x0 + 12, y0 + h_, x0 + w_ - 12, y0 + h_),
             'f': (x0, y0 + 12, x0, y0 + h_ / 2 - 12), 'b': (x0 + w_, y0 + 12, x0 + w_, y0 + h_ / 2 - 12), 'e': (x0, y0 + h_ / 2 + 12, x0, y0 + h_ - 12), 'c': (x0 + w_, y0 + h_ / 2 + 12, x0 + w_, y0 + h_ - 12)}
        for k in c:
            g.line(L[k], fill=col, width=16)
    im.save(p)


def quadrant(base, shift):
    """the pedestal's plate as the X-57's: no mixture, the four levers' slots moved to the middle, and the words on the aircraft's own quadrant. shift: the slots' move, as a share of the picture"""
    p = os.path.join(base, 'Pedestal', 'pedestal.png')
    im = Image.open(p).convert('RGB').resize((S, S), Image.LANCZOS)
    plate = im.getpixel((int(0.6 * S), int(0.7 * S)))
    g = ImageDraw.Draw(im)
    k = S / 700.0                                                 # the boxes below were read off a 700-pixel copy
    for box in ((262, 145, 279, 330), (250, 394, 292, 410), (335, 232, 348, 278), (335, 148, 347, 174), (316, 394, 364, 416), (376, 12, 442, 416)):
        g.rectangle(tuple(q * k for q in box), fill=plate)
    band = tuple(int(q * k) for q in (235, 6, 375, 400))
    slots = im.crop(band)
    g.rectangle(band, fill=plate)
    im.paste(slots, (band[0] + int(shift * S), band[1]))

    def tape(x0, x1, v, word):
        y = (1 - v) * S
        g.rectangle((x0, y - 17, x1, y + 17), fill=(232, 232, 226))
        f = font(25)
        g.text(((x0 + x1 - g.textlength(word, font=f)) / 2, y - 15), word, font=f, fill=INK)
    for v, word in ((0.937, 'OVERDRIVE'), (0.847, 'FULL'), (0.504, 'IDLE'), (0.452, 'REGEN')):
        tape(0.32 * S, 0.378 * S, v, word)
    for v, word in ((0.925, 'MAX RPM'), (0.55, 'MIN RPM'), (0.452, 'FEATHER')):
        tape(0.572 * S, 0.63 * S, v, word)
    im.save(p)


def paint(base):
    d = os.path.join(base, 'Instruments')
    for f in (asi, vsi, alt, hi, turn, ai, flaps, chrono):
        f(d)
    print('  8 instrument faces redrawn')
