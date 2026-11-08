"""A small reader for NASA's .xlsx files, which needs nothing beyond the standard library.

Each cell is placed by its own reference, because a row's XML leaves out empty cells: reading them in order would
shift what follows. Formula cells give their stored value.

    sheets = read(path)             # {sheet name: Grid}
    g = sheets['Flaps 30']
    g[3, 1]                         # row 3, column 1 (from 0), None if empty
    g.row(3)                        # the row as a list, None where empty
"""
import re
import zipfile
import xml.etree.ElementTree as ET

M = '{http://schemas.openxmlformats.org/spreadsheetml/2006/main}'
R = '{http://schemas.openxmlformats.org/officeDocument/2006/relationships}'


def col_index(ref):
    """'AB12' -> 27"""
    n = 0
    for ch in re.match(r'[A-Z]+', ref).group(0):
        n = n * 26 + ord(ch) - 64
    return n - 1


def col_name(i):
    s = ''
    i += 1
    while i:
        i, r = divmod(i - 1, 26)
        s = chr(65 + r) + s
    return s


class Grid:
    def __init__(s, cells):
        s.cells = cells
        s.nrows = 1 + max((r for r, _ in cells), default=-1)
        s.ncols = 1 + max((c for _, c in cells), default=-1)

    def __getitem__(s, rc):
        return s.cells.get(rc)

    def row(s, r):
        return [s.cells.get((r, c)) for c in range(s.ncols)]


def _text(si):
    return ''.join(t.text or '' for t in si.iter(M + 't'))


def read(path):
    z = zipfile.ZipFile(path)
    strings = []
    if 'xl/sharedStrings.xml' in z.namelist():
        strings = [_text(si) for si in ET.fromstring(z.read('xl/sharedStrings.xml')).findall(M + 'si')]
    rels = {r.get('Id'): r.get('Target') for r in ET.fromstring(z.read('xl/_rels/workbook.xml.rels'))}
    out = {}
    for sh in ET.fromstring(z.read('xl/workbook.xml')).find(M + 'sheets'):
        target = rels[sh.get(R + 'id')].lstrip('/')
        target = target if target.startswith('xl/') else 'xl/' + target
        cells = {}
        for row in ET.fromstring(z.read(target)).find(M + 'sheetData'):
            r = int(row.get('r')) - 1
            for c in row:
                v = c.find(M + 'v')
                t = c.get('t')
                if t == 'inlineStr':
                    val = _text(c.find(M + 'is'))
                elif v is None or v.text is None:
                    continue
                elif t == 's':
                    val = strings[int(v.text)]
                elif t in ('str', 'e'):
                    val = v.text
                else:
                    val = float(v.text)
                if isinstance(val, str):
                    val = val.strip()
                    if not val:
                        continue
                cells[r, col_index(c.get('r'))] = val
        out[sh.get('name')] = Grid(cells)
    return out
