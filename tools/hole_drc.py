r"""Independent hole-to-copper / hole-to-hole clearance check.

The project's DRC constraints (led_board.kicad_pro) are:

    min_clearance      0.10   copper edge  -> copper edge, same layer, diff net
    min_hole_clearance 0.25   drill edge   -> copper edge, ANY layer, diff net
    min_hole_to_hole   0.20   drill edge   -> drill edge

A drilled hole is a cylinder through the whole stack, so hole clearance is not
per-layer: a B.Cu track has to clear the hole of a via or through-hole pad even
when the pad's copper only exists on F.Cu.

Same-net pairs are exempt (otherwise no track could ever reach its own via).

Run:  python tools/hole_drc.py            # whole board
      python tools/hole_drc.py --board X   # some other .kicad_pcb
"""
from __future__ import annotations

import argparse
import collections
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kicad_sexpr as ks

HOLE_CLEAR = 0.25
COPPER_CLEAR = 0.10
HOLE_TO_HOLE = 0.20
TOL = 1e-4          # file coords are rounded; ignore sub-0.1um shortfalls

CU_LAYERS = ("F.Cu", "In1.Cu", "In2.Cu", "B.Cu")
POWER_NETS = ("GND", "/V_A", "/V_B")


# ------------------------------------------------------------------ geometry --

def pt_seg(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 < 1e-18 else max(0.0, min(1.0, ((p[0]-a[0])*dx + (p[1]-a[1])*dy) / l2))
    return math.hypot(p[0] - a[0] - t*dx, p[1] - a[1] - t*dy)


def seg_seg(a, b, c, d):
    def cr(o, p, q):
        return (p[0]-o[0])*(q[1]-o[1]) - (p[1]-o[1])*(q[0]-o[0])
    d1, d2, d3, d4 = cr(c, d, a), cr(c, d, b), cr(a, b, c), cr(a, b, d)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return 0.0
    return min(pt_seg(a, c, d), pt_seg(b, c, d), pt_seg(c, a, b), pt_seg(d, a, b))


def rot(x, y, deg):
    """Footprint-local offset -> board offset (KiCad's y-down convention)."""
    a = math.radians(deg)
    return (x * math.cos(a) + y * math.sin(a), -x * math.sin(a) + y * math.cos(a))


class Shape:
    """Convex copper/hole shape: a core polyline/point inflated by `r`.

    circle -> single point + r; oval -> 2 points + r; rect/roundrect -> 4
    corners (inset by the corner radius) + r.  Distance between two such shapes
    is the distance between their cores minus both radii, which is exact for
    everything this board actually uses.
    """

    __slots__ = ("core", "r", "bbox")

    def __init__(self, core, r):
        self.core = core
        self.r = r
        xs = [p[0] for p in core]
        ys = [p[1] for p in core]
        self.bbox = (min(xs) - r, min(ys) - r, max(xs) + r, max(ys) + r)

    def dist(self, other):
        a, b = self.core, other.core
        if len(a) == 1 and len(b) == 1:
            d = math.hypot(a[0][0]-b[0][0], a[0][1]-b[0][1])
        elif len(a) == 1:
            d = _pt_poly(a[0], b)
        elif len(b) == 1:
            d = _pt_poly(b[0], a)
        else:
            d = _poly_poly(a, b)
        return d - self.r - other.r


def _edges(core):
    if len(core) == 2:
        return [(core[0], core[1])]
    return list(zip(core, core[1:] + core[:1]))


def _inside(p, core):
    if len(core) < 3:
        return False
    s = 0
    for a, b in _edges(core):
        cr = (b[0]-a[0])*(p[1]-a[1]) - (b[1]-a[1])*(p[0]-a[0])
        s += 1 if cr > 0 else -1
    return abs(s) == len(core)


def _pt_poly(p, core):
    if _inside(p, core):
        return 0.0
    return min(pt_seg(p, a, b) for a, b in _edges(core))


def _poly_poly(c1, c2):
    if _inside(c1[0], c2) or _inside(c2[0], c1):
        return 0.0
    return min(seg_seg(a, b, c, d) for a, b in _edges(c1) for c, d in _edges(c2))


def circle(x, y, r):
    return Shape([(x, y)], r)


def stadium(x, y, w, h, deg):
    """KiCad oval pad: capsule of diameter min(w,h)."""
    r = min(w, h) / 2
    half = abs(max(w, h) / 2 - r)
    dx, dy = rot(half, 0.0, deg) if w >= h else rot(0.0, half, deg)
    return Shape([(x - dx, y - dy), (x + dx, y + dy)], r)


def rounded_rect(x, y, w, h, deg, cr):
    cr = max(0.0, min(cr, min(w, h) / 2))
    ax, ay = w / 2 - cr, h / 2 - cr
    core = []
    for lx, ly in ((-ax, -ay), (ax, -ay), (ax, ay), (-ax, ay)):
        dx, dy = rot(lx, ly, deg)
        core.append((x + dx, y + dy))
    if cr == 0.0 and (ax == 0.0 or ay == 0.0):      # degenerate -> line/point
        core = list(dict.fromkeys(core))
    return Shape(core, cr)


# -------------------------------------------------------------------- loader --

Item = collections.namedtuple("Item", "kind net layers shape desc")


def load(path):
    pcb = ks.load(path)
    copper, holes = [], []
    anon = [0]

    def netof(node):
        n = node.find("net")
        if n is not None and len(n) > 1:
            return str(n[1])
        anon[0] += 1
        return "!nonet%d" % anon[0]

    for s in pcb.find_all("segment"):
        ly = str(s.value("layer"))
        if ly not in CU_LAYERS:
            continue
        a, b = s.find("start"), s.find("end")
        p1 = (float(a[1]), float(a[2]))
        p2 = (float(b[1]), float(b[2]))
        w = float(s.value("width"))
        copper.append(Item("track", str(s.value("net")), (ly,),
                           Shape(list(dict.fromkeys([p1, p2])), w / 2),
                           "track %s %s->%s" % (ly, fmt(p1), fmt(p2))))

    for v in pcb.find_all("via"):
        at = v.find("at")
        x, y = float(at[1]), float(at[2])
        net = str(v.value("net"))
        size = float(v.value("size"))
        drill = float(v.value("drill"))
        lys = tuple(str(l) for l in v.find("layers")[1:])
        copper.append(Item("via", net, lys, circle(x, y, size / 2),
                           "via %s d%.2f" % (fmt((x, y)), size)))
        holes.append(Item("via-hole", net, CU_LAYERS, circle(x, y, drill / 2),
                          "via-hole %s drill%.2f" % (fmt((x, y)), drill)))

    for fp in pcb.find_all("footprint"):
        ref = fp_ref(fp) or "?"
        at = fp.find("at")
        ox, oy = float(at[1]), float(at[2])
        frot = float(at[3]) if len(at) > 3 else 0.0
        for pad in fp.find_all("pad"):
            name, ptype, pshape = str(pad[1]), str(pad[2]), str(pad[3])
            pa = pad.find("at")
            dx, dy = rot(float(pa[1]), float(pa[2]), frot)
            x, y = ox + dx, oy + dy
            deg = float(pa[3]) if len(pa) > 3 else frot   # stored absolute
            sz = pad.find("size")
            w, h = float(sz[1]), float(sz[2])
            net = netof(pad)
            lys = tuple(str(l) for l in pad.find("layers")[1:])
            lys = CU_LAYERS if any(l == "*.Cu" for l in lys) else \
                tuple(l for l in lys if l in CU_LAYERS)
            desc = "%s pad %s (%s %s)" % (ref, name, ptype, pshape)

            if lys:
                for sh in pad_shapes(pad, x, y, w, h, deg, pshape):
                    copper.append(Item("pad", net, lys, sh, desc))
            d = pad.find("drill")
            if d is not None:
                vals = [t for t in d[1:] if not isinstance(t, ks.Node)
                        and t != "oval"]
                dv = [float(t) for t in vals]
                oval = "oval" in [str(t) for t in d[1:]]
                if oval and len(dv) >= 2:
                    hs = stadium(x, y, dv[0], dv[1], deg)
                else:
                    hs = circle(x, y, dv[0] / 2)
                holes.append(Item("pad-hole", net, CU_LAYERS, hs,
                                  desc + " hole"))
    return pcb, copper, holes


def pad_shapes(pad, x, y, w, h, deg, pshape):
    if pshape == "circle":
        return [circle(x, y, w / 2)]
    if pshape == "oval":
        return [stadium(x, y, w, h, deg)]
    if pshape in ("rect", "trapezoid"):
        return [rounded_rect(x, y, w, h, deg, 0.0)]
    if pshape == "roundrect":
        rr = float(pad.value("roundrect_rratio", 0.0))
        return [rounded_rect(x, y, w, h, deg, rr * min(w, h))]
    # custom: anchor shape (conservatively a rect) plus every primitive poly
    out = [rounded_rect(x, y, w, h, deg, 0.0)]
    prims = pad.find("primitives")
    if prims is not None:
        for gp in prims.find_all("gr_poly"):
            pts = gp.find("pts")
            core = []
            for q in pts.find_all("xy"):
                ddx, ddy = rot(float(q[1]), float(q[2]), deg)
                core.append((x + ddx, y + ddy))
            if len(core) >= 2:
                out.append(Shape(core, 0.0))
    return out


def fp_ref(fp):
    for p in fp.find_all("property"):
        if p[1] == "Reference":
            return str(p[2])
    return None


def fmt(p):
    return "(%.3f,%.3f)" % (p[0], p[1])


# --------------------------------------------------------------------- check --

def bucketize(items, cell):
    grid = collections.defaultdict(list)
    for it in items:
        x0, y0, x1, y1 = it.shape.bbox
        for gx in range(int(math.floor(x0 / cell)), int(math.floor(x1 / cell)) + 1):
            for gy in range(int(math.floor(y0 / cell)), int(math.floor(y1 / cell)) + 1):
                grid[(gx, gy)].append(it)
    return grid


def check(copper, holes, hole_clear=HOLE_CLEAR):
    cell = 2.0
    grid = bucketize(copper, cell)
    bad = []
    for h in holes:
        x0, y0, x1, y1 = h.shape.bbox
        near = set()
        for gx in range(int(math.floor((x0-hole_clear) / cell)),
                        int(math.floor((x1+hole_clear) / cell)) + 1):
            for gy in range(int(math.floor((y0-hole_clear) / cell)),
                            int(math.floor((y1+hole_clear) / cell)) + 1):
                for it in grid.get((gx, gy), ()):
                    near.add(id(it))
                    NEAR[id(it)] = it
        for k in near:
            it = NEAR[k]
            if it.net == h.net:
                continue
            d = h.shape.dist(it.shape)
            if d < hole_clear - TOL:
                bad.append((round(d, 4), hole_clear, h, it))
    bad.sort(key=lambda t: t[0])
    return bad


NEAR = {}


def check_copper(copper, clear=COPPER_CLEAR):
    cell = 2.0
    grid = bucketize(copper, cell)
    bad, seen = [], set()
    for bucket in grid.values():
        for i in range(len(bucket)):
            for j in range(i + 1, len(bucket)):
                a, b = bucket[i], bucket[j]
                if a.net == b.net:
                    continue
                if not set(a.layers) & set(b.layers):
                    continue
                key = (id(a), id(b)) if id(a) < id(b) else (id(b), id(a))
                if key in seen:
                    continue
                seen.add(key)
                d = a.shape.dist(b.shape)
                if d < clear - TOL:
                    bad.append((round(d, 4), clear, a, b))
    bad.sort(key=lambda t: t[0])
    return bad


def check_h2h(holes, clear=HOLE_TO_HOLE):
    cell = 2.0
    grid = bucketize(holes, cell)
    bad, seen = [], set()
    for bucket in grid.values():
        for i in range(len(bucket)):
            for j in range(i + 1, len(bucket)):
                a, b = bucket[i], bucket[j]
                key = (id(a), id(b)) if id(a) < id(b) else (id(b), id(a))
                if key in seen:
                    continue
                seen.add(key)
                d = a.shape.dist(b.shape)
                if d < clear - TOL:
                    bad.append((round(d, 4), clear, a, b))
    bad.sort(key=lambda t: t[0])
    return bad


def report(title, bad, limit=25):
    print("\n%s: %d violations" % (title, len(bad)))
    kinds = collections.Counter((a.kind, b.kind) for _, _, a, b in bad)
    for k, n in kinds.most_common():
        print("   %-22s %d" % ("%s vs %s" % k, n))
    for d, need, a, b in bad[:limit]:
        print("   d=%.4f need=%.2f  %s [%s]  vs  %s [%s]"
              % (d, need, a.desc, a.net, b.desc, b.net))
    if len(bad) > limit:
        print("   ... %d more" % (len(bad) - limit))


def main():
    ap = argparse.ArgumentParser()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--board", default=os.path.join(root, "led_board.kicad_pcb"))
    ap.add_argument("--limit", type=int, default=25)
    a = ap.parse_args()

    pcb, copper, holes = load(a.board)
    print("%s\ncopper items: %d   holes: %d" % (a.board, len(copper), len(holes)))
    nz = len(pcb.find_all("zone"))
    if nz:
        print("note: %d zone(s) present - their fills are not checked here; "
              "refill in KiCad after any edit." % nz)

    hb = check(copper, holes)
    report("HOLE -> COPPER (%.2f mm)" % HOLE_CLEAR, hb, a.limit)
    cb = check_copper(copper)
    report("COPPER -> COPPER (%.2f mm)" % COPPER_CLEAR, cb, a.limit)
    hh = check_h2h(holes)
    report("HOLE -> HOLE (%.2f mm)" % HOLE_TO_HOLE, hh, a.limit)

    total = len(hb) + len(cb) + len(hh)
    print("\nTOTAL: %d" % total)
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
