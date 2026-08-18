r"""Place all 192 well LEDs and route their signal nets, following column 1.

Board layout
------------
The plate artwork on Dwgs.User marks 96 wells on a 9 mm SBS grid
(x = 106.5 + 9*(col-1), y = 81.64 + 9*(row-1)).  Each well gets two LEDs, as
already done by hand for column 1:

    UV  (D1..D96,    SMTL3528UV-405)   at (well_x - 1.5, well_y + 2.36), rot -90
    GF  (D97..D192,  GFJTLPS124KXK511) at (well_x + 1.5, well_y - 0.64), rot  90

D<n> carries nets /<n>A + /V_A, D<96+n> carries /<n>B + /V_B, so the reference
number alone decides which well an LED belongs to.  Placement therefore means:
pick which well sits at which grid position.

Which well goes where is dictated by J1 (SEAM-50-01-L-06-2-RA-GP-TR, 6x50).
Rows A and B are GND / V_A / V_B; rows C..F carry the 192 well signals, four
per pin column, i.e. two wells per pin column.  Board column c uses the four
pin columns 49-4*(c-1) .. 46-4*(c-1), each feeding two adjacent board rows:

    row 1,2 <- pin col 49-4d     row 5,6 <- pin col 47-4d
    row 3,4 <- pin col 48-4d     row 7,8 <- pin col 46-4d     (d = c-1)

and within a pin column the C/D pair feeds the odd row, the E/F pair the even
row.  Column 1 was placed exactly this way (wells 93,94,95,96,69,70,71,72 top
to bottom), so this rule reproduces it and extends it to columns 2..12.

Routing (all on B.Cu, 0.1 mm, matching column 1)
------------------------------------------------
Per net:  via on the LED's pad 1  ->  short vertical  ->  45 deg jog into a
0.2 mm-pitch vertical corridor to the right of the column's GF LEDs  ->  long
vertical up the corridor  ->  single 45 deg diagonal  ->  vertical escape
channel in the gap between connector pin columns  ->  via on the connector pad.

Nets are ordered left to right by (pin column descending, row C->D->E->F).
Both the corridor x and the connector escape x increase with that order, which
makes the whole fan planar: the diagonals of a column are parallel 45 deg lines
with distinct x+y (or x-y), so no two can cross.  The fan runs right-to-left for
board columns 1..5 and left-to-right for 7..12 - the two families are
perpendicular, but their x ranges do not overlap (the changeover happens inside
column 6, whose only right-to-left net stops at x=150.275 while its first
left-to-right net starts at x=150.590).  See check_clearances() for the numbers.

23 of the 48 pin columns are "flipped": row C carries the well's B net rather
than its A net, so the net that leaves the connector first needs the *right*
LED of the pair.  Those pairs get no via-column reuse, and the A net reaches its
UV LED by stubbing 2 mm below its own via and turning back up (see
assign_corridors), which is the only way to unwind the crossing on one layer
without touching the connector's net assignment.

Column 1 was routed by hand and is left untouched; this script only adds what is
missing.  Re-running is safe and idempotent: everything it created is recorded in
generated.json and removed first.  validate() re-checks the derived well grid,
column 1 placement and column 1 via positions against the board on every run.
"""

from __future__ import annotations

import json
import math
import os
import sys
import uuid

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kicad_sexpr as ks
from kicad_sexpr import Node, Quoted

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB = os.path.join(ROOT, "led_board.kicad_pcb")
STATE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "generated.json")

# ---------------------------------------------------------------- geometry ---

WELL_X0, WELL_Y0, WELL_PITCH = 106.5, 81.64, 9.0
N_COLS, N_ROWS = 12, 8

UV_DX, UV_DY, UV_ROT = -1.5, +2.36, -90.0   # SMTL3528UV-405
GF_DX, GF_DY, GF_ROT = +1.5, -0.64, +90.0   # GFJTLPS124KXK511
UV_PAD1_DY = -2.30                          # pad 1 offset along +y after rot
GF_PAD1_DY = -1.31

TRACK_W = 0.10
VIA_D, VIA_DRILL = 0.30, 0.20
LAYER = "B.Cu"

BUS_Y = 48.60          # below every bit of J1 copper; diagonals start here
# Escape channels in the gap left of each pin column.  1.27 mm of pin pitch
# split four ways: any less and the 45 deg runs leaving them (0.1 mm wide,
# 0.1 mm clearance -> 0.283 mm minimum x offset) would not clear each other.
ESCAPE_DX = {"C": -0.950, "D": -0.635, "E": -0.320, "F": 0.0}
CORRIDOR_GAP = 0.775   # first corridor, right of the GF LED via column
CORRIDOR_PITCH = 0.20
JOG_UP = 3.45          # normal jog, this far above the well centre
JOG_DOWN = 2.00        # jog below the well centre (see assign_corridors)

CLEARANCE = 0.10       # Default net class


def well_x(col):
    return WELL_X0 + WELL_PITCH * (col - 1)


def well_y(row):
    return WELL_Y0 + WELL_PITCH * (row - 1)


# ------------------------------------------------------------- pcb helpers ---


def fp_ref(fp):
    for p in fp.find_all("property"):
        if p[1] == "Reference":
            return str(p[2])
    return None


def fp_rotate(rot, x, y):
    """Local footprint coords -> offset from the footprint origin."""
    a = math.radians(rot)
    return (x * math.cos(a) + y * math.sin(a), -x * math.sin(a) + y * math.cos(a))


def pad_net(pad):
    n = pad.find("net")
    return str(n[1]) if n is not None and len(n) > 1 else None


def read_connector(pcb):
    """{(row_letter, pin_col): (x, y, net)} plus the four row y values."""
    for fp in pcb.find_all("footprint"):
        if fp_ref(fp) != "J1":
            continue
        at = fp.find("at")
        ox, oy = float(at[1]), float(at[2])
        rot = float(at[3]) if len(at) > 3 else 0.0
        pins = {}
        for pad in fp.find_all("pad"):
            name = str(pad[1])
            if len(name) < 2 or name[0] not in "ABCDEF" or not name[1:].isdigit():
                continue
            pa = pad.find("at")
            dx, dy = fp_rotate(rot, float(pa[1]), float(pa[2]))
            pins[(name[0], int(name[1:]))] = (
                round(ox + dx, 4), round(oy + dy, 4), pad_net(pad))
        return pins
    raise SystemExit("J1 not found")


def uid(*parts):
    return str(uuid.uuid5(uuid.UUID("6f1a3d52-0000-4000-8000-000000000001"),
                          "|".join(str(p) for p in parts)))


def make_via(x, y, net, key):
    n = Node(["via"])
    n.append(Node(["at", fmt(x), fmt(y)]))
    n.append(Node(["size", fmt(VIA_D)]))
    n.append(Node(["drill", fmt(VIA_DRILL)]))
    n.append(Node(["layers", Quoted("F.Cu"), Quoted("B.Cu")]))
    n.append(Node(["net", Quoted(net)]))
    n.append(Node(["uuid", Quoted(uid(key))]))
    return n


def make_segment(x1, y1, x2, y2, net, key):
    n = Node(["segment"])
    n.append(Node(["start", fmt(x1), fmt(y1)]))
    n.append(Node(["end", fmt(x2), fmt(y2)]))
    n.append(Node(["width", fmt(TRACK_W)]))
    n.append(Node(["layer", Quoted(LAYER)]))
    n.append(Node(["net", Quoted(net)]))
    n.append(Node(["uuid", Quoted(uid(key))]))
    return n


def fmt(v):
    s = f"{round(v, 6):.6f}".rstrip("0").rstrip(".")
    return s if s not in ("", "-0") else "0"


def node_uuid(n):
    u = n.find("uuid")
    return str(u[1]) if u is not None else None


# --------------------------------------------------------- well assignment ---


def well_of(net):
    """'/93A' -> (93, 'A')"""
    return int(net[1:-1]), net[-1]


def build_plan(pins):
    """-> rows: list of dicts, one per (col, row) well position."""
    plan = []
    for col in range(1, N_COLS + 1):
        d = col - 1
        pin_cols = [49 - 4 * d, 48 - 4 * d, 47 - 4 * d, 46 - 4 * d]
        for row in range(1, N_ROWS + 1):
            p = pin_cols[(row - 1) // 2]
            first = (row - 1) % 2 == 0          # C/D pair vs E/F pair
            rows = ("C", "D") if first else ("E", "F")
            n0, n1 = pins[(rows[0], p)][2], pins[(rows[1], p)][2]
            w0, s0 = well_of(n0)
            w1, s1 = well_of(n1)
            assert w0 == w1 and {s0, s1} == {"A", "B"}, (p, n0, n1)
            plan.append({
                "col": col, "row": row, "well": w0, "pin_col": p,
                "A_row": rows[0] if s0 == "A" else rows[1],
                "B_row": rows[0] if s0 == "B" else rows[1],
            })
    assert len({e["well"] for e in plan}) == 96
    return plan


def assign_corridors(plan):
    """Add corridor x and jog height per net, in connector escape order.

    Corridors form a 0.2 mm-pitch ladder just right of the column's GF LED via
    column, one rung per net, in the order the nets leave the connector.  As in
    column 1, the first two nets can reuse the LED via columns themselves
    (x_uv, then x_gf) and skip their jog entirely - but only when the pin
    column's C row carries the A net, i.e. when the net that comes first is
    the UV LED's, which is the left of the pair.  Where C carries the B net
    instead ("flipped" pin columns) every net gets its own rung.

    Jog height: nets normally turn into their rung 3.45 mm above the well, in
    the clear band between two LED rows.  In a flipped pair the A net has to
    reach across x_gf *and* across the B net's rung, both of which are busy
    just above the well, so it stubs 2 mm the other way - below its own via -
    and turns there instead.
    """
    by_col = {}
    for e in plan:
        by_col.setdefault(e["col"], []).append(e)
    nets = []
    for col in range(1, N_COLS + 1):
        entries = sorted(by_col[col], key=lambda e: e["row"])
        xu, xg = well_x(col) + UV_DX, well_x(col) + GF_DX
        # nets in escape order: row1 C,D  row2 E,F  row3 C,D  ...
        ordered = []
        for e in entries:
            first = "A" if e["A_row"] in ("C", "E") else "B"
            ordered.append((e, first, False))
            ordered.append((e, "B" if first == "A" else "A", first == "B"))
        # Reuse the two LED via columns only when the first net to leave the
        # connector is the UV (left) LED's, i.e. the first pin column is not
        # flipped.  Otherwise the pair would be crossed before it started.
        head = ordered[0][1] == "A"
        for j, (e, side, flipped_second) in enumerate(ordered):
            if head and j < 2:
                x = xu if j == 0 else xg
            else:
                k = j - 2 if head else j
                x = xg + CORRIDOR_GAP + CORRIDOR_PITCH * k
            wy = well_y(e["row"])
            nets.append({
                "net": "/%d%s" % (e["well"], side),
                "col": e["col"], "row": e["row"], "well": e["well"],
                "pin_col": e["pin_col"],
                "pin_row": e["A_row"] if side == "A" else e["B_row"],
                "corridor": round(x, 4),
                "led_x": round(xu if side == "A" else xg, 4),
                "led_via_y": round(wy + (UV_PAD1_DY + UV_DY if side == "A"
                                         else GF_PAD1_DY + GF_DY), 4),
                "y_jog": round(wy + JOG_DOWN if flipped_second
                               else wy - JOG_UP, 4),
            })
    return nets


# ---------------------------------------------------------------- routing ----


def route(n, pins):
    """Corner list from the connector pad down to the LED via."""
    px, py, _ = pins[(n["pin_row"], n["pin_col"])]
    xe = round(px + ESCAPE_DX[n["pin_row"]], 4)
    xc = n["corridor"]
    pts = [(px, py)]
    if xe != px:                                  # 45 deg step into the gap
        pts.append((xe, round(py + abs(px - xe), 4)))
    pts.append((xe, BUS_Y))                       # down to the fan-out line
    yb = round(BUS_Y + abs(xe - xc), 4)
    if xc != xe:                                  # the one long 45 deg run
        pts.append((xc, yb))

    lx, lvy = n["led_x"], n["led_via_y"]
    if xc == lx:
        pts.append((lx, lvy))
    else:
        delta = abs(xc - lx)
        y_jog = max(n["y_jog"], yb + delta)       # never overshoot the bend
        pts.append((xc, round(y_jog - delta, 4)))
        pts.append((lx, round(y_jog, 4)))
        pts.append((lx, lvy))

    out = [pts[0]]
    for p in pts[1:]:
        if p != out[-1]:
            out.append(p)
    for a, b in zip(out, out[1:]):
        dx, dy = abs(b[0] - a[0]), abs(b[1] - a[1])
        assert dx < 1e-6 or dy < 1e-6 or abs(dx - dy) < 1e-6, (n["net"], a, b)
    return out, (px, py), xe


# ------------------------------------------------------------ drc checking ---


def seg_seg_dist(a, b, c, d):
    def sub(p, q):
        return (p[0] - q[0], p[1] - q[1])

    def cross(p, q):
        return p[0] * q[1] - p[1] * q[0]

    r, s = sub(b, a), sub(d, c)
    den = cross(r, s)
    qp = sub(c, a)
    if abs(den) > 1e-12:
        t, u = cross(qp, s) / den, cross(qp, r) / den
        if -1e-12 <= t <= 1 + 1e-12 and -1e-12 <= u <= 1 + 1e-12:
            return 0.0
    return min(pt_seg_dist(a, c, d), pt_seg_dist(b, c, d),
               pt_seg_dist(c, a, b), pt_seg_dist(d, a, b))


def pt_seg_dist(p, a, b):
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    if l2 < 1e-18:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def pt_rect_dist(p, cx, cy, w, h):
    dx = max(abs(p[0] - cx) - w / 2, 0.0)
    dy = max(abs(p[1] - cy) - h / 2, 0.0)
    return math.hypot(dx, dy)


def seg_rect_dist(a, b, cx, cy, w, h):
    x0, x1 = cx - w / 2, cx + w / 2
    y0, y1 = cy - h / 2, cy + h / 2
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    edges = list(zip(corners, corners[1:] + corners[:1]))
    if x0 <= a[0] <= x1 and y0 <= a[1] <= y1:
        return 0.0
    return min(seg_seg_dist(a, b, e[0], e[1]) for e in edges)


def collect_obstacles(pcb, new_uuids):
    """Copper on B.Cu that is not a track: vias and *.Cu pads."""
    circles, rects = [], []          # (x, y, r, net) / (x, y, w, h, net)
    for v in pcb.find_all("via"):
        at = v.find("at")
        circles.append((float(at[1]), float(at[2]),
                        float(v.value("size")) / 2, str(v.value("net")),
                        node_uuid(v) in new_uuids))
    for fp in pcb.find_all("footprint"):
        at = fp.find("at")
        ox, oy = float(at[1]), float(at[2])
        frot = float(at[3]) if len(at) > 3 else 0.0
        for pad in fp.find_all("pad"):
            layers = [str(x) for x in pad.find("layers")[1:]]
            if not any(l in ("*.Cu", LAYER) for l in layers):
                continue
            pa = pad.find("at")
            dx, dy = fp_rotate(frot, float(pa[1]), float(pa[2]))
            x, y = ox + dx, oy + dy
            sz = pad.find("size")
            w, h = float(sz[1]), float(sz[2])
            prot = (float(pa[3]) if len(pa) > 3 else 0.0) - frot
            if abs(math.sin(math.radians(prot))) > 0.5:
                w, h = h, w
            if abs(math.sin(math.radians(frot))) > 0.5:
                w, h = h, w
            net = pad_net(pad) or "<no-net:%s>" % id(pad)
            if str(pad[3]) == "circle":
                circles.append((x, y, w / 2, net, False))
            else:
                rects.append((x, y, w, h, net, False))
    return circles, rects


# Coordinates in the file are rounded, so a nominally exact 0.1 mm gap can read
# as 0.099998.  Anything that tight is intentional, not a routing error.
TOL = 2e-3


def check_clearances(pcb, new_uuids):
    """-> (violations involving new copper, violations only in old copper)"""
    segs = []
    for s in pcb.find_all("segment"):
        if str(s.value("layer")) != LAYER:
            continue
        st, en = s.find("start"), s.find("end")
        segs.append(((float(st[1]), float(st[2])), (float(en[1]), float(en[2])),
                     float(s.value("width")) / 2, str(s.value("net")),
                     node_uuid(s) in new_uuids))
    circles, rects = collect_obstacles(pcb, new_uuids)

    bad = []
    # bucket segments by x to keep the pair sweep cheap
    BUCKET = 5.0
    buckets = {}
    for i, s in enumerate(segs):
        lo = int(min(s[0][0], s[1][0]) / BUCKET) - 1
        hi = int(max(s[0][0], s[1][0]) / BUCKET) + 1
        for k in range(lo, hi + 1):
            buckets.setdefault(k, []).append(i)
    seen = set()
    for ids in buckets.values():
        for x in range(len(ids)):
            for y in range(x + 1, len(ids)):
                i, j = ids[x], ids[y]
                if (i, j) in seen:
                    continue
                seen.add((i, j))
                a1, b1, h1, n1, new1 = segs[i]
                a2, b2, h2, n2, new2 = segs[j]
                if n1 == n2:
                    continue
                need = CLEARANCE + h1 + h2
                d = seg_seg_dist(a1, b1, a2, b2)
                if d < need - TOL:
                    bad.append((new1 or new2, "track/track", n1, n2, a1, a2,
                                round(d, 4), round(need, 4)))
    for a, b, hw, net, new in segs:
        for cx, cy, r, cnet, cnew in circles:
            if cnet == net:
                continue
            d = pt_seg_dist((cx, cy), a, b)
            if d < CLEARANCE + hw + r - TOL:
                bad.append((new or cnew, "track/round", net, cnet, a, (cx, cy),
                            round(d, 4), round(CLEARANCE + hw + r, 4)))
        for cx, cy, w, h, pnet, pnew in rects:
            if pnet == net:
                continue
            d = seg_rect_dist(a, b, cx, cy, w, h)
            if d < CLEARANCE + hw - TOL:
                bad.append((new or pnew, "track/rect", net, pnet, a, (cx, cy),
                            round(d, 4), round(CLEARANCE + hw, 4)))
    for i in range(len(circles)):
        for j in range(i + 1, len(circles)):
            x1, y1, r1, n1, new1 = circles[i]
            x2, y2, r2, n2, new2 = circles[j]
            if n1 == n2:
                continue
            d = math.hypot(x1 - x2, y1 - y2)
            if d < CLEARANCE + r1 + r2 - TOL:
                bad.append((new1 or new2, "round/round", n1, n2, (x1, y1),
                            (x2, y2), round(d, 4), round(CLEARANCE + r1 + r2, 4)))
    for cx, cy, r, cnet, cnew in circles:
        for rx, ry, w, h, pnet, pnew in rects:
            if cnet == pnet:
                continue
            d = pt_rect_dist((cx, cy), rx, ry, w, h)
            if d < CLEARANCE + r - TOL:
                bad.append((cnew or pnew, "round/rect", cnet, pnet, (cx, cy),
                            (rx, ry), round(d, 4), round(CLEARANCE + r, 4)))
    return ([b[1:] for b in bad if b[0]], [b[1:] for b in bad if not b[0]])


# ----------------------------------------------------- self-validation -------


def validate(pcb, plan, nets):
    """Check the derived geometry really is what column 1 already contains.

    Everything here is a fact about the board rather than about this script, so
    if any of it fires the assumptions behind the whole plan are wrong.
    """
    # a. the 9 mm well grid matches the plate artwork on Dwgs.User
    circles = set()
    for n in pcb.find_all("gr_circle"):
        if str(n.value("layer")) != "Dwgs.User":
            continue
        c = n.find("center")
        circles.add((round(float(c[1]), 3), round(float(c[2]), 3)))
    grid = {(round(well_x(c), 3), round(well_y(r), 3))
            for c in range(1, N_COLS + 1) for r in range(1, N_ROWS + 1)}
    assert circles == grid, "well grid != plate artwork (%d vs %d)" % (
        len(circles), len(grid))

    # b. column 1 is already placed exactly where the plan puts it
    at_now = {}
    for fp in pcb.find_all("footprint"):
        ref = fp_ref(fp)
        at = fp.find("at")
        at_now[ref] = (round(float(at[1]), 3), round(float(at[2]), 3),
                       round(float(at[3]) if len(at) > 3 else 0.0, 3))
    for e in plan:
        if e["col"] != 1:
            continue
        wx, wy = well_x(e["col"]), well_y(e["row"])
        for ref, want in (("D%d" % e["well"], (wx + UV_DX, wy + UV_DY, UV_ROT)),
                          ("D%d" % (e["well"] + 96),
                           (wx + GF_DX, wy + GF_DY, GF_ROT))):
            got = at_now[ref]
            exp = (round(want[0], 3), round(want[1], 3), round(want[2], 3))
            assert got == exp, "col 1: %s is at %s, plan says %s" % (ref, got, exp)

    # c. the LED-pad vias the plan will drop land on pad 1, and match the vias
    #    already there for column 1
    have = {(str(v.value("net")), round(float(v.find("at")[1]), 3),
             round(float(v.find("at")[2]), 3)) for v in pcb.find_all("via")}
    for n in nets:
        if n["col"] != 1:
            continue
        key = (n["net"], round(n["led_x"], 3), round(n["led_via_y"], 3))
        assert key in have, "col 1: no existing LED via at %s" % (key,)
    print("validated: well grid, column 1 placement, column 1 LED via positions")


# -------------------------------------------------------------------- main ---


def main():
    pcb = ks.load(PCB)
    pins = read_connector(pcb)

    # 1. drop whatever a previous run of this script added
    old = set(json.load(open(STATE))["uuids"]) if os.path.exists(STATE) else set()
    if old:
        keep = [c for c in pcb[1:]
                if not (isinstance(c, Node) and c.tag in ("segment", "via")
                        and node_uuid(c) in old)]
        removed = len(pcb) - 1 - len(keep)
        pcb[1:] = keep
        print("removed %d previously generated items" % removed)

    plan = build_plan(pins)
    nets = assign_corridors(plan)

    # 2. sanity: escape order and corridor order must agree (planarity)
    order = {"C": 0, "D": 1, "E": 2, "F": 3}
    nets.sort(key=lambda n: (-n["pin_col"], order[n["pin_row"]]))
    assert len(nets) == 192
    for a, b in zip(nets, nets[1:]):
        assert b["corridor"] > a["corridor"], (a["net"], b["net"])
        ea = pins[(a["pin_row"], a["pin_col"])][0] + ESCAPE_DX[a["pin_row"]]
        eb = pins[(b["pin_row"], b["pin_col"])][0] + ESCAPE_DX[b["pin_row"]]
        assert eb > ea + 1e-9, (a["net"], b["net"])

    validate(pcb, plan, nets)

    # 3. place every LED
    want = {}
    for e in plan:
        wx, wy = well_x(e["col"]), well_y(e["row"])
        want["D%d" % e["well"]] = (wx + UV_DX, wy + UV_DY, UV_ROT)
        want["D%d" % (e["well"] + 96)] = (wx + GF_DX, wy + GF_DY, GF_ROT)
    moved = 0
    for fp in pcb.find_all("footprint"):
        ref = fp_ref(fp)
        if ref not in want:
            continue
        x, y, rot = want[ref]
        at = fp.find("at")
        before = (str(at[1]), str(at[2]))
        at[1], at[2] = fmt(x), fmt(y)
        if len(at) > 3:
            at[3] = fmt(rot)
        if before != (at[1], at[2]):
            moved += 1
    print("placed 192 LEDs (%d moved)" % moved)

    # 4. route every net that has no track yet
    have = {str(s.value("net")) for s in pcb.find_all("segment")}
    via_at = {(str(v.value("net")), round(float(v.find("at")[1]), 3),
               round(float(v.find("at")[2]), 3)) for v in pcb.find_all("via")}
    new, added_uuids = [], []
    skipped = []
    for n in nets:
        if n["net"] in have:
            skipped.append(n["net"])
            continue
        pts, pad, xe = route(n, pins)
        for x, y in (pad, pts[-1]):
            key = (n["net"], round(x, 3), round(y, 3))
            if key in via_at:
                continue
            via_at.add(key)
            v = make_via(x, y, n["net"], "via|%s|%.3f|%.3f" % key)
            new.append(v)
            added_uuids.append(node_uuid(v))
        for i, (a, b) in enumerate(zip(pts, pts[1:])):
            s = make_segment(a[0], a[1], b[0], b[1], n["net"],
                             "seg|%s|%d" % (n["net"], i))
            new.append(s)
            added_uuids.append(node_uuid(s))
    print("kept %d hand-routed nets: %s" % (len(skipped), ", ".join(skipped)))
    print("routed %d nets: %d segments, %d vias"
          % (192 - len(skipped),
             sum(1 for n in new if n.tag == "segment"),
             sum(1 for n in new if n.tag == "via")))
    pcb.extend(new)

    # 5. verify, then write
    bad, old_bad = check_clearances(pcb, set(added_uuids))
    for b in old_bad:
        print("  pre-existing: %-12s %-8s %-8s %s %s  d=%s need=%s" % b)
    if bad:
        print("\n%d CLEARANCE VIOLATIONS:" % len(bad))
        for b in bad[:40]:
            print("  %-12s %-8s %-8s %s %s  d=%s need=%s" % b)
        raise SystemExit("not written")
    print("clearance check clean (%.2f mm rule)" % CLEARANCE)

    ks.save(pcb, PCB)
    json.dump({"uuids": added_uuids}, open(STATE, "w"), indent=1)
    print("wrote %s" % PCB)


if __name__ == "__main__":
    main()
