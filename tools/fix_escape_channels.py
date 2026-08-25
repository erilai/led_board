r"""Widen the J1 escape channels to satisfy the 0.25 mm hole-to-copper rule.

Why
---
The escape channels were laid out for a 0.10 mm copper rule: four channels in
the 1.27 mm pin pitch at offsets C -0.950, D -0.635, E -0.320, F 0.0 from their
pin column's axis.  Row F rides its own column's axis (it is the bottom row, so
it descends without passing another pad), and C/D/E sit in the gap to the left.

With hole-to-copper at 0.25 mm a 0.10 mm track must keep its centreline

    0.10 (via drill 0.20 -> radius) + 0.25 + 0.05 (track half width) = 0.40 mm

from every connector via centre.  Two of the four channels broke that:

    C  0.950 from its own axis, but only 1.27-0.950 = 0.320 from the axis of
       the pin column to its left  -> 0.17 mm gap
    E  0.320 from its own axis                                -> 0.17 mm gap

The 1.27 mm pitch leaves a 1.27 - 2*0.40 = 0.47 mm window for C/D/E, which is
just enough for three channels at the 0.20 mm minimum pitch.  New offsets:

    C -0.855   D -0.635 (unchanged)   E -0.415   F 0.0 (unchanged)

giving 0.415 mm to both bounding axes (0.015 mm of margin on the 0.40 mm
requirement) and 0.220 mm channel pitch (0.020 mm of margin on 0.20 mm).

How, without disturbing anything else
-------------------------------------
Each net leaves its pad on a 45 deg step, drops down its channel to the
fan-out line, then runs one long 45 deg diagonal to its corridor in the LED
field.  Moving a channel sideways would normally drag that diagonal with it and
ripple through the whole fan - the diagonals are only 0.22 mm apart.

So this keeps every diagonal on exactly the line it already occupies and only
slides its *start point* along that line: shifting the channel by dx moves the
corner to (xe+dx, y -/+ dx), which is still on the same 45 deg line.  The
diagonal's far end - and therefore the corridor, the jog and the LED via - do
not move at all.  Three segments per net change; nothing else in the fan does.

Column 1 (pin columns 46..49) is hand-routed with its own irregular offsets and
a much tighter fan-out, and is handled by fix_col1_escapes.py instead.

Usage:  python tools/fix_escape_channels.py [--write]
"""
from __future__ import annotations

import argparse
import collections
import math
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kicad_sexpr as ks
import place_and_route_leds as R

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB = os.path.join(ROOT, "led_board.kicad_pcb")

NEW_ESCAPE_DX = {"C": -0.855, "D": -0.635, "E": -0.415, "F": 0.0}
COL1_PIN_COLS = {46, 47, 48, 49}        # board column 1, routed by hand

VIA_HOLE_R = 0.10                       # connector vias are 0.30/0.20
TRACK_HW = 0.05
NEED_FROM_AXIS = VIA_HOLE_R + 0.25 + TRACK_HW       # 0.40


def k(p):
    return (round(p[0], 4), round(p[1], 4))


def seg_key(a, b):
    return tuple(sorted([k(a), k(b)]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--out", help="write here instead of the board (implies "
                                  "--write, no backup)")
    args = ap.parse_args()

    pcb = ks.load(PCB)
    pins = R.read_connector(pcb)
    plan = R.build_plan(pins)
    nets = R.assign_corridors(plan)
    order = {"C": 0, "D": 1, "E": 2, "F": 3}
    nets.sort(key=lambda n: (-n["pin_col"], order[n["pin_row"]]))

    # index every B.Cu segment by its endpoint pair so the old geometry can be
    # located and edited in place
    index = {}
    for s in pcb.find_all("segment"):
        if str(s.value("layer")) != "B.Cu":
            continue
        a, b = s.find("start"), s.find("end")
        pa = (float(a[1]), float(a[2]))
        pb = (float(b[1]), float(b[2]))
        index.setdefault(seg_key(pa, pb), []).append(s)

    edited, skipped, missing = 0, [], []
    for n in nets:
        row, pcol = n["pin_row"], n["pin_col"]
        if pcol in COL1_PIN_COLS:
            skipped.append(n["net"])
            continue
        dx_old = R.ESCAPE_DX[row]
        dx_new = NEW_ESCAPE_DX[row]
        if abs(dx_old - dx_new) < 1e-9:
            continue

        px, py, _ = pins[(row, pcol)]
        xe_old = round(px + dx_old, 4)
        xe_new = round(px + dx_new, 4)
        xc = n["corridor"]
        bus = R.BUS_Y
        yb = round(bus + abs(xe_old - xc), 4)

        # the diagonal keeps its line; only its start point slides along it
        down_left = xc < xe_old
        if down_left:
            y_end = round(xe_old + bus - xe_new, 4)
        else:
            y_end = round(xe_new - (xe_old - bus), 4)

        y_step_old = round(py + abs(px - xe_old), 4)
        y_step_new = round(py + abs(px - xe_new), 4)

        old = [((px, py), (xe_old, y_step_old)),
               ((xe_old, y_step_old), (xe_old, bus)),
               ((xe_old, bus), (xc, yb))]
        new = [((px, py), (xe_new, y_step_new)),
               ((xe_new, y_step_new), (xe_new, y_end)),
               ((xe_new, y_end), (xc, yb))]

        nodes = []
        for a, b in old:
            got = index.get(seg_key(a, b))
            if not got:
                missing.append((n["net"], row, pcol, seg_key(a, b)))
                nodes = None
                break
            nodes.append(got[0])
        if nodes is None:
            continue

        for node, (a, b) in zip(nodes, new):
            dx, dy = abs(b[0] - a[0]), abs(b[1] - a[1])
            assert dx < 1e-9 or dy < 1e-9 or abs(dx - dy) < 1e-6, \
                ("not 45/ortho", n["net"], a, b)
            st, en = node.find("start"), node.find("end")
            st[1], st[2] = R.fmt(a[0]), R.fmt(a[1])
            en[1], en[2] = R.fmt(b[0]), R.fmt(b[1])
        # the reshaped diagonal must be collinear with the one it replaces
        oa, ob = old[2]
        na, nb = new[2]
        cross = ((ob[0]-oa[0]) * (nb[1]-oa[1]) - (ob[1]-oa[1]) * (nb[0]-oa[0]))
        assert abs(cross) < 1e-6, ("diagonal moved", n["net"], cross)
        assert abs((na[0]-oa[0]) * (ob[1]-oa[1])
                   - (na[1]-oa[1]) * (ob[0]-oa[0])) < 1e-6, \
            ("corner off line", n["net"])
        edited += 1

    print("channels re-cut: %d nets (%d segments)" % (edited, edited * 3))
    print("column 1 left to fix_col1_escapes.py: %d nets" % len(skipped))
    if missing:
        print("\n%d nets whose old geometry was not found:" % len(missing))
        for m in missing[:15]:
            print("   ", m)

    # report the resulting geometry margins
    print("\nnew offsets and clearance to the bounding via axes:")
    for row in "CDEF":
        o = NEW_ESCAPE_DX[row]
        own, left = abs(o), abs(1.27 + o)
        note = "own axis (same net)" if row == "F" else "%.3f" % own
        print("   %s  dx=%+.3f   to own axis %s   to left axis %.3f   "
              "(need %.2f)" % (row, o, note, left, NEED_FROM_AXIS))

    if args.out:
        ks.save(pcb, args.out)
        print("\nwrote %s" % args.out)
    elif args.write:
        bak = PCB + ".bak_escape"
        if not os.path.exists(bak):
            shutil.copy2(PCB, bak)
            print("\nbacked up -> %s" % bak)
        ks.save(pcb, PCB)
        print("wrote %s" % PCB)
    else:
        print("\ndry run; pass --write to save")
    return 0


if __name__ == "__main__":
    sys.exit(main())
