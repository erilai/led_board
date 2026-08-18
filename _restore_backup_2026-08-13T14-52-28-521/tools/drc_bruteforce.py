"""Independent brute-force B.Cu clearance check. No bucketing, no shared geometry code.

Deliberately re-derives everything from the file so a bug in the router's own
checker cannot hide here. Compares every copper pair on B.Cu: O(n^2).
"""
import math, os, sys, itertools, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kicad_sexpr as ks

CLEAR = 0.10
TOL = 2e-3          # file coords are rounded to 4 dp
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
pcb = ks.load(os.path.join(ROOT, "led_board.kicad_pcb"))

tracks, discs = [], []   # (net, (x1,y1), (x2,y2), halfwidth) / (net, x, y, r)
for s in pcb.find_all("segment"):
    if str(s.value("layer")) != "B.Cu":
        continue
    a, b = s.find("start"), s.find("end")
    tracks.append((str(s.value("net")), (float(a[1]), float(a[2])),
                   (float(b[1]), float(b[2])), float(s.value("width")) / 2))
for v in pcb.find_all("via"):
    a = v.find("at")
    discs.append((str(v.value("net")), float(a[1]), float(a[2]),
                  float(v.value("size")) / 2))

# through-hole / *.Cu pads reach B.Cu too
nofnet = 0
for fp in pcb.find_all("footprint"):
    at = fp.find("at")
    ox, oy = float(at[1]), float(at[2])
    rot = math.radians(float(at[3]) if len(at) > 3 else 0.0)
    for pad in fp.find_all("pad"):
        ly = [str(x) for x in pad.find("layers")[1:]]
        if not any(l in ("*.Cu", "B.Cu") for l in ly):
            continue
        pa = pad.find("at")
        lx, ly2 = float(pa[1]), float(pa[2])
        x = ox + lx * math.cos(rot) + ly2 * math.sin(rot)
        y = oy - lx * math.sin(rot) + ly2 * math.cos(rot)
        sz = pad.find("size")
        assert str(pad[3]) == "circle", "non-circular B.Cu pad needs handling"
        n = pad.find("net")
        if n is not None and len(n) > 1:
            net = str(n[1])
        else:
            nofnet += 1
            net = "!hole%d" % nofnet
        discs.append((net, x, y, float(sz[1]) / 2))

print("B.Cu copper: %d tracks, %d discs (%d unnetted holes)"
      % (len(tracks), len(discs), nofnet))


def seg_pt(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0]-ax)*dx + (p[1]-ay)*dy) / L))
    return math.hypot(p[0] - ax - t*dx, p[1] - ay - t*dy)


def seg_seg(a, b, c, d):
    # proper intersection test first
    def cr(o, p, q):
        return (p[0]-o[0])*(q[1]-o[1]) - (p[1]-o[1])*(q[0]-o[0])
    d1, d2 = cr(c, d, a), cr(c, d, b)
    d3, d4 = cr(a, b, c), cr(a, b, d)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return 0.0
    return min(seg_pt(a, c, d), seg_pt(b, c, d), seg_pt(c, a, b), seg_pt(d, a, b))


bad = []
for (n1, a1, b1, h1), (n2, a2, b2, h2) in itertools.combinations(tracks, 2):
    if n1 == n2:
        continue
    need = CLEAR + h1 + h2
    # cheap reject
    if min(a1[0], b1[0]) - max(a2[0], b2[0]) > need or \
       min(a2[0], b2[0]) - max(a1[0], b1[0]) > need or \
       min(a1[1], b1[1]) - max(a2[1], b2[1]) > need or \
       min(a2[1], b2[1]) - max(a1[1], b1[1]) > need:
        continue
    dist = seg_seg(a1, b1, a2, b2)
    if dist < need - TOL:
        bad.append(("track/track", n1, n2, a1, a2, round(dist, 4), round(need, 4)))

for net, x, y, r in discs:
    for n2, a, b, h in tracks:
        if n2 == net:
            continue
        need = CLEAR + r + h
        dist = seg_pt((x, y), a, b)
        if dist < need - TOL:
            bad.append(("disc/track", net, n2, (x, y), a, round(dist, 4), round(need, 4)))

for (n1, x1, y1, r1), (n2, x2, y2, r2) in itertools.combinations(discs, 2):
    if n1 == n2:
        continue
    need = CLEAR + r1 + r2
    dist = math.hypot(x1 - x2, y1 - y2)
    if dist < need - TOL:
        bad.append(("disc/disc", n1, n2, (x1, y1), (x2, y2),
                    round(dist, 4), round(need, 4)))

print("violations: %d" % len(bad))
for v in sorted(bad, key=lambda v: v[5])[:25]:
    print("  %-12s %-9s %-9s %s %s d=%s need=%s" % v)

# tightest legal gaps, as a margin report
gaps = []
for (n1, a1, b1, h1), (n2, a2, b2, h2) in itertools.combinations(tracks, 2):
    if n1 == n2:
        continue
    if min(a1[0], b1[0]) - max(a2[0], b2[0]) > 1.0 or \
       min(a2[0], b2[0]) - max(a1[0], b1[0]) > 1.0 or \
       min(a1[1], b1[1]) - max(a2[1], b2[1]) > 1.0 or \
       min(a2[1], b2[1]) - max(a1[1], b1[1]) > 1.0:
        continue
    gaps.append((round(seg_seg(a1, b1, a2, b2) - h1 - h2, 4), n1, n2))
gaps.sort()
print("\ntightest track-to-track edge gaps (rule = %.2f mm):" % CLEAR)
for g in gaps[:8]:
    print("   %.4f mm  %s vs %s" % g)
dgaps = sorted((round(seg_pt((x, y), a, b) - r - h, 4), net, n2)
               for net, x, y, r in discs for n2, a, b, h in tracks
               if net != n2 and seg_pt((x, y), a, b) - r - h < 1.0)
print("tightest via/hole-to-track edge gaps:")
for g in dgaps[:8]:
    print("   %.4f mm  %s vs %s" % g)

sys.exit(1 if bad else 0)
