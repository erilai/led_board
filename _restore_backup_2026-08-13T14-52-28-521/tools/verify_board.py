"""Independent check of the written board.

1. nothing outside copper changed vs the pre-run backup
2. every one of the 192 signal nets is electrically continuous:
   connector F.Cu pad -> via -> B.Cu tracks -> via -> LED F.Cu pad 1
3. no stray / zero-length / off-angle segments
4. clearance re-check over *all* copper, not just what was added
"""
import math, os, sys, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kicad_sexpr as ks
from kicad_sexpr import Node
import place_and_route_leds as P

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NEW = os.path.join(ROOT, "led_board.kicad_pcb")
OLD = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "led_board.kicad_pcb.before_autoroute")

new, old = ks.load(NEW), ks.load(OLD)
fail = []


def note(ok, msg):
    print(("  ok   " if ok else "  FAIL ") + msg)
    if not ok:
        fail.append(msg)


# ---- 1. structural diff -----------------------------------------------------
print("1. structure")


def strip(pcb):
    """Everything that is not a track, plus footprints with `at` normalised."""
    out = []
    for c in pcb[1:]:
        if isinstance(c, Node) and c.tag in ("segment", "via"):
            continue
        out.append(ks.dumps(c))
    return out


def fp_only(pcb):
    d = {}
    for fp in pcb.find_all("footprint"):
        d[P.fp_ref(fp)] = ks.dumps(fp)
    return d


so, sn = strip(old), strip(new)
note(len(so) == len(sn), "top-level non-copper item count %d -> %d" % (len(so), len(sn)))
fo, fn = fp_only(old), fp_only(new)
note(set(fo) == set(fn), "same %d footprint refs" % len(fn))
# non-footprint, non-copper items must be byte-identical
no = [s for s in so if not s.startswith("(footprint")]
nn = [s for s in sn if not s.startswith("(footprint")]
note(no == nn, "%d non-footprint non-copper items unchanged (outline, plate artwork, "
               "layers, setup, nets)" % len(no))
# footprints may only differ in their `at` line
diff_ref, bad_ref = [], []
for r in fn:
    if fo[r] == fn[r]:
        continue
    diff_ref.append(r)
    a = [l for l in fo[r].splitlines() if not l.strip().startswith("(at ")]
    b = [l for l in fn[r].splitlines() if not l.strip().startswith("(at ")]
    if a != b:
        bad_ref.append(r)
note(not bad_ref, "%d footprints changed, and only in their (at ...) line%s"
     % (len(diff_ref), "" if not bad_ref else " -- EXCEPT " + str(bad_ref[:5])))

# ---- 2. placement -----------------------------------------------------------
print("2. placement")
pins = P.read_connector(new)
plan = P.build_plan(pins)
nets = P.assign_corridors(plan)
at = {}
for fp in new.find_all("footprint"):
    a = fp.find("at")
    at[P.fp_ref(fp)] = (round(float(a[1]), 4), round(float(a[2]), 4),
                        round(float(a[3]) if len(a) > 3 else 0.0, 4))
bad = []
for e in plan:
    wx, wy = P.well_x(e["col"]), P.well_y(e["row"])
    for ref, w in (("D%d" % e["well"], (wx + P.UV_DX, wy + P.UV_DY, P.UV_ROT)),
                   ("D%d" % (e["well"] + 96), (wx + P.GF_DX, wy + P.GF_DY, P.GF_ROT))):
        if at[ref] != tuple(round(v, 4) for v in w):
            bad.append((ref, at[ref], w))
note(not bad, "all 192 LEDs on the well grid at column-1 offsets %s" % (bad[:3],))
note(len({(v[0], v[1]) for k, v in at.items() if k != "J1"}) == 192,
     "192 distinct LED positions (none stacked)")
rots = collections.Counter(v[2] for k, v in at.items() if k != "J1")
note(dict(rots) == {P.UV_ROT: 96, P.GF_ROT: 96},
     "rotations untouched: %s" % dict(rots))
# two different LEDs per well
per_well = collections.defaultdict(set)
for ref, v in at.items():
    if ref == "J1":
        continue
    n = int(ref[1:])
    per_well[n if n <= 96 else n - 96].add("UV" if n <= 96 else "GF")
note(len(per_well) == 96 and all(v == {"UV", "GF"} for v in per_well.values()),
     "each of the 96 wells has one UV + one GF LED")

# ---- 3. connectivity --------------------------------------------------------
print("3. connectivity")
segs = collections.defaultdict(list)
for s in new.find_all("segment"):
    st, en = s.find("start"), s.find("end")
    a = (round(float(st[1]), 4), round(float(st[2]), 4))
    b = (round(float(en[1]), 4), round(float(en[2]), 4))
    segs[str(s.value("net"))].append((a, b, str(s.value("layer"))))
vias = collections.defaultdict(list)
for v in new.find_all("via"):
    a = v.find("at")
    vias[str(v.value("net"))].append((round(float(a[1]), 4), round(float(a[2]), 4)))

# F.Cu pad rectangles per net (connector + LED pad 1)
pads = collections.defaultdict(list)
for fp in new.find_all("footprint"):
    a = fp.find("at")
    ox, oy = float(a[1]), float(a[2])
    frot = float(a[3]) if len(a) > 3 else 0.0
    for pad in fp.find_all("pad"):
        ly = [str(x) for x in pad.find("layers")[1:]]
        if not any(l in ("*.Cu", "F.Cu") for l in ly):
            continue
        n = P.pad_net(pad)
        if not n or not n.startswith("/") or n in ("/V_A", "/V_B"):
            continue
        pa = pad.find("at")
        dx, dy = P.fp_rotate(frot, float(pa[1]), float(pa[2]))
        sz = pad.find("size")
        w, h = float(sz[1]), float(sz[2])
        prot = (float(pa[3]) if len(pa) > 3 else 0.0) - frot
        if abs(math.sin(math.radians(prot))) > 0.5:
            w, h = h, w
        if abs(math.sin(math.radians(frot))) > 0.5:
            w, h = h, w
        pads[n].append((P.fp_ref(fp), ox + dx, oy + dy, w, h))

signal = sorted(pads, key=lambda s: (int(s[1:-1]), s[-1]))
note(len(signal) == 192, "192 signal nets on pads: %d" % len(signal))

HAND = {str(s.value("net")) for s in old.find_all("segment")}   # column 1, by hand
print("   (%d nets were already routed by hand and are reported separately)"
      % len(HAND))

unrouted, broken, offangle, stray = [], [], [], []
hand_offangle, hand_stray = [], []
for net in signal:
    ss = segs[net]
    if not ss:
        unrouted.append(net)
        continue
    sl = hand_stray if net in HAND else stray
    ol = hand_offangle if net in HAND else offangle
    for a, b, layer in ss:
        if layer != "B.Cu":
            sl.append((net, layer, a))
        if a == b:
            sl.append((net, "zero-length", a))
            continue
        dx, dy = abs(b[0] - a[0]), abs(b[1] - a[1])
        if not (dx < 1e-6 or dy < 1e-6 or abs(dx - dy) < 1e-6):
            ol.append((net, a, b, round(abs(dx - dy), 6)))
    # union-find over segment endpoints, joined through vias that sit in a pad
    parent = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(x, y):
        parent[find(x)] = find(y)

    for a, b, _ in ss:
        union(a, b)
    # a via bridges F.Cu pad -> B.Cu track when it lies inside the pad
    reached = set()
    for ref, px, py, w, h in pads[net]:
        hit = None
        for vx, vy in vias[net]:
            if abs(vx - px) <= w / 2 + 1e-6 and abs(vy - py) <= h / 2 + 1e-6:
                hit = (vx, vy)
                break
        if hit is None:
            broken.append((net, "pad %s has no via on it" % ref))
            continue
        if hit not in parent:
            broken.append((net, "via at %s touches no track" % (hit,)))
            continue
        reached.add(find(hit))
    if len(reached) > 1:
        broken.append((net, "connector and LED are on %d separate copper "
                            "islands" % len(reached)))

note(not unrouted, "every signal net has copper (%d unrouted)" % len(unrouted))
note(not broken, "every net is continuous pad->via->tracks->via->pad (%d broken) %s"
     % (len(broken), broken[:3]))
note(not offangle, "generated segments all 0/45/90 deg (%d off-angle) %s"
     % (len(offangle), offangle[:3]))
note(not stray, "generated segments all on B.Cu, none zero-length (%d stray) %s"
     % (len(stray), stray[:3]))
for n, a, b, err in hand_offangle:
    print("  note   pre-existing hand route %s: %s->%s is off 45 deg by %s mm"
          % (n, a, b, err))
for n, why, a in hand_stray:
    print("  note   pre-existing hand route %s: %s segment at %s" % (n, why, a))
note(all(len(vias[n]) == 2 for n in signal),
     "exactly 2 vias per net (%d nets differ)"
     % sum(1 for n in signal if len(vias[n]) != 2))

# ---- 4. clearance -----------------------------------------------------------
print("4. clearance (%0.2f mm, all copper on B.Cu)" % P.CLEARANCE)
a, b = P.check_clearances(new, set())
allbad = a + b
note(not allbad, "%d violations" % len(allbad))
for v in allbad[:20]:
    print("     %-12s %-10s %-10s %s %s d=%s need=%s" % v)

print("\n%s" % ("ALL CHECKS PASSED" if not fail else "%d CHECK(S) FAILED" % len(fail)))
sys.exit(1 if fail else 0)
