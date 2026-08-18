"""Report the current state of the board: LED placement, J1 pins, existing tracks."""
import math, os, sys, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kicad_sexpr as ks

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
pcb = ks.load(os.path.join(ROOT, "led_board.kicad_pcb"))


def fp_ref(fp):
    for p in fp.find_all("property"):
        if p[1] == "Reference":
            return str(p[2])


def rot(r, x, y):
    a = math.radians(r)
    return (x * math.cos(a) + y * math.sin(a), -x * math.sin(a) + y * math.cos(a))


def pad_net(pad):
    n = pad.find("net")
    return str(n[1]) if n is not None and len(n) > 1 else None


leds, j1 = {}, None
for fp in pcb.find_all("footprint"):
    ref = fp_ref(fp)
    at = fp.find("at")
    x, y = float(at[1]), float(at[2])
    r = float(at[3]) if len(at) > 3 else 0.0
    lib = str(fp[1])
    if ref == "J1":
        j1 = (fp, x, y, r)
        continue
    pads = {}
    for pad in fp.find_all("pad"):
        pa = pad.find("at")
        dx, dy = rot(r, float(pa[1]), float(pa[2]))
        pads[str(pad[1])] = (round(x + dx, 4), round(y + dy, 4), pad_net(pad))
    leds[ref] = dict(x=x, y=y, rot=r, lib=lib, pads=pads)

print("footprints: %d LEDs + J1" % len(leds))
libs = collections.Counter(v["lib"] for v in leds.values())
for k, v in libs.items():
    print("   lib %-40s x%d" % (k, v))

# distinct positions -> is anything unplaced / stacked?
pos = collections.Counter((v["x"], v["y"]) for v in leds.values())
print("distinct (x,y): %d ; positions used >1x: %d"
      % (len(pos), sum(1 for c in pos.values() if c > 1)))
for p, c in pos.most_common(4):
    print("   %s x%d" % (p, c))

print("\n--- LEDs sorted by position (first 40) ---")
for ref, v in sorted(leds.items(), key=lambda kv: (kv[1]["x"], kv[1]["y"]))[:40]:
    nets = ",".join("%s=%s" % (k, n) for k, (_, _, n) in sorted(v["pads"].items()))
    print("  %-6s (%9.4f,%9.4f) rot%7.1f %-22s %s"
          % (ref, v["x"], v["y"], v["rot"], v["lib"][:22], nets))

print("\n--- D1..D4, D97..D100 ---")
for ref in ["D1", "D2", "D3", "D4", "D97", "D98", "D99", "D100"]:
    if ref in leds:
        v = leds[ref]
        nets = ",".join("%s=%s" % (k, n) for k, (_, _, n) in sorted(v["pads"].items()))
        print("  %-5s (%9.4f,%9.4f) rot%7.1f %-24s %s"
              % (ref, v["x"], v["y"], v["rot"], v["lib"][:24], nets))
        for pn, (px, py, n) in sorted(v["pads"].items()):
            print("        pad %-3s (%9.4f,%9.4f) net=%s" % (pn, px, py, n))

fp, jx, jy, jr = j1
print("\n--- J1 at (%s,%s) rot %s  lib=%s" % (jx, jy, jr, str(fp[1])))
pins = {}
for pad in fp.find_all("pad"):
    name = str(pad[1])
    if len(name) < 2 or name[0] not in "ABCDEF" or not name[1:].isdigit():
        continue
    pa = pad.find("at")
    dx, dy = rot(jr, float(pa[1]), float(pa[2]))
    pins[name] = (round(jx + dx, 4), round(jy + dy, 4), pad_net(pad))
rows = sorted({n[0] for n in pins})
print("pad rows: %s ; count %d" % (rows, len(pins)))
for rw in rows:
    ys = sorted({pins[p][1] for p in pins if p[0] == rw})
    print("  row %s: y=%s  (n=%d)" % (rw, ys, sum(1 for p in pins if p[0] == rw)))
print("\n  pin cols 46..50, all rows:")
for c in range(46, 51):
    for rw in rows:
        k = "%s%d" % (rw, c)
        if k in pins:
            print("    %-4s (%9.4f,%9.4f) net=%s" % ((k,) + pins[k]))

print("\n--- existing copper ---")
segs = pcb.find_all("segment")
vias = pcb.find_all("via")
bylayer = collections.Counter(str(s.value("layer")) for s in segs)
print("segments %d %s ; vias %d" % (len(segs), dict(bylayer), len(vias)))
nets = collections.Counter(str(s.value("net")) for s in segs)
print("nets with tracks (%d): %s" % (len(nets), sorted(nets)))
