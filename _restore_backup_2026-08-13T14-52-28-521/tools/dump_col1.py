"""Print the hand-routed column 1 tracks, per net, ordered pad -> LED."""
import os, sys, math, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kicad_sexpr as ks

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
pcb = ks.load(os.path.join(ROOT, "led_board.kicad_pcb"))

segs = collections.defaultdict(list)
for s in pcb.find_all("segment"):
    st, en = s.find("start"), s.find("end")
    segs[str(s.value("net"))].append(
        ((float(st[1]), float(st[2])), (float(en[1]), float(en[2])),
         float(s.value("width")), str(s.value("layer"))))
vias = collections.defaultdict(list)
for v in pcb.find_all("via"):
    at = v.find("at")
    vias[str(v.value("net"))].append(
        (float(at[1]), float(at[2]), float(v.value("size")), float(v.value("drill")),
         [str(x) for x in v.find("layers")[1:]]))

order = ["/93A", "/93B", "/94A", "/94B", "/95A", "/95B", "/96A", "/96B",
         "/69A", "/69B", "/70A", "/70B", "/71A", "/71B", "/72A", "/72B"]
for net in order:
    ss = segs[net]
    print("\n=== %s : %d segments, %d vias  widths=%s layers=%s" % (
        net, len(ss), len(vias[net]),
        sorted({s[2] for s in ss}), sorted({s[3] for s in ss})))
    for x, y, sz, dr, ly in vias[net]:
        print("    via (%8.4f,%8.4f) size=%s drill=%s %s" % (x, y, sz, dr, ly))
    # chain the segments end-to-end starting from the highest point (connector)
    pts = collections.defaultdict(list)
    for a, b, w, l in ss:
        pts[a].append(b)
        pts[b].append(a)
    ends = [p for p, v in pts.items() if len(v) == 1]
    start = min(ends, key=lambda p: p[1]) if ends else min(pts)
    seen, cur, prev = set(), start, None
    chain = [cur]
    while True:
        nxt = [p for p in pts[cur] if p != prev and (cur, p) not in seen]
        if not nxt:
            break
        seen.add((cur, nxt[0]))
        seen.add((nxt[0], cur))
        prev, cur = cur, nxt[0]
        chain.append(cur)
    for i, (a, b) in enumerate(zip(chain, chain[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        kind = ("vert" if abs(dx) < 1e-9 else "horz" if abs(dy) < 1e-9
                else "45" if abs(abs(dx) - abs(dy)) < 1e-6 else "ANGLE%.2f" % math.degrees(math.atan2(dy, dx)))
        print("   %2d (%8.4f,%8.4f)->(%8.4f,%8.4f) d=(%+7.4f,%+7.4f) %s"
              % (i, a[0], a[1], b[0], b[1], dx, dy, kind))
    if len(chain) - 1 != len(ss):
        print("    !! chain covered %d of %d segments (branches?)" % (len(chain) - 1, len(ss)))
        for a, b, w, l in ss:
            print("       seg (%8.4f,%8.4f)->(%8.4f,%8.4f)" % (a[0], a[1], b[0], b[1]))
