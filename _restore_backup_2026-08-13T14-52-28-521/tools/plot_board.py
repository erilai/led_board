"""Render B.Cu copper + LED placement to PNG so the result can be eyeballed."""
import os, sys, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import kicad_sexpr as ks
import place_and_route_leds as P

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
pcb = ks.load(os.path.join(ROOT, "led_board.kicad_pcb"))
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plots")
os.makedirs(OUT, exist_ok=True)

hand = set()
bak = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "led_board.kicad_pcb.before_autoroute")
if os.path.exists(bak):
    hand = {str(s.value("net")) for s in ks.load(bak).find_all("segment")}


def draw(ax):
    # plate artwork
    for n in pcb.find_all("gr_circle"):
        c, e = n.find("center"), n.find("end")
        cx, cy = float(c[1]), float(c[2])
        r = math.hypot(float(e[1]) - cx, float(e[2]) - cy)
        ax.add_patch(Circle((cx, cy), r, fill=False, ec="0.85", lw=0.4, zorder=0))
    for n in pcb.find_all("gr_line"):
        s, e = n.find("start"), n.find("end")
        ax.plot([float(s[1]), float(e[1])], [float(s[2]), float(e[2])],
                color="0.88", lw=0.4, zorder=0)
    # tracks
    for s in pcb.find_all("segment"):
        st, en = s.find("start"), s.find("end")
        net = str(s.value("net"))
        col = "#d62728" if net in hand else "#1f77b4"
        ax.plot([float(st[1]), float(en[1])], [float(st[2]), float(en[2])],
                color=col, lw=float(s.value("width")) * 3.0, zorder=2,
                solid_capstyle="round")
    # vias
    for v in pcb.find_all("via"):
        a = v.find("at")
        ax.add_patch(Circle((float(a[1]), float(a[2])),
                            float(v.value("size")) / 2, fc="#2ca02c",
                            ec="none", zorder=3))
    # footprint pads (F.Cu)
    for fp in pcb.find_all("footprint"):
        a = fp.find("at")
        ox, oy = float(a[1]), float(a[2])
        frot = float(a[3]) if len(a) > 3 else 0.0
        ref = P.fp_ref(fp)
        for pad in fp.find_all("pad"):
            pa = pad.find("at")
            dx, dy = P.fp_rotate(frot, float(pa[1]), float(pa[2]))
            sz = pad.find("size")
            w, h = float(sz[1]), float(sz[2])
            prot = (float(pa[3]) if len(pa) > 3 else 0.0) - frot
            if abs(math.sin(math.radians(prot))) > 0.5:
                w, h = h, w
            if abs(math.sin(math.radians(frot))) > 0.5:
                w, h = h, w
            x, y = ox + dx, oy + dy
            fc = "#ffbb78" if ref == "J1" else "#c5b0d5"
            ax.add_patch(Rectangle((x - w / 2, y - h / 2), w, h, fc=fc,
                                   ec="none", alpha=.85, zorder=1))
    ax.set_aspect("equal")
    ax.invert_yaxis()


def save(name, xlim, ylim, w=22, labels=None):
    fig, ax = plt.subplots(figsize=(w, w * (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])))
    draw(ax)
    for t in (labels or []):
        ax.annotate(t[2], (t[0], t[1]), color="k", fontsize=5, ha="center")
    ax.set_xlim(*xlim)
    ax.set_ylim(ylim[1], ylim[0])
    ax.set_title(name)
    fig.tight_layout()
    p = os.path.join(OUT, name + ".png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    print("wrote", p)


labels = []
pins = P.read_connector(pcb)
for e in P.build_plan(pins):
    labels.append((P.well_x(e["col"]), P.well_y(e["row"]) + 4.6, str(e["well"])))

save("01_whole_board", (100, 215), (35, 152), 26, labels)
save("02_connector_fanout", (118, 194), (37, 80), 26)
save("03_columns_1_3", (100, 136), (44, 152), 14, labels)
save("04_columns_6_8", (145, 190), (44, 152), 14, labels)
save("05_columns_10_12", (178, 215), (44, 152), 14, labels)
save("06_col7_flipped_detail", (158, 172), (76, 130), 12, labels)
save("07_connector_escapes", (124, 141), (42, 56), 20)

# tight zooms: a non-flipped well (col 2) vs a flipped well (col 3)
save("08_zoom_col2_nonflipped", (112.0, 121.5), (76.0, 96.0), 9, labels)
save("09_zoom_col3_flipped", (121.0, 130.5), (76.0, 96.0), 9, labels)
save("10_zoom_col12_flipped", (202.5, 212.0), (76.0, 96.0), 9, labels)
