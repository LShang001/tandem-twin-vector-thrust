"""Compare the baseline and v2-clamp maneuver matrices."""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

BASE = r"C:\Users\12631\AppData\Local\Temp\man2\results.jsonl"
V2 = r"C:\Users\12631\AppData\Local\Temp\man_v3\results.jsonl"
OUT = r"D:\纵列双发矢量推力飞行器\TandemVec-EKF-Report\figs\nav_maneuver_v3_compare.png"
CONDS = ["nominal", "outage", "wind", "both"]
DIRS = ["N", "E", "S", "W", "NE"]
DISTS = [0, 3, 10, 25, 50]


def load(path):
    rows = []
    for line in open(path, encoding="utf-8"):
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return rows


def nf(r):
    p = r.get("pos_rms")
    return bool(p is None or p != p or p > 3.0 or (r.get("pos_max") or 0) > 10.0)


def sf(r):
    return bool(abs((r.get("final_alt") or 0) - 10.0) > 3.0
                or (r.get("max_roll") or 0) > 45.0)


def r90(r):
    return bool((r.get("max_roll") or 0) > 90.0)


def counts(rows, cond):
    g = [r for r in rows if r["cond"] == cond]
    return (len(g), sum(nf(r) for r in g), sum(sf(r) for r in g), sum(r90(r) for r in g))


def main():
    base = load(BASE)
    v2 = load(V2)
    print("base", len(base), "v3", len(v2))
    fig, ax = plt.subplots(2, 2, figsize=(14, 9))
    x = np.arange(len(CONDS))
    w = .36

    def series(fn):
        b = [sum(fn(r) for r in base if r["cond"] == c) for c in CONDS]
        o = [sum(fn(r) for r in v2 if r["cond"] == c) for c in CONDS]
        return b, o

    for j, (fn, title) in enumerate([(sf, "Stability failures"),
                                     (r90, "Roll > 90 deg")]):
        b, o = series(fn)
        a = ax[0, j]
        a.bar(x - .18, b, w, label="baseline", color="indianred")
        a.bar(x + .18, o, w, label="v3 cap", color="steelblue")
        for i, (bv, ov) in enumerate(zip(b, o)):
            a.text(i - .18, bv + .4, str(bv), ha="center", fontsize=8)
            a.text(i + .18, ov + .4, str(ov), ha="center", fontsize=8)
        a.set_xticks(x, CONDS)
        a.set_ylabel("runs / 840")
        a.set_title(title)
        a.legend(fontsize=8)
        a.grid(alpha=.25, axis="y")

    xx = np.arange(len(DIRS))
    b = [sum(sf(r) for r in base if r["cond"] == "outage" and r["dir"] == dv) for dv in DIRS]
    o = [sum(sf(r) for r in v2 if r["cond"] == "outage" and r["dir"] == dv) for dv in DIRS]
    ax[1, 0].bar(xx - .18, b, .36, label="baseline", color="indianred")
    ax[1, 0].bar(xx + .18, o, .36, label="v3", color="steelblue")
    for i, (a, c) in enumerate(zip(b, o)):
        ax[1, 0].text(i - .18, a + .3, str(a), ha="center", fontsize=8)
        ax[1, 0].text(i + .18, c + .3, str(c), ha="center", fontsize=8)
    ax[1, 0].set_xticks(xx, DIRS)
    ax[1, 0].set_ylabel("stability failures / 160-200")
    ax[1, 0].set_title("Outage: direction dependence")
    ax[1, 0].legend(fontsize=8)
    ax[1, 0].grid(alpha=.25, axis="y")

    mat = np.zeros((len(DIRS), len(DISTS)))
    for i, dv in enumerate(DIRS):
        for j, d in enumerate(DISTS):
            g = [r for r in v2 if r["cond"] == "both" and r["dir"] == dv
                 and int(r["dist"]) == d]
            mat[i, j] = (100.0 * sum(nf(r) or sf(r) for r in g) / len(g)) if g else np.nan
    im = ax[1, 1].imshow(mat, cmap="Reds", vmin=0, vmax=10)
    ax[1, 1].set_xticks(range(len(DISTS)), [str(d) for d in DISTS])
    ax[1, 1].set_yticks(range(len(DIRS)), DIRS)
    ax[1, 1].set_xlabel("commanded leg [m]")
    ax[1, 1].set_title("v3: any-failure rate [%], outage+wind")
    for i in range(len(DIRS)):
        for j in range(len(DISTS)):
            if not np.isnan(mat[i, j]):
                ax[1, 1].text(j, i, f"{mat[i, j]:.0f}", ha="center", va="center",
                              fontsize=8,
                              color="white" if mat[i, j] > 6 else "k")
    fig.colorbar(im, ax=ax[1, 1], fraction=.046)
    fig.suptitle("Baseline vs v3 hard bias cap: 3360-run step-maneuver matrix")
    fig.tight_layout(rect=(0, 0, 1, .94))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, dpi=150)
    print("saved", OUT)
    for c in CONDS:
        print(c, "base", counts(base, c), "v3", counts(v2, c))


if __name__ == "__main__":
    main()
