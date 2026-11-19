"""결과 내보내기: 평면 그림(PNG), DXF, Rhino 3dm, 통계(JSON·CSV·TXT).

내부 계산은 m. DXF·3dm 은 입력 도면과 같은 단위(mm 도면이면 mm)로 써서 원래 도면 위에 겹친다.
"""
import csv
import json
import math
import os

import numpy as np
import shapely
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.ops import unary_union

HERE = os.path.dirname(__file__)
FONT_DIR = os.path.join(HERE, "web", "vendor", "fonts")

KIND_NAME = {"N": "일반", "E": "확장", "C": "경형", "A": "장애인", "V": "전기차"}
LAYERS = {   # 3dm 용 이름: (ACI 색, RGB)
    "P-대지경계": (7, (20, 20, 20)),
    "P-주차영역": (8, (140, 140, 140)),
    "P-코어": (8, (110, 110, 110)),
    "P-경사로": (5, (36, 87, 214)),
    "P-차로": (9, (200, 200, 200)),
    "P-주차_일반": (7, (20, 20, 20)),
    "P-주차_확장": (5, (36, 87, 214)),
    "P-주차_경형": (4, (90, 150, 230)),
    "P-주차_장애인": (5, (20, 60, 170)),
    "P-주차_전기차": (5, (60, 120, 230)),
    "P-기둥": (7, (30, 30, 30)),
    "P-주차번호": (8, (120, 120, 120)),
    "P-문자": (7, (20, 20, 20)),
    "P-슬래브": (9, (215, 215, 215)),
    "P-차량(예시)": (8, (160, 160, 160)),
}
KIND_LAYER = {"N": "P-주차_일반", "E": "P-주차_확장", "C": "P-주차_경형", "A": "P-주차_장애인", "V": "P-주차_전기차"}

# DXF: 예시 도면(지하주차장 예시.dxf)의 레이어 체계
DXF_LAYERS = {   # 이름: (ACI, 선종류)
    "A-SITE": (1, "PHANTOM2"),
    "A-WAL": (3, "Continuous"),
    "A-CON": (2, "Continuous"),
    "A-CEN": (1, "CENTER"),
    "A-SYM": (7, "Continuous"),
    "A-PARK-B": (9, "Continuous"),
    "A-PARK-NO": (8, "Continuous"),
    "A-RAMP": (4, "Continuous"),
    "A-TXT": (7, "Continuous"),
    "A-TXT1": (7, "Continuous"),
    "A-TXT3": (3, "Continuous"),
    "DIM": (7, "Continuous"),
    "SOL 차량동선": (10, "HIDDEN"),
    "SOL 차량동선(부)": (170, "HIDDEN"),
    "SOL 진행방향": (5, "Continuous"),
}
BLOCK_NAME = {"N": "일반주차구획", "E": "확장형주차구획", "C": "경형", "V": "전기자동차", "A": "장애인주차구획"}


def _polys(g):
    if g is None or g.is_empty:
        return []
    if isinstance(g, Polygon):
        return [g]
    return [p for p in shapely.get_parts(g) if isinstance(p, Polygon) and not p.is_empty]


def _rot(v, th):
    a = math.radians(th)
    return (v[0] * math.cos(a) - v[1] * math.sin(a), v[0] * math.sin(a) + v[1] * math.cos(a))


def _stall_place(poly, into, W):
    """구획 → (앞-왼쪽 모서리, 너비 방향 단위벡터, 들어가는 방향 단위벡터). 블록 삽입용."""
    u = into
    cs = list(poly.exterior.coords)[:-1]
    pr = sorted(cs, key=lambda q: q[0] * u[0] + q[1] * u[1])
    fm = ((pr[0][0] + pr[1][0]) / 2, (pr[0][1] + pr[1][1]) / 2)
    v = (u[1], -u[0])
    o = (fm[0] - v[0] * W / 2, fm[1] - v[1] * W / 2)
    return o, v, u


def _arrow_pts(x, y, dx, dy, L=3.0, w=1.4):
    """진행 방향 화살표(가운데 x,y · 길이 L) 꼭짓점."""
    n = math.hypot(dx, dy) or 1.0
    ux, uy = dx / n, dy / n
    vx, vy = -uy, ux
    t = (x + ux * L / 2, y + uy * L / 2)
    b = (x - ux * L / 2, y - uy * L / 2)
    m = (x + ux * L * 0.05, y + uy * L * 0.05)
    s = w * 0.22
    return [t, (m[0] + vx * w / 2, m[1] + vy * w / 2), (m[0] + vx * s, m[1] + vy * s), (b[0] + vx * s, b[1] + vy * s),
            (b[0] - vx * s, b[1] - vy * s), (m[0] - vx * s, m[1] - vy * s), (m[0] - vx * w / 2, m[1] - vy * w / 2)]


def world(res, floor=0):
    """한 층의 도형을 세계 좌표로. floor 0 = B1(장애인 구획 있음)."""
    fr = res["frame"]
    w = fr.to_world
    th = fr.theta
    sts = res["stalls_b1"] if floor == 0 else res["stalls"]
    order = sorted(range(len(sts)), key=lambda i: (sts[i]["row"], sts[i]["x"]))
    stalls = []
    for n, i in enumerate(order, 1):
        s = sts[i]
        wp = w(s["poly"])
        o, v, u = _stall_place(wp, _rot(s["into"], th), s["W"])
        stalls.append(dict(kind=s["kind"], poly=wp, no=n, ev=bool(s.get("ev")), W=s["W"], L=s["L"], o=o, v=v, u=u))
    ramp = res.get("ramp")
    # 격자(기둥 중심선)
    gx, gy = [], []
    g = res.get("grid") or {}
    minx, miny, maxx, maxy = fr.site.bounds
    ext = 4.0
    for x in g.get("xs", []):
        gx.append(w(LineString([(x, miny - ext), (x, maxy + ext)])))
    for y in g.get("ys", []):
        gy.append(w(LineString([(minx - ext, y), (maxx + ext, y)])))
    groups = [(gx, gy)] if gx or gy else []
    if res.get("grid_world"):
        # 구역별 방향: 구역마다 따로 된 격자(이미 세계 좌표). 기호는 대지 밖 끝에 오도록 선 방향을 맞춤
        # (gx 는 끝점 coords[-1], gy 는 시작점 coords[0] 에 기호를 붙임)
        sw = w(fr.site).buffer(0.5)
        def orient(ln_, tail):
            a, b = Point(ln_.coords[0]), Point(ln_.coords[-1])
            out_a, out_b = not sw.contains(a), not sw.contains(b)
            if (tail and out_a and not out_b) or (not tail and out_b and not out_a):
                return LineString(list(ln_.coords)[::-1])
            return ln_
        for grp in res["grid_world"]:
            xs_ = [orient(l_, True) for l_ in grp["x"]]
            ys_ = [orient(l_, False) for l_ in grp["y"]]
            groups.append((xs_, ys_))
            gx += xs_
            gy += ys_
    flows = [(w(LineString([a, b])), k) for a, b, k in res.get("flows", [])]
    dead = [(w(Point(x, y)), sg) for x, y, sg in res.get("dead", [])]
    arrows = []      # 일방 차로 진행 방향 (세계 좌표 점, 방향)
    for x, y, dx, dy in res.get("arrows", []):
        q = w(Point(x, y))
        arrows.append((q.x, q.y, *_rot((dx, dy), th)))
    return dict(
        site=w(fr.site), usable=w(fr.usable), cores=[w(c) for c in fr.cores],
        ramp=w(ramp["poly"]) if ramp is not None else None,
        ramp_mouth=(w(shapely.Point(ramp["mouth_mid"])) if ramp is not None and ramp.get("mouth_mid") else None),
        drive=w(res["drive"]), columns=[w(c) for c in res["columns"]], stalls=stalls,
        grid_x=gx, grid_y=gy, grid_groups=groups, flows=flows, dead=dead, arrows=arrows, theta=th,
    )


def count_table(st):
    """예시 도면의 주차대수표: (구분, 규격, 층별..., 합계)."""
    pf = st["per_floor"]
    rows = [("경차", "2.0 x 3.6", "경형"), ("일반차", "2.5 x 5.0", "일반"), ("확장형", "2.6 x 5.2", "확장"),
            ("전기차", "(일반·확장)", "전기차"), ("장애인", "3.3 x 5.0", "장애인")]
    out = []
    for name, size, key in rows:
        vals = [p[key] for p in pf]
        out.append([name, size] + [str(v) for v in vals] + [str(sum(vals))])
    out.append(["합계", ""] + [str(p["physical"]) for p in pf] + [f"{st['total_physical']:,}대"])
    head = ["구분", "규격"] + [p["floor"] for p in pf] + ["합계"]
    return head, out


# ------------------------------------------------------------------ 화면용(JSON)
def to_json(res, scale=1.0):
    out = {"stats": res["stats"], "floors": []}
    for f in range(min(2, res["stats"]["floors"])):
        W = world(res, f)
        r = lambda g: [[round(x * scale, 3), round(y * scale, 3)] for x, y in g.exterior.coords]
        ln = lambda g: [[round(x * scale, 3), round(y * scale, 3)] for x, y in g.coords]
        out["floors"].append(dict(
            site=[r(p) for p in _polys(W["site"])],
            usable=[r(p) for p in _polys(W["usable"])],
            cores=[r(p) for c in W["cores"] for p in _polys(c)],
            ramp=[r(p) for p in _polys(W["ramp"])] if W["ramp"] is not None else [],
            drive=[dict(o=r(p), h=[[[round(x * scale, 3), round(y * scale, 3)] for x, y in h.coords] for h in p.interiors])
                   for p in _polys(W["drive"])],
            columns=[r(c) for c in W["columns"]],
            stalls=[dict(k=s["kind"], p=r(s["poly"]), n=s["no"], ev=s["ev"]) for s in W["stalls"]],
            grid=[ln(g) for g in W["grid_x"] + W["grid_y"]],
            flows=[dict(l=ln(g), k=k) for g, k in W["flows"]],
            dead=[[round(p.x, 3), round(p.y, 3)] for p, _ in W["dead"]],
            arrows=[[round(x * scale, 3), round(y * scale, 3), round(dx, 4), round(dy, 4)] for x, y, dx, dy in W["arrows"]],
        ))
    return out


# ------------------------------------------------------------------ PNG
def _font():
    from matplotlib import font_manager
    for fn in ("Pretendard-Regular.ttf", "Pretendard-SemiBold.ttf"):
        p = os.path.join(FONT_DIR, fn)
        if os.path.exists(p):
            font_manager.fontManager.addfont(p)
    names = [f.name for f in font_manager.fontManager.ttflist if "Pretendard" in f.name]
    return names[0] if names else "DejaVu Sans"


BLUE, BLUE_D, BLUE_L = "#2457d6", "#143c9a", "#9db8ef"


def plan_png(res, path, floor=0, title=None, dpi=170):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MPoly
    fam = _font()
    plt.rcParams["font.family"] = fam
    W = world(res, floor)
    st = res["stats"]
    minx, miny, maxx, maxy = W["site"].bounds
    w, h = maxx - minx, maxy - miny
    fig_w = 12
    fig_h = max(5.5, (fig_w - 2.6) * h / max(w, 1) + 2.0)
    fig = plt.figure(figsize=(fig_w, fig_h))
    ax = fig.add_axes([0.01, 0.07, 0.77, 1 - 0.85 / fig_h - 0.07])
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    def add(g, **kw):
        for p in _polys(g):
            ax.add_patch(MPoly(np.asarray(p.exterior.coords), closed=True, **kw))
            for hole in p.interiors:
                ax.add_patch(MPoly(np.asarray(hole.coords), closed=True, fc="white", ec=kw.get("ec", "none"),
                                   lw=kw.get("lw", 0)))

    add(W["site"], fc="none", ec="#111", lw=1.4)
    add(W["usable"], fc="white", ec="#555", lw=0.9)
    add(W["drive"], fc="#f2f4f8", ec="none")
    for c in W["cores"]:
        add(c, fc="#e3e3e3", ec="#444", lw=0.8, hatch="////")
    if W["ramp"] is not None:
        add(W["ramp"], fc="#eef3fd", ec=BLUE, lw=1.0, hatch="\\\\\\")
    # 격자
    for g in W["grid_x"] + W["grid_y"]:
        xs, ys = g.xy
        ax.plot(xs, ys, color="#9a9a9a", lw=0.45, ls=(0, (8, 2, 1, 2)), zorder=1)
    rb = max(0.9, min(1.6, max(w, h) / 60))
    for i, g in enumerate(W["grid_x"]):
        p = g.coords[-1]
        ax.add_patch(plt.Circle(p, rb, fc="white", ec="#555", lw=0.6, zorder=5))
        ax.text(p[0], p[1], f"X{i + 1}", ha="center", va="center", fontsize=5.5, color="#333", zorder=6)
    for i, g in enumerate(W["grid_y"]):
        p = g.coords[0]
        ax.add_patch(plt.Circle(p, rb, fc="white", ec="#555", lw=0.6, zorder=5))
        ax.text(p[0], p[1], f"Y{i + 1}", ha="center", va="center", fontsize=5.5, color="#333", zorder=6)
    # 구획: 테두리 + 대각선(예시 도면 표기)
    fs = max(3.0, min(5.5, 380 / max(w, h)))
    for s in W["stalls"]:
        k = s["kind"]
        if k == "A":
            add(s["poly"], fc=BLUE, ec=BLUE_D, lw=0.6, zorder=3)
        else:
            ec = BLUE if (k == "E" or s["ev"]) else "#111"
            add(s["poly"], fc="#eef3fd" if s["ev"] else "white", ec=ec, lw=0.55,
                ls=(0, (2, 1)) if k == "C" else "-", zorder=3)
            o, v, u = s["o"], s["v"], s["u"]
            a = o
            b = (o[0] + v[0] * s["W"] + u[0] * s["L"], o[1] + v[1] * s["W"] + u[1] * s["L"])
            ax.plot([a[0], b[0]], [a[1], b[1]], color="#c4c4c4", lw=0.3, zorder=3)
        c = s["poly"].centroid
        lab = "EV" if s["ev"] else str(s["no"])
        ax.text(c.x, c.y, lab, ha="center", va="center", fontsize=fs, zorder=4,
                color="white" if k == "A" else (BLUE_D if s["ev"] else "#555"),
                fontweight="semibold" if s["ev"] else "normal")
    for c in W["columns"]:
        add(c, fc="#111", ec="#111", lw=0.3, zorder=4)
    # 동선
    for g, k in W["flows"]:
        xs, ys = g.xy
        ax.plot(xs, ys, color=BLUE if k == "main" else BLUE_L, lw=2.0 if k == "main" else 1.1,
                ls=(0, (5, 2.5)), zorder=2, solid_capstyle="round")
    for x, y, dx, dy in W["arrows"]:
        ax.add_patch(plt.Polygon(_arrow_pts(x, y, dx, dy), closed=True, fc=BLUE, ec="none", zorder=6))
    for p, sg in W["dead"]:
        ax.text(p.x, p.y, "회차", ha="center", va="center", fontsize=6.5, color=BLUE_D, zorder=6,
                bbox=dict(boxstyle="round,pad=0.25", fc="white", ec=BLUE_D, lw=0.6))
    if W["ramp_mouth"] is not None:
        m = W["ramp_mouth"]
        ax.plot([m.x], [m.y], marker="o", ms=4, color=BLUE, zorder=6)
    pad = 2 * rb + 3
    ax.set_xlim(minx - pad, maxx + pad)
    ax.set_ylim(miny - pad, maxy + pad)
    ax.set_aspect("equal")
    ax.axis("off")
    pf = st["per_floor"][floor]
    t = title or f"B{floor + 1} 주차 배치"
    cs = st["circulation"]
    sub = (f"{pf['legal']}대 · 전체 {st['floors']}개 층 법정 {st['total_legal']}대 · 대당 {st['area_per_stall']}㎡ · "
           f"{st['park_angle']}° 주차 · 차로 {st['aisle_w']}m · 기둥 {len(W['columns'])}개")
    fig.text(0.015, 1 - 0.18 / fig_h, t, fontsize=14, fontweight="semibold", va="top", color="#111")
    fig.text(0.015, 1 - 0.5 / fig_h, sub, fontsize=8.5, va="top", color="#444")
    # 오른쪽: 대수표 + 동선 지표
    head, rows = count_table(st)
    y = 1 - 0.95 / fig_h
    fig.text(0.80, y, "주차대수", fontsize=10, fontweight="semibold", color="#111", va="top")
    y -= 0.32 / fig_h
    cols_x = [0.80, 0.86, 0.925, 0.965][:2] + [0.92 + 0.0 * i for i in range(0)]
    nfl = len(head) - 2
    xs = [0.80, 0.85] + [0.915 + i * (0.075 / max(nfl, 1)) for i in range(nfl)]
    for row_ in [head] + rows:
        for x, txt in zip(xs, row_):
            fig.text(x, y, txt, fontsize=7.2, color="#111" if row_ is head or row_[0] == "합계" else "#333",
                     fontweight="semibold" if row_ is head or row_[0] == "합계" else "normal", va="top")
        y -= 0.24 / fig_h
    y -= 0.2 / fig_h
    fig.text(0.80, y, "동선", fontsize=10, fontweight="semibold", color="#111", va="top")
    y -= 0.32 / fig_h
    ph = st["phase"]
    lines = [f"평균 주행 {cs['avg_drive']}m (최대 {cs['max_drive']}m)",
             f"막다른 차로 {cs['dead_ends']}곳 (구획 {cs['dead_end_stalls']}면)",
             f"순환 고리 {cs['loops']} · 교차 T {cs['t_junctions']} / + {cs['crossings']}",
             f"동선 감점 {cs['penalty']}대분 (가중치 {cs['weight']})",
             "", "기둥",
             f"기둥 없이 {ph['no_columns']}면 → 기둥 넣고 {ph['with_columns']}면",
             f"간격 {min(st['spans']) if st['spans'] else '-'}~{max(st['spans']) if st['spans'] else '-'}m"]
    for ln in lines:
        big = ln == "기둥"
        fig.text(0.80, y, ln, fontsize=10 if big else 7.4, fontweight="semibold" if big else "normal",
                 color="#111" if big else "#333", va="top")
        y -= (0.32 if big else 0.22) / fig_h
    leg = [("일반", dict(fc="white", ec="#111")), ("확장", dict(fc="white", ec=BLUE)), ("전기차", dict(fc="#eef3fd", ec=BLUE)),
           ("경형", dict(fc="white", ec="#5a7fd0")), ("장애인", dict(fc=BLUE, ec=BLUE_D)), ("기둥", dict(fc="#111", ec="#111"))]
    x = 0.015
    for name, sty in leg:
        fig.patches.append(MPoly([[x, 0.02], [x + 0.016, 0.02], [x + 0.016, 0.04], [x, 0.04]], closed=True,
                                 transform=fig.transFigure, lw=0.7, **sty))
        fig.text(x + 0.021, 0.03, name, fontsize=8, va="center", color="#333")
        x += 0.075
    for name, col, lw in (("주 동선", BLUE, 2.0), ("부 동선", BLUE_L, 1.1)):
        fig.lines.append(matplotlib.lines.Line2D([x, x + 0.03], [0.03, 0.03], transform=fig.transFigure, color=col,
                                                 lw=lw, ls=(0, (5, 2.5))))
        fig.text(x + 0.035, 0.03, name, fontsize=8, va="center", color="#333")
        x += 0.09
    fig.savefig(path, dpi=dpi)
    plt.close(fig)


# ------------------------------------------------------------------ DXF
def _blocks(doc, rules, S):
    """구획 블록: 앞-왼쪽 모서리가 원점, x = 너비, y = 들어가는 방향(앞 0 → 뒤 L). 예시처럼 대각선 + 차막이 2개."""
    def stops(b, W, L, inset=0.2, sw=0.75, back=1.2, t=0.1):
        for x0 in (inset, W - inset - sw):
            b.add_lwpolyline([((x0) * S, (L - back) * S), ((x0 + sw) * S, (L - back) * S),
                              ((x0 + sw) * S, (L - back - t) * S), (x0 * S, (L - back - t) * S)], close=True)

    def rect(b, W, L):
        b.add_lwpolyline([(0, 0), (W * S, 0), (W * S, L * S), (0, L * S)], close=True)

    dims = {"N": (rules.stall_w, rules.stall_l), "E": (rules.ext_w, rules.ext_l), "C": (rules.compact_w, rules.compact_l),
            "A": (rules.acc_w, rules.acc_l)}
    made = {}
    for k in ("N", "E", "C"):
        W, L = dims[k]
        b = doc.blocks.new(BLOCK_NAME[k] + f"_{W:.1f}x{L:.1f}".replace(".", "_"))
        rect(b, W, L)
        b.add_line((0, 0), (W * S, L * S))
        stops(b, W, L, inset=0.15 if k == "C" else 0.2, sw=0.6 if k == "C" else 0.75)
        made[k] = b.name
    for base in ("N", "E"):
        W, L = dims[base]
        b = doc.blocks.new(BLOCK_NAME["V"] + f"_{W:.1f}x{L:.1f}".replace(".", "_"))
        rect(b, W, L)
        b.add_line((0, 0), (W * S, L * S))
        stops(b, W, L)
        # 충전 표시: 번개 + EV
        cx, cy = W / 2, L * 0.42
        bolt = [(cx + 0.10, cy + 0.55), (cx - 0.22, cy - 0.02), (cx + 0.02, cy - 0.02), (cx - 0.10, cy - 0.55),
                (cx + 0.22, cy + 0.06), (cx - 0.02, cy + 0.06)]
        b.add_lwpolyline([(x * S, y * S) for x, y in bolt], close=True)
        b.add_text("EV", height=0.32 * S).set_placement((cx * S, (cy - 0.95) * S),
                                                         align=__import__("ezdxf").enums.TextEntityAlignment.MIDDLE_CENTER)
        made["V" + base] = b.name
    W, L = dims["A"]
    b = doc.blocks.new(BLOCK_NAME["A"] + f"_{W:.1f}x{L:.1f}".replace(".", "_"))
    rect(b, W, L)
    strip = 1.0     # 휠체어 승하차 빗금 폭
    b.add_line(((W - strip) * S, 0), ((W - strip) * S, L * S))
    t = 0.15
    while t < L + strip:
        x0, y0 = W - strip, t
        x1, y1 = W, t - strip
        # 빗금을 칸 안으로 자름
        if y1 < 0:
            x1 = W - strip + (t if t < strip else strip)
            x1 = min(W, W - strip + t)
            y1 = max(0.0, y1)
        if y0 > L:
            x0 = W - strip + (y0 - L)
            y0 = L
        if x1 > x0:
            b.add_line((x0 * S, y0 * S), (x1 * S, y1 * S))
        t += 0.3
    Wc = W - strip
    for x0 in (0.2, Wc - 0.95):
        b.add_lwpolyline([(x0 * S, (L - 1.2) * S), ((x0 + 0.75) * S, (L - 1.2) * S), ((x0 + 0.75) * S, (L - 1.3) * S),
                          (x0 * S, (L - 1.3) * S)], close=True)
    # 휠체어 기호(간략)
    cx, cy = Wc / 2, L * 0.45
    b.add_circle((cx * S, (cy - 0.2) * S), 0.45 * S)
    b.add_circle(((cx - 0.05) * S, (cy + 0.75) * S), 0.12 * S)
    b.add_lwpolyline([((cx - 0.05) * S, (cy + 0.6) * S), ((cx - 0.05) * S, (cy + 0.05) * S), ((cx + 0.45) * S, (cy + 0.05) * S),
                      ((cx + 0.6) * S, (cy - 0.55) * S)])
    made["A"] = b.name
    return made


def _fillet_flows(flows, r):
    """주/부 동선 선분 → (직선 토막들, 둥근 모서리 호들). 호 = (시작, 끝, bulge, 종류)."""
    segs = []
    for g, k in flows:
        (x1, y1), (x2, y2) = g.coords[0], g.coords[-1]
        if math.hypot(x2 - x1, y2 - y1) > 1e-6:
            segs.append([(x1, y1), (x2, y2), k])
    key = lambda p: (round(p[0], 2), round(p[1], 2))
    inc = {}
    for i, (a, b, k) in enumerate(segs):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        inc.setdefault(key(a), []).append((i, 0, ((b[0] - a[0]) / L, (b[1] - a[1]) / L), L, a))
        inc.setdefault(key(b), []).append((i, 1, ((a[0] - b[0]) / L, (a[1] - b[1]) / L), L, b))
    trim = [[0.0, 0.0] for _ in segs]
    arcs = []
    for v, lst in inc.items():
        if len(lst) < 2:
            continue
        for i, end, d, L, p in lst:
            straight = any(j != i and d[0] * d2[0] + d[1] * d2[1] < -0.99 for j, _, d2, _, _ in lst)
            if not straight:
                trim[i][end] = min(r, 0.45 * L)
        for a_ in range(len(lst)):
            for b_ in range(a_ + 1, len(lst)):
                i1, e1, d1, L1, p = lst[a_]
                i2, e2, d2, L2, _ = lst[b_]
                if abs(d1[0] * d2[0] + d1[1] * d2[1]) > 0.1:
                    continue
                rr = min(r, 0.45 * L1, 0.45 * L2)
                if rr < 0.3:
                    continue
                P1 = (p[0] + d1[0] * rr, p[1] + d1[1] * rr)
                P2 = (p[0] + d2[0] * rr, p[1] + d2[1] * rr)
                C = (p[0] + (d1[0] + d2[0]) * rr, p[1] + (d1[1] + d2[1]) * rr)
                cr = (P1[0] - C[0]) * (P2[1] - C[1]) - (P1[1] - C[1]) * (P2[0] - C[0])
                bulge = math.tan(math.pi / 8) * (1 if cr > 0 else -1)
                kind = "main" if segs[i1][2] == "main" and segs[i2][2] == "main" else "sub"
                arcs.append((P1, P2, bulge, kind))
    lines = []
    for (a, b, k), (t0, t1) in zip(segs, trim):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
        if L - t0 - t1 < 0.05:
            continue
        lines.append(((a[0] + ux * t0, a[1] + uy * t0), (b[0] - ux * t1, b[1] - uy * t1), k))
    return lines, arcs


def write_dxf(res, path, unit_scale=1.0, insunits=6):
    import ezdxf
    from ezdxf.enums import TextEntityAlignment as TA
    S = unit_scale
    doc = ezdxf.new("R2010", setup=True)
    doc.header["$INSUNITS"] = insunits
    doc.header["$LTSCALE"] = 0.5 * S
    for name, (aci, lt) in DXF_LAYERS.items():
        doc.layers.add(name, color=aci, linetype=lt if lt in doc.linetypes else "Continuous")
    doc.layers.get("A-PARK-NO").off()
    # 한글 글꼴: 윈도우 기본 '맑은 고딕'(없으면 캐드가 대체 글꼴로 보여 줌)
    doc.styles.get("Standard").dxf.font = "malgun.ttf"
    ds = doc.dimstyles.new("PK-DIM")
    ds.dxf.dimtxt = 2.5
    ds.dxf.dimasz = 0.8
    ds.dxf.dimscale = 0.25 * S
    ds.dxf.dimexo = 1.5
    ds.dxf.dimexe = 0.0
    ds.dxf.dimdec = 0
    ds.dxf.dimtad = 1
    ds.dxf.dimclrd = 9
    ds.dxf.dimclre = 9
    ds.dxf.dimclrt = 3
    ds.dxf.dimgap = 0.6
    ds.dxf.dimdle = 1.0
    ds.dxf.dimlfac = 1.0
    ds.dxf.dimtxsty = "Standard"
    try:
        ds.set_arrows(blk=ezdxf.ARROWS.dot_small)
    except Exception:
        pass
    msp = doc.modelspace()
    rules = res["rules"]
    st = res["stats"]
    floors = st["floors"]
    W0 = world(res, 0)
    minx, miny, maxx, maxy = W0["site"].bounds
    big = max(maxx - minx, maxy - miny)
    gap = (maxx - minx) + max(30.0, 0.35 * big)
    th_tab = min(2.25, max(0.6, big / 150))
    blocks = _blocks(doc, rules, S)
    r_fl = rules.flow_radius

    def P(x, y, dx):
        return ((x + dx) * S, y * S)

    def pl(g, layer, dx, width=None):
        for p in _polys(g):
            for ring in [p.exterior] + list(p.interiors):
                e = msp.add_lwpolyline([P(x, y, dx) for x, y in ring.coords[:-1]], close=True, dxfattribs={"layer": layer})
                if width:
                    e.dxf.const_width = width * S

    shown = [0] + ([1] if floors > 1 else [])
    for fi in shown:
        W = world(res, fi)
        dx = fi * gap
        pl(W["site"], "A-SITE", dx, width=0.3)
        pl(W["usable"], "A-WAL", dx)
        for c in W["cores"]:
            pl(c, "A-WAL", dx)
            for p in _polys(c):
                h = msp.add_hatch(color=8, dxfattribs={"layer": "A-WAL"})
                h.set_pattern_fill("ANSI31", scale=0.05 * S)
                h.paths.add_polyline_path([P(x, y, dx) for x, y in p.exterior.coords[:-1]], is_closed=True)
        if W["ramp"] is not None:
            pl(W["ramp"], "A-RAMP", dx)
            m = W["ramp_mouth"]
            if m is not None:
                msp.add_text("RAMP", height=th_tab * 0.6 * S, dxfattribs={"layer": "A-TXT1"}).set_placement(
                    P(W["ramp"].centroid.x, W["ramp"].centroid.y, dx), align=TA.MIDDLE_CENTER)
        # 기둥 (예시: A-CON 닫힌 폴리라인)
        for c in W["columns"]:
            pl(c, "A-CON", dx)
        # 구획 블록 (A-PARK-B)
        rot = lambda v: math.degrees(math.atan2(v[1], v[0]))
        for s in W["stalls"]:
            k = s["kind"]
            name = blocks["V" + k] if (s["ev"] and k in ("N", "E")) else blocks[k]
            msp.add_blockref(name, P(s["o"][0], s["o"][1], dx), dxfattribs={"layer": "A-PARK-B", "rotation": rot(s["v"])})
            c = s["poly"].centroid
            msp.add_text(str(s["no"]), height=0.35 * S, dxfattribs={"layer": "A-PARK-NO"}).set_placement(
                P(c.x, c.y, dx), align=TA.MIDDLE_CENTER)
        # 격자선 + 기호 + 치수
        rb = max(0.8, min(1.5, big / 70))
        th = W["theta"]
        for i, g in enumerate(W["grid_x"]):
            a, b = g.coords[0], g.coords[-1]
            msp.add_line(P(*a, dx), P(*b, dx), dxfattribs={"layer": "A-CEN"})
            ln_ = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
            ux, uy = (b[0] - a[0]) / ln_, (b[1] - a[1]) / ln_
            cc = (b[0] + ux * rb, b[1] + uy * rb)
            msp.add_circle(P(*cc, dx), rb * S, dxfattribs={"layer": "A-SYM"})
            msp.add_text(f"X{i + 1}", height=rb * 0.8 * S, dxfattribs={"layer": "A-SYM"}).set_placement(
                P(*cc, dx), align=TA.MIDDLE_CENTER)
        for i, g in enumerate(W["grid_y"]):
            a, b = g.coords[0], g.coords[-1]
            msp.add_line(P(*a, dx), P(*b, dx), dxfattribs={"layer": "A-CEN"})
            ln_ = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
            ux, uy = (a[0] - b[0]) / ln_, (a[1] - b[1]) / ln_
            cc = (a[0] + ux * rb, a[1] + uy * rb)
            msp.add_circle(P(*cc, dx), rb * S, dxfattribs={"layer": "A-SYM"})
            msp.add_text(f"Y{i + 1}", height=rb * 0.8 * S, dxfattribs={"layer": "A-SYM"}).set_placement(
                P(*cc, dx), align=TA.MIDDLE_CENTER)

        def dim_chain(lines, axis_end, off):
            pts = []
            for g in lines:
                pts.append(g.coords[-1] if axis_end else g.coords[0])
            for a, b in zip(pts, pts[1:]):
                try:
                    d = msp.add_aligned_dim(p1=P(*a, dx), p2=P(*b, dx), distance=off * S, dimstyle="PK-DIM",
                                            dxfattribs={"layer": "DIM"})
                    d.render()
                except Exception:
                    pass
        for gx_, gy_ in W["grid_groups"]:
            dim_chain(gx_, True, -1.5)
            dim_chain(gy_, False, 1.5)
        # 차량 동선 (예시: 폭 500 폴리라인, 모서리 둥글게)
        lines, arcs = _fillet_flows(W["flows"], r_fl)
        for a, b, k in lines:
            msp.add_lwpolyline([P(*a, dx), P(*b, dx)], dxfattribs={
                "layer": "SOL 차량동선" if k == "main" else "SOL 차량동선(부)", "const_width": 0.5 * S})
        for a, b, bu, k in arcs:
            msp.add_lwpolyline([(*P(*a, dx), 0, 0, bu), (*P(*b, dx), 0, 0, 0)], format="xyseb", dxfattribs={
                "layer": "SOL 차량동선" if k == "main" else "SOL 차량동선(부)", "const_width": 0.5 * S})
        for x, y, dx_, dy_ in W["arrows"]:
            pts_ = [P(px, py, dx) for px, py in _arrow_pts(x, y, dx_, dy_)]
            msp.add_lwpolyline(pts_, close=True, dxfattribs={"layer": "SOL 진행방향"})
            hh = msp.add_hatch(color=5, dxfattribs={"layer": "SOL 진행방향"})
            hh.paths.add_polyline_path(pts_, is_closed=True)
        for p, sg in W["dead"]:
            msp.add_text("회차", height=1.2 * S, dxfattribs={"layer": "A-TXT1"}).set_placement(
                P(p.x, p.y, dx), align=TA.MIDDLE_CENTER)
        # 층 제목
        title = f"지하{fi + 1}층 주차계획도" + ("" if fi == 0 or floors <= 2 else f" (B2~B{floors} 같음)")
        msp.add_text(title, height=th_tab * 1.3 * S, dxfattribs={"layer": "A-TXT"}).set_placement(
            P(minx, miny - 4 - th_tab * 1.5, dx))
        pf = st["per_floor"][fi]
        msp.add_text(f"{pf['legal']}대 (법정)", height=th_tab * 0.8 * S, dxfattribs={"layer": "A-TXT"}).set_placement(
            P(minx, miny - 4 - th_tab * 3.2, dx))

    # 주차대수표 (A-TXT3) — 마지막 평면 오른쪽
    head, rows = count_table(st)
    tx = maxx + (len(shown) - 1) * gap + max(8.0, 0.08 * big)
    ty = maxy
    cw = [th_tab * 4.5, th_tab * 6.5] + [th_tab * 4.0] * (len(head) - 3) + [th_tab * 5.5]
    rh = th_tab * 2.0
    allrows = [head] + rows
    xs = [tx]
    for wdt in cw:
        xs.append(xs[-1] + wdt)
    for ri in range(len(allrows) + 1):
        y = ty - ri * rh
        msp.add_line(P(xs[0], y, 0), P(xs[-1], y, 0), dxfattribs={"layer": "A-TXT3"})
    for x in xs:
        msp.add_line(P(x, ty, 0), P(x, ty - len(allrows) * rh, 0), dxfattribs={"layer": "A-TXT3"})
    for ri, row_ in enumerate(allrows):
        for ci, txt in enumerate(row_):
            if not txt:
                continue
            msp.add_text(txt, height=th_tab * 0.75 * S, dxfattribs={"layer": "A-TXT3"}).set_placement(
                P((xs[ci] + xs[ci + 1]) / 2, ty - (ri + 0.5) * rh, 0), align=TA.MIDDLE_CENTER)
    # 요약 (법규·동선·기둥)
    cs = st["circulation"]
    ph = st["phase"]
    lines = [f"전체 {floors}개 층 법정 {st['total_legal']}대 (실제 구획 {st['total_physical']}면) · 대당 {st['area_per_stall']}㎡",
             f"{st['park_angle']}° 주차 · 차로 {st['aisle_w']}m · 기둥 {rules.column_w * 1000:.0f}×{rules.column_d * 1000:.0f} "
             f"{st['columns']}개 · 기둥 간격 {', '.join(f'{x:g}' for x in sorted(set(st['spans'])))}m",
             f"배치 순서: 코어·경사로 고정 → 기둥 없이 주차 {ph['no_columns']}면 → 기둥 배열 후 {ph['with_columns']}면",
             f"동선: 평균 주행 {cs['avg_drive']}m · 막다른 차로 {cs['dead_ends']}곳 · 순환 {cs['loops']} · 교차 T{cs['t_junctions']}/+{cs['crossings']}"]
    for ch in st["checks"]:
        lines.append(f"{'○' if ch['ok'] else '×'} {ch['item']}: {ch['need']} / {ch['have']}")
    y0 = ty - (len(allrows) + 1.5) * rh
    for i, t in enumerate(lines):
        msp.add_text(t, height=th_tab * 0.55 * S, dxfattribs={"layer": "A-TXT"}).set_placement(
            P(tx, y0 - i * th_tab * 1.1, 0))
    doc.saveas(path, encoding="utf-8")


# ------------------------------------------------------------------ 3dm
def _mesh_polygon(rh, poly, z, zfun=None):
    """구멍 있는 다각형 → 삼각 메시 (zfun(x,y) 가 있으면 높이를 그 값으로)."""
    m = rh.Mesh()
    try:
        tris = shapely.constrained_delaunay_triangles(poly)
    except Exception:
        tris = shapely.delaunay_triangles(poly)
    idx = {}
    for t in shapely.get_parts(tris):
        cs = list(t.exterior.coords)[:3]
        ids = []
        for x, y in cs:
            key = (round(x, 5), round(y, 5))
            if key not in idx:
                zz = zfun(x, y) if zfun else z
                idx[key] = m.Vertices.Add(x, y, zz)
            ids.append(idx[key])
        m.Faces.AddFace(ids[0], ids[1], ids[2])
    m.Normals.ComputeNormals()
    m.Compact()
    return m


def _mesh_prism(rh, poly, z0, z1):
    m = rh.Mesh()
    bottom = _mesh_polygon(rh, poly, z0)
    top = _mesh_polygon(rh, poly, z1)
    m.Append(bottom)
    m.Append(top)
    for ring in [poly.exterior] + list(poly.interiors):
        cs = list(ring.coords)
        for (x1, y1), (x2, y2) in zip(cs, cs[1:]):
            a = m.Vertices.Add(x1, y1, z0); b = m.Vertices.Add(x2, y2, z0)
            c = m.Vertices.Add(x2, y2, z1); d = m.Vertices.Add(x1, y1, z1)
            m.Faces.AddFace(a, b, c, d)
    m.Normals.ComputeNormals()
    m.Compact()
    return m


def _box(rh, cx, cy, ang, L, W, z0, z1):
    m = rh.Mesh()
    ca, sa = math.cos(ang), math.sin(ang)
    pts = []
    for z in (z0, z1):
        for u, v in ((-L / 2, -W / 2), (L / 2, -W / 2), (L / 2, W / 2), (-L / 2, W / 2)):
            pts.append(m.Vertices.Add(cx + u * ca - v * sa, cy + u * sa + v * ca, z))
    f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    for a, b, c, d in f:
        m.Faces.AddFace(pts[a], pts[b], pts[c], pts[d])
    m.Normals.ComputeNormals()
    return m


def write_3dm(res, path, unit_scale=1.0, unit="m", cars=True):
    import rhino3dm as rh
    st = res["stats"]
    rules = res["rules"]
    fh = rules.floor_h
    model = rh.File3dm()
    model.Settings.ModelUnitSystem = {"mm": rh.UnitSystem.Millimeters, "cm": rh.UnitSystem.Centimeters}.get(
        unit, rh.UnitSystem.Meters)
    S = unit_scale
    xf = rh.Transform.Scale(rh.Point3d(0, 0, 0), S)

    def layer(name, parent=None):
        L = rh.Layer()
        L.Name = name
        rgb = LAYERS.get(name, (7, (90, 90, 90)))[1] if parent is None else LAYERS.get(name.split("::")[-1], (7, (90, 90, 90)))[1]
        L.Color = (*rgb, 255)
        if parent is not None:
            L.ParentLayerId = model.Layers.FindIndex(parent).Id
        return model.Layers.Add(L)

    def add_mesh(m, li, name=None):
        m.Transform(xf)
        a = rh.ObjectAttributes()
        a.LayerIndex = li
        if name:
            a.Name = name
        model.Objects.AddMesh(m, a)

    def add_curve(poly, z, li, name=None):
        for p in _polys(poly):
            pts = [rh.Point3d(x * S, y * S, z * S) for x, y in p.exterior.coords]
            a = rh.ObjectAttributes()
            a.LayerIndex = li
            if name:
                a.Name = name
            model.Objects.AddPolyline(rh.Point3dList(pts) if hasattr(rh, "Point3dList") else pts, a)

    floors = st["floors"]
    for fi in range(floors):
        fname = f"B{fi + 1}"
        top = layer(fname)
        sub = {n: layer(n, top) for n in ["P-슬래브", "P-코어", "P-경사로", "P-차로", "P-기둥", "P-주차_일반", "P-주차_확장",
                                           "P-주차_경형", "P-주차_장애인", "P-주차_전기차", "P-차량(예시)"]}
        W = world(res, 0 if fi == 0 else 1)
        z = -(fi + 1) * fh
        for p in _polys(W["usable"]):
            add_mesh(_mesh_polygon(rh, p, z - 0.25), sub["P-슬래브"], f"{fname} 바닥")
        for p in _polys(W["drive"]):
            add_mesh(_mesh_polygon(rh, p, z + 0.005), sub["P-차로"], f"{fname} 차로")
        for c in W["cores"]:
            for p in _polys(c):
                add_mesh(_mesh_prism(rh, p, z, z + fh), sub["P-코어"], "코어")
        for c in W["columns"]:
            for p in _polys(c):
                add_mesh(_mesh_prism(rh, p, z, z + fh - 0.25), sub["P-기둥"], "기둥")
        if W["ramp"] is not None:
            mouth = W["ramp_mouth"]
            ramp = W["ramp"]
            if mouth is not None:
                far = max((shapely.Point(c) for p in _polys(ramp) for c in p.exterior.coords),
                          key=lambda q: q.distance(mouth))
                dx, dy = far.x - mouth.x, far.y - mouth.y
                Ld = math.hypot(dx, dy) or 1
                ux, uy = dx / Ld, dy / Ld
                ext = max(((c[0] - mouth.x) * ux + (c[1] - mouth.y) * uy) for p in _polys(ramp) for c in p.exterior.coords)
                zf = lambda x, y: z + fh * min(1, max(0, ((x - mouth.x) * ux + (y - mouth.y) * uy) / max(ext, 1)))
            else:
                zf = None
            for p in _polys(ramp):
                add_mesh(_mesh_polygon(rh, p, z, zf), sub["P-경사로"], "경사로 (위층으로)")
        for s in W["stalls"]:
            kk = "V" if s["ev"] else s["kind"]
            add_curve(s["poly"], z + 0.01, sub[KIND_LAYER[kk]], f"{fname}-{s['no']} {KIND_NAME[kk]}")
        if cars and fi == 0:
            for s in W["stalls"]:
                p = s["poly"]
                rr = p.minimum_rotated_rectangle
                cs = list(rr.exterior.coords)
                e1 = (cs[1][0] - cs[0][0], cs[1][1] - cs[0][1])
                e2 = (cs[2][0] - cs[1][0], cs[2][1] - cs[1][1])
                lng = e1 if math.hypot(*e1) >= math.hypot(*e2) else e2
                ang = math.atan2(lng[1], lng[0])
                c = p.centroid
                Lc = 3.4 if s["kind"] == "C" else 4.6
                Wc = 1.6 if s["kind"] == "C" else 1.85
                add_mesh(_box(rh, c.x, c.y, ang, Lc, Wc, z + 0.18, z + 0.85), sub["P-차량(예시)"], "차량")
                add_mesh(_box(rh, c.x, c.y, ang, Lc * 0.5, Wc * 0.88, z + 0.85, z + 1.42), sub["P-차량(예시)"], "차량")
    model.Write(path, 7)


# ------------------------------------------------------------------ 통계
def write_stats(res, folder, base):
    st = res["stats"]
    with open(os.path.join(folder, base + "_통계.json"), "w", encoding="utf-8") as f:
        json.dump(dict(stats=st, rules=res["rules"].to_dict()), f, ensure_ascii=False, indent=2)
    with open(os.path.join(folder, base + "_층별대수.csv"), "w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["층", "일반(2.5×5.0)", "확장(2.6×5.2)", "전기차", "경형(2.0×3.6)", "장애인(3.3×5.0)", "구획 수", "법정 인정 대수"])
        for p in st["per_floor"]:
            w.writerow([p["floor"], p["일반"], p["확장"], p["전기차"], p["경형"], p["장애인"], p["physical"], p["legal"]])
        w.writerow(["합계"] + [sum(p[k] for p in st["per_floor"]) for k in ("일반", "확장", "전기차", "경형", "장애인")]
                   + [st["total_physical"], st["total_legal"]])
    txt = summary_text(res)
    with open(os.path.join(folder, base + "_요약.txt"), "w", encoding="utf-8") as f:
        f.write(txt)
    return txt


def summary_text(res):
    st = res["stats"]
    L = [f"지하주차장 배치 요약",
         f"",
         f"전체 {st['floors']}개 층 법정 {st['total_legal']}대 (실제 구획 {st['total_physical']}면)",
         f"층당: " + ", ".join(f"{p['floor']} {p['legal']}대" for p in st["per_floor"]),
         f"주차 영역 {st['floor_area']}㎡ (코어 제외 {st['park_area']}㎡) · 대당 {st['area_per_stall']}㎡",
         f"{st['park_angle']}° 주차 · 차로 {st['aisle_w']}m · 격자 방향 {st['grid_angle']}° · 연결 차로 {st['links']}곳",
         "",
         "배치 순서",
         f"  1) 코어·경사로 고정  2) 기둥 없이 주차·동선 → {st['phase']['no_columns']}면  "
         f"3) 기둥 {st['columns']}개 배열 → {st['phase']['with_columns']}면 (기둥 때문에 {st['phase']['column_loss']}면 줄어듦)",
         f"  기둥 간격: {', '.join(f'{x:g}' for x in st['spans']) or '-'} m",
         "",
         "동선 (B1)",
         f"  경사로에서 평균 주행 {st['circulation']['avg_drive']}m · 가장 먼 구획 {st['circulation']['max_drive']}m",
         f"  막다른 차로 {st['circulation']['dead_ends']}곳 (길이 합 {st['circulation']['dead_end_len']}m · 붙은 구획 {st['circulation']['dead_end_stalls']}면)",
         f"  순환 고리 {st['circulation']['loops']} · T자 교차 {st['circulation']['t_junctions']} · 십자 교차 {st['circulation']['crossings']}",
         f"  동선 감점 {st['circulation']['penalty']}대분 (가중치 {st['circulation']['weight']}) · 순위 점수 {st['score']}",
         "", "피드백",
         *[f"  [{ {'bad': '손실 큼', 'warn': '주의', 'good': '좋음', 'info': '참고'}.get(f_['level'], '') }] {f_['title']} — {f_['detail']}"
           for f_ in st.get("feedback", [])],
         ""]
    if st.get("need_floors"):
        L.append(f"법정 대수 목표를 채우려면 지하 {st['need_floors']}개 층이 필요합니다.")
        L.append("")
    L.append("법규 확인")
    for ch in st["checks"]:
        L.append(f"  {'○' if ch['ok'] else '×'} {ch['item']}: 기준 {ch['need']} / 배치 {ch['have']}")
    L.append("")
    L.append("주의: 차로 너비 표(60°·45°)와 장애인 비율(지자체 조례)은 사업지 기준으로 다시 확인하세요.")
    return "\n".join(L) + "\n"
