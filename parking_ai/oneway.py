"""일방통행·경사로 검사 (v1.6).

1) 비스듬한 주차(60°·45°)를 일방 차로 너비로 쓸 때: 차로마다 구획이 기운 쪽으로만 달릴 수 있음.
   연결 차로·경사로 앞마당은 양방향(너비 6m)으로 보고, 경사로에서 들어가 구획에 대고 다시 경사로로 나갈 수
   있는지 구획마다 따짐. 못 가는 구획은 빼고, 차로마다 진행 방향 화살표를 둠.
2) 그린 경사로: 직선이면 폭, 곡선이면 내변 반경·폭(주차장법 시행규칙 제6조).
   - 직선 1차로 3.3m · 2차로 6.0m / 곡선 1차로 3.6m · 2차로 6.5m
   - 곡선 내변 반경 6m 이상(그 경사로를 쓰는 주차장 총대수 50대 이하면 5m)
   - 50대 이상이면 2차로(6m) 또는 들어가는·나가는 경사로를 따로
"""
import math
from collections import defaultdict, deque

import numpy as np


def lean_of(ai):
    return 1 if ai % 2 == 0 else -1


def check_flow(lay, S, stalls, rows, tol=0.3):
    """반환: (keep 마스크 list[bool], arrows [(x, y, dx, dy)], info dict)."""
    Y = lay.Y
    S = list(S) + [k for k in getattr(lay, "forced", []) if k not in S]
    # 차로 조각마다 (위치, hub) 접점
    contacts = defaultdict(list)
    hubs = 0
    hub_of_ramp = {}
    for k in S:
        for pc in lay.conn[k]:
            h = hubs
            hubs += 1
            xm = (pc["xl"] + pc["xr"]) / 2
            for nid in pc["contacts"]:
                contacts[nid].append((xm, h, (pc["xr"] - pc["xl"]) / 2))
            for rn in pc["ramp"]:
                hub_of_ramp.setdefault(rn, []).append(h)
    ramp_hubs = set(h for hs in hub_of_ramp.values() for h in hs)
    for rn, nid in lay.base_links:
        h = hubs
        hubs += 1
        ab = lay.aprons[rn - lay.ramp_node].bounds if rn - lay.ramp_node < len(lay.aprons) else None
        _, a0, a1 = lay.nodes[nid]
        if ab is None:
            continue
        lo, hi = max(a0, ab[0]), min(a1, ab[2])
        contacts[nid].append(((lo + hi) / 2, h, max(0.0, (hi - lo) / 2)))
        ramp_hubs.add(h)
    # 같은 경사로 앞마당끼리 묶기(양방향)
    adj = defaultdict(set)
    radj = defaultdict(set)
    rl = sorted(ramp_hubs)
    for a, b in zip(rl, rl[1:]):
        adj[a].add(b); adj[b].add(a); radj[a].add(b); radj[b].add(a)
    for nid, cs in contacts.items():
        ai = lay.nodes[nid][0]
        d = lean_of(ai)
        cs.sort(key=lambda c: d * c[0])
        for (x1, h1, _), (x2, h2, _) in zip(cs, cs[1:]):
            adj[h1].add(h2)
            radj[h2].add(h1)

    def bfs(start, g):
        seen = set(start)
        q = deque(start)
        while q:
            u = q.popleft()
            for v in g[u]:
                if v not in seen:
                    seen.add(v)
                    q.append(v)
        return seen
    fwd = bfs(list(ramp_hubs), adj)       # 경사로에서 갈 수 있는 접점
    bwd = bfs(list(ramp_hubs), radj)      # 경사로로 돌아갈 수 있는 접점

    keep = []
    bad_rows = set()
    for s in stalls:
        if s["row"] >= len(rows):
            keep.append(True)
            continue
        ai = rows[s["row"]]["aisle"]
        xf = s["poly"].centroid.x
        nid = None
        for n_, (a_i, a0, a1) in enumerate(lay.nodes):
            if a_i == ai and a0 - 0.5 <= xf <= a1 + 0.5:
                nid = n_
                break
        if nid is None or nid not in contacts:
            keep.append(False)
            continue
        d = lean_of(ai)
        cs = contacts[nid]
        # 들어올 때: 위쪽(흐름 거꾸로 쪽) 접점 중 경사로에서 닿는 것
        inn = any(h in fwd and d * (xf - x) >= -(hw + tol + s["W"]) for x, h, hw in cs)
        out = any(h in bwd and d * (x - xf) >= -(hw + tol + s["W"]) for x, h, hw in cs)
        ok = inn and out
        keep.append(ok)
        if not ok:
            bad_rows.add(nid)

    # 화살표: 차로 조각마다 15m 간격, 구획이 하나라도 있는 조각만
    arrows = []
    used = set()
    for s, k in zip(stalls, keep):
        if k and s["row"] < len(rows):
            ai = rows[s["row"]]["aisle"]
            used.add(ai)
    for nid, (ai, a0, a1) in enumerate(lay.nodes):
        if ai not in used or a1 - a0 < 6:
            continue
        ya, yb = Y.aisles[ai]
        ym = (ya + yb) / 2
        d = lean_of(ai)
        n = max(1, int((a1 - a0) // 15))
        for i in range(n):
            xm = a0 + (a1 - a0) * (i + 0.5) / n
            arrows.append((xm, ym, float(d), 0.0))
    info = dict(dropped=keep.count(False), bad_pieces=len(bad_rows))
    return keep, arrows, info


# ------------------------------------------------------------------ 경사로 모양
def _fit_circle(pts):
    P = np.asarray(pts, float)
    A = np.c_[2 * P[:, 0], 2 * P[:, 1], np.ones(len(P))]
    b = (P ** 2).sum(1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, c = sol
    r = math.sqrt(max(c + cx * cx + cy * cy, 0))
    return cx, cy, r


def _inscribed_width(poly):
    """가장 좁은 곳 너비 어림: 안으로 줄였을 때 사라지기 직전 두께 × 2 (꺾인 경사로도 됨)."""
    lo, hi = 0.0, 20.0
    for _ in range(20):
        m = (lo + hi) / 2
        g = poly.buffer(-m, join_style=2)
        # 사라지거나, 다리가 끊겨 여러 조각이 되면 그 너비가 좁은 곳
        if g.is_empty or getattr(g, "geoms", None) is not None and len(g.geoms) > 1:
            hi = m
        else:
            lo = m
    return 2 * lo


def ramp_shape(poly):
    """dict(kind='straight'|'curve', width, inner_r, outer_r)."""
    rr = poly.minimum_rotated_rectangle
    cs = list(rr.exterior.coords)
    e1 = math.hypot(cs[1][0] - cs[0][0], cs[1][1] - cs[0][1])
    e2 = math.hypot(cs[2][0] - cs[1][0], cs[2][1] - cs[1][1])
    fill = poly.area / max(rr.area, 1e-9)
    ext = list(poly.exterior.coords)[:-1]
    if fill > 0.9 or len(ext) < 6:
        return dict(kind="straight", width=min(e1, e2), length=max(e1, e2))
    # 곡선: 볼록 껍질에서 떨어진 꼭짓점 = 안쪽 호. 그 점들에 원을 맞춰 중심·내변 반경을 잡음
    pts = np.asarray(ext)
    from shapely.geometry import Point
    hb = poly.convex_hull.exterior
    inner_pts = np.array([p for p in pts if hb.distance(Point(p)) > 0.05])
    if len(inner_pts) < 3:
        return dict(kind="straight", width=min(min(e1, e2), _inscribed_width(poly)), length=max(e1, e2))
    cx, cy, _ = _fit_circle(inner_pts)
    ds = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
    if poly.contains(Point(cx, cy)):
        return dict(kind="straight", width=min(e1, e2), length=max(e1, e2))
    # 모서리 거리보다 경계까지 최단 거리가 내변 반경
    inner = poly.exterior.distance(Point(cx, cy))
    outer = float(ds.max())
    return dict(kind="curve", width=outer - inner, inner_r=inner, outer_r=outer)


def ramp_checks(ramps_world, total_legal, rules):
    """그린 경사로 법규 확인 목록."""
    out = []
    if not ramps_world:
        return out
    n = len(ramps_world)
    two_needed = total_legal > 50
    for i, rp in enumerate(ramps_world):
        sh = ramp_shape(rp)
        nm = f"경사로 {i + 1}" if n > 1 else "경사로"
        if sh["kind"] == "straight":
            one, two = rules.ramp_lane, rules.ramp_two_lane
        else:
            one, two = 3.6, 6.5
        w = sh["width"]
        if two_needed and n == 1:
            need = f"{'곡선' if sh['kind'] == 'curve' else '직선'} 2차로 {two:.1f}m 이상 (총 50대 넘음, 경사로 하나)"
            ok = w >= two - 0.05
        else:
            need = f"{'곡선' if sh['kind'] == 'curve' else '직선'} {one:.1f}m 이상" + (" (경사로 여러 개: 들어가는·나가는 쪽 분리로 봄)" if two_needed else "")
            ok = w >= one - 0.05
        out.append(dict(item=f"{nm} 너비", need=need, have=f"{w:.2f}m", ok=bool(ok), ramp=True))
        if sh["kind"] == "curve":
            rmin = 5.0 if total_legal <= 50 else 6.0
            out.append(dict(item=f"{nm} 곡선 내변 반경", need=f"{rmin:.0f}m 이상 (총 {total_legal}대)",
                            have=f"{sh['inner_r']:.2f}m (외변 {sh['outer_r']:.1f}m)", ok=bool(sh["inner_r"] >= rmin - 0.05),
                            ramp=True))
    return out
