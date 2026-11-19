"""자투리 채우기: 주 배치가 끝난 뒤, 차로 가장자리 어디든(연결 차로 옆, 코어·경사로 옆, 깎인 모서리)
남는 땅에 차로에 수직으로 구획을 더 넣는다.

- 차로 면(가로 차로 + 연결 차로)의 경계 선분마다, 바깥쪽으로 구획(너비 W, 깊이 L)을 0.1m씩 밀며
  들어가는 첫 자리에 놓고 W 만큼 건너뛰는 욕심쟁이 채우기.
- 구획은 대지 안, 기존 구획·기둥·코어·경사로·차로·앞마당과 겹치지 않아야 한다.
- 확장형 비율이 모자라지 않도록 필요한 만큼 확장형으로 놓는다. 일반이 안 들어가면 경형.
"""
import math

import shapely
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

TOL = 1e-6
CELL = 5.0


def _segments(g):
    out = []
    polys = [g] if g.geom_type == "Polygon" else list(getattr(g, "geoms", []))
    for p in polys:
        if p.geom_type != "Polygon":
            continue
        for ring in [p.exterior] + list(p.interiors):
            cs = list(ring.coords)
            for a, b in zip(cs, cs[1:]):
                L = math.hypot(b[0] - a[0], b[1] - a[1])
                if L > 1.9:
                    out.append((a, b, L))
    return out


def _rect(p, t, n, s, W, L):
    x0, y0 = p[0] + t[0] * s, p[1] + t[1] * s
    x1, y1 = x0 + t[0] * W, y0 + t[1] * W
    return Polygon([(x0, y0), (x1, y1), (x1 + n[0] * L, y1 + n[1] * L), (x0 + n[0] * L, y0 + n[1] * L)])


def fill(region, lanes, blockers, sizes, n_now, n_ext_now, ext_ratio, ext_threshold, step=0.1):
    """lanes: 구획이 마주볼 차로 면. blockers: 겹치면 안 되는 것(구획·기둥·코어·경사로·앞마당·차로 포함).
    sizes: dict N/E/C → (W, L). 반환: [(kind, poly, into(ux,uy), W, L)]
    겹침 검사는 공간 색인(STRtree)으로: 큰 대지에서 수천 개 도형을 합치지 않음."""
    step = max(step, 0.1)
    out = []
    if lanes is None or lanes.is_empty:
        return out
    blk = [g for g in blockers if g is not None and not g.is_empty]
    tree = shapely.STRtree(blk) if blk else None
    shapely.prepare(region)
    placed, pb, grid = [], [], {}   # 이번에 놓은 구획·범위·5m 칸 색인
    total, n_ext = n_now, n_ext_now
    segs = _segments(lanes)
    segs.sort(key=lambda t: -t[2])

    def free(r):
        rs = r.buffer(-1e-3, join_style=2)
        if tree is not None and len(tree.query(rs, predicate="intersects")):
            return False
        b = rs.bounds
        seen = set()
        for cx in range(int(math.floor(b[0] / CELL)), int(math.floor(b[2] / CELL)) + 1):
            for cy in range(int(math.floor(b[1] / CELL)), int(math.floor(b[3] / CELL)) + 1):
                for j in grid.get((cx, cy), ()):
                    if j in seen:
                        continue
                    seen.add(j)
                    qb = pb[j]
                    if qb[0] < b[2] and qb[2] > b[0] and qb[1] < b[3] and qb[3] > b[1] and placed[j].intersects(rs):
                        return False
        return True

    def add(r):
        j = len(placed)
        placed.append(r)
        b = r.bounds
        pb.append(b)
        for cx in range(int(math.floor(b[0] / CELL)), int(math.floor(b[2] / CELL)) + 1):
            for cy in range(int(math.floor(b[1] / CELL)), int(math.floor(b[3] / CELL)) + 1):
                grid.setdefault((cx, cy), []).append(j)

    for a, b, Lseg in segs:
        t = ((b[0] - a[0]) / Lseg, (b[1] - a[1]) / Lseg)
        # 바깥쪽 법선(차로 면 밖으로)
        n = (t[1], -t[0])
        mid = ((a[0] + b[0]) / 2 + n[0] * 0.05, (a[1] + b[1]) / 2 + n[1] * 0.05)
        if lanes.contains(shapely.Point(mid)):
            n = (-n[0], -n[1])
        # 이 변 앞 띠에서 빈 곳만 훑기: 띠 ∩ 대지 − 주변 막는 것 → 변 방향으로 투영한 구간
        Lmin = min(v[1] for v in sizes.values())
        Wmin = min(v[0] for v in sizes.values())
        strip = _rect(a, t, n, 0.0, Lseg, Lmin).buffer(-1e-3, join_style=2)
        near = [blk[i] for i in tree.query(strip, predicate="intersects")] if tree is not None else []
        near += [q for q, qb in zip(placed, pb) if q.intersects(strip)] if placed else []
        freeg = strip.intersection(region)
        if near and not freeg.is_empty:
            freeg = freeg.difference(unary_union(near))
        if freeg.is_empty or freeg.area < Wmin * Lmin * 0.9:
            continue
        spans = []
        for g in getattr(freeg, "geoms", [freeg]):
            if g.is_empty or g.area < 0.5:
                continue
            cs = list(g.exterior.coords) if g.geom_type == "Polygon" else list(g.coords)
            ts_ = [(x - a[0]) * t[0] + (y - a[1]) * t[1] for x, y in cs]
            spans.append((max(0.0, min(ts_) - 0.05), min(Lseg, max(ts_) + 0.05)))
        spans.sort()
        if not spans:
            continue
        si = 0
        s = spans[0][0]
        while s <= Lseg - 1.9:
            while si < len(spans) and s > spans[si][1] - Wmin + 1e-6:
                si += 1
                if si < len(spans):
                    s = max(s, spans[si][0])
            if si >= len(spans):
                break
            need_ext = (total + 1) >= ext_threshold and n_ext < ext_ratio * (total + 1) - 1e-9
            order = ["E", "N", "C"] if need_ext else ["N", "E", "C"]
            ok = None
            for k in order:
                W, L = sizes[k]
                if s + W > Lseg + TOL:
                    continue
                r = _rect(a, t, n, s, W, L)
                if region.contains(r) and free(r):
                    ok = (k, r, W, L)
                    break
            if ok is None:
                s += step
                continue
            k, r, W, L = ok
            add(r)
            out.append((k, r, n, W, L))
            total += 1
            n_ext += 1 if k == "E" else 0
            s += W
    return out
