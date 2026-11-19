"""경사로 위치 추천: 고른 경사로를 대지 둘레 벽을 따라 옮겨 가며 자리마다 빠르게 배치를 찾고 대수를 비교.

- 후보: 대지 경계(쓸 수 있는 변, 길이 ≥ 경사로 길이)마다 경사로 긴 변을 벽에 붙여 step 간격으로 미끄러뜨림.
  코어·다른 경사로·장애물과 겹치거나 대지 밖이면 뺌. 지금 자리도 넣음.
- 평가: 실시간 이동과 같은 빠른 탐색(격자 방향·주차 각도·둘레 줄 조합은 고른 안 그대로)을 후보마다 한 프로세스에서.
- 결과: [(경사로 도형, 법정 대수, 순위 점수)] — 화면에서 색 지도로 보이고 누르면 그 자리 배치를 엶.
"""
import math

from shapely.geometry import Point, Polygon
from shapely.ops import unary_union


def _dims(rp):
    rr = rp.minimum_rotated_rectangle
    cs = list(rr.exterior.coords)
    e1 = math.hypot(cs[1][0] - cs[0][0], cs[1][1] - cs[0][1])
    e2 = math.hypot(cs[2][0] - cs[1][0], cs[2][1] - cs[1][1])
    return max(e1, e2), min(e1, e2)


def candidates(site, cores, ramps, obstacles, idx, step=None, max_n=80):
    rp = ramps[idx]
    L, W = _dims(rp)
    others = [g for j, g in enumerate(ramps) if j != idx] + list(cores) + list(obstacles)
    block = unary_union(others).buffer(0.3) if others else None
    inside = site.buffer(0.05)
    if step is None:
        step = max(3.0, min(12.0, math.sqrt(site.area) / 10))
    out = []
    ring = list(site.exterior.coords)
    for a, b in zip(ring, ring[1:]):
        el = math.hypot(b[0] - a[0], b[1] - a[1])
        if el < L * 0.9:
            continue
        ux, uy = (b[0] - a[0]) / el, (b[1] - a[1]) / el
        nx, ny = -uy, ux
        mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        if not site.contains(Point(mx + nx * 0.5, my + ny * 0.5)):
            nx, ny = -nx, -ny
        n_st = max(1, int((el - L) // step) + 1)
        span = el - L
        for k in range(n_st):
            s = (span / 2) if n_st == 1 else (span * k / (n_st - 1))
            x0, y0 = a[0] + ux * s + nx * 0.05, a[1] + uy * s + ny * 0.05
            poly = Polygon([(x0, y0), (x0 + ux * L, y0 + uy * L),
                            (x0 + ux * L + nx * W, y0 + uy * L + ny * W), (x0 + nx * W, y0 + ny * W)])
            if not inside.contains(poly):
                continue
            if block is not None and poly.intersects(block):
                continue
            out.append(poly)
    # 너무 많으면 고르게 줄임
    if len(out) > max_n:
        k = len(out) / max_n
        out = [out[int(i * k)] for i in range(max_n)]
    return [rp] + out


def _job(site, cores, ramps, obstacles, rules):
    from . import engine
    try:
        res = engine.search(site, cores, ramps, obstacles, rules, log=lambda m: None, workers=1)
    except Exception:
        return False
    if not res:
        return False
    st = res[0]["stats"]
    ok = res[0]["rank"][0] == 1
    return st["total_legal"], float(st["score"]), ok


def scan(site, cores, ramps, obstacles, rules, idx, executor=None, progress=None, stop=None):
    """반환: [dict(poly=[[x,y]..], total, score, ok, current)]"""
    from . import engine
    cands = candidates(site, cores, ramps, obstacles, idx, max_n=40 if site.area > 20000 else 80)
    argsets = []
    for c in cands:
        rs = list(ramps)
        rs[idx] = c
        argsets.append((site, cores, rs, obstacles, rules))
    out = engine._run(executor, _job, argsets, stop, (lambda i, n: progress(i, n)) if progress else None)
    res = []
    for i, (c, r) in enumerate(zip(cands, out)):
        if r is None or r is False:
            continue
        res.append(dict(poly=[[round(x, 3), round(y, 3)] for x, y in c.exterior.coords], total=r[0], score=round(r[1], 1),
                        ok=r[2], current=(i == 0)))
    return res
