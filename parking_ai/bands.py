"""줄·차로 쌓기를 동적계획법(DP)으로.

예전 방식은 [주차줄 | 차로 | 주차줄] 모듈을 같은 간격으로 반복해 쌓았다. 그러면 코어가 모듈보다 깊을 때
(예: 코어 13.7m > 두 줄 10.4m) 차로가 코어에 막히고, 대지 끝에서 남는 폭을 쓸 수 없다.

여기서는 y 방향(차로와 수직)을 h(0.1~0.2m) 간격으로 나누고, 아래에서 위로 다음 단위를 고르는
1차원 배낭 문제로 푼다 (Zhang·Li·Yu, CAADRIA 2023 의 '줄 종류 정수 배낭'을 비정형 대지·코어에 맞게 DP 로).
  - D(t1,t2): [줄 t1 (앞이 위) | 차로 | 줄 t2 (앞이 아래)]  양쪽 주차 차로
  - S_up(t):  [줄 t (앞이 위) | 차로]                          한쪽 주차 차로(차로가 위)
  - S_dn(t):  [차로 | 줄 t (앞이 아래)]                         한쪽 주차 차로(차로가 아래)
  - G:        빈 띠 h
  t ∈ {일반, 확장}. 값 = 그 높이에서 대지·코어를 피해 구획이 들어가고, 앞이 차로(막히지 않은 구간)에 닿는 구획 수.
확장형 비율은 라그랑주 승수 λ 로: 점수 = 구획 수 + λ·(확장 − 비율 × 구획 수). λ 를 이분 탐색해 비율을 맞춘다.
차로끼리의 연결(세로 연결 차로)은 그 다음 단계(Layout.solve)가 정한다.
"""
import math

import numpy as np
from shapely.geometry import box

from . import engine as E

TOL = 1e-6


def _count(ivs, W):
    return sum(int(math.floor((b - a) / W + 1e-7)) for a, b in ivs)


def _inter(A, B):
    out = []
    i = j = 0
    while i < len(A) and j < len(B):
        lo, hi = max(A[i][0], B[j][0]), min(A[i][1], B[j][1])
        if hi > lo + TOL:
            out.append((lo, hi))
        if A[i][1] < B[j][1]:
            i += 1
        else:
            j += 1
    return out


class DPY:
    """YLayout 과 같은 모양(rows, aisles)을 내는 DP 결과."""

    def __init__(self, frame, rules, ang, rows, aisles, tag):
        self.frame, self.rules, self.ang = frame, rules, ang
        self.mask, self.y0 = tag, 0.0
        self.aisle_w = rules.aisle_for(ang)
        self.rows, self.aisles = rows, aisles

    solve_bands = E.YLayout.solve_bands


def _tables(frame, rules, region, h):
    minx, miny, maxx, maxy = frame.bounds
    lo, hi = minx - 1, maxx + 1
    a = rules.aisle_for(90)
    N = int(math.ceil((maxy - miny) / h)) + 1
    ys = miny + np.arange(N) * h
    depth = {"N": rules.stall_l, "E": rules.ext_l}
    tabs = {}
    for key, d in (("N", depth["N"]), ("E", depth["E"]), ("A", a)):
        boxes = [box(lo, y + E.EPS, hi, y + d - E.EPS) for y in ys]
        g = E._good_intervals(region, boxes, 0, lo, hi)
        if key == "A":
            g = [[iv for iv in gi if iv[1] - iv[0] >= rules.stall_w] for gi in g]
        tabs[key] = g
    return ys, tabs, depth, a


def solve(frame, rules, pre, lam):
    ys, T, depth, a, h = pre
    N = len(ys)
    st = lambda d: int(round(d / h))
    na = st(a)
    Wd = {"N": rules.stall_w, "E": rules.ext_w}
    r = rules.ext_ratio

    def rowval(t, i_row, i_aisle):
        if i_row >= N or i_aisle >= N or i_row < 0 or i_aisle < 0:
            return 0, 0
        n = _count(_inter(T[t][i_row], T["A"][i_aisle]), Wd[t])
        return n, (n if t == "E" else 0)

    units = []
    for t1 in ("N", "E"):
        for t2 in ("N", "E"):
            units.append(("D", t1, t2, st(depth[t1]) + na + st(depth[t2])))
        units.append(("U", t1, None, st(depth[t1]) + na))
        units.append(("L", None, t1, na + st(depth[t1])))
    NEG = -1e18
    best = np.full(N + 1, NEG)
    best[0] = 0.0
    choice = [None] * (N + 1)
    cnt = [(0, 0)] * (N + 1)
    for i in range(N + 1):
        if best[i] <= NEG / 2:
            continue
        # 빈 띠: 같은 대수면 빈 띠를 가운데(코어 쪽)로 — 벽에 붙은 줄을 남김
        if i + 1 <= N:
            gv = best[i] - 1e-3 * (1.0 + 2.0 * abs(i - N / 2) / max(N, 1))
            if gv > best[i + 1]:
                best[i + 1] = gv
                choice[i + 1] = (i, None)
                cnt[i + 1] = cnt[i]
        for u in units:
            kind, t1, t2, H = u
            j = i + H
            if j > N:
                continue
            n = e = 0
            if kind == "D":
                ia = i + st(depth[t1])
                n1, e1 = rowval(t1, i, ia)
                n2, e2 = rowval(t2, ia + na, ia)
                n, e = n1 + n2, e1 + e2
            elif kind == "U":
                ia = i + st(depth[t1])
                n, e = rowval(t1, i, ia)
            else:
                n, e = rowval(t2, i + na, i)
            if n == 0:
                continue
            v = best[i] + n + lam * (e - r * n)
            if v > best[j] + 1e-9:
                best[j] = v
                choice[j] = (i, u)
                cnt[j] = (cnt[i][0] + n, cnt[i][1] + e)
    j = N if best[N] > NEG / 2 else int(np.argmax(best))
    seq = []
    while j > 0 and choice[j] is not None:
        i, u = choice[j]
        if u is not None:
            seq.append((i, u))
        j = i
    seq.reverse()
    rows, aisles = [], []
    ri = 0
    for i, (kind, t1, t2, H) in seq:
        y = ys[0] + i * h
        if kind in ("D", "U"):
            d = st(depth[t1]) * h
            rows.append(dict(ya=y, yb=y + d, front=+1, aisle=len(aisles), ext=t1 == "E", ri=ri)); ri += 1
            y += d
        aisles.append((y, y + na * h))
        y += na * h
        if kind in ("D", "L"):
            t = t2
            d = st(depth[t]) * h
            rows.append(dict(ya=y, yb=y + d, front=-1, aisle=len(aisles) - 1, ext=t == "E", ri=ri)); ri += 1
    n_tot, e_tot = cnt[N if best[N] > NEG / 2 else int(np.argmax(best))]
    return rows, aisles, n_tot, e_tot


def dp_layout(frame, rules, region, fixed_ramp=None, margin=0.03):
    """확장형 비율(+여유)을 맞추는 가장 작은 λ 로 DP. 반환 DPY."""
    reg = region
    if fixed_ramp is not None and fixed_ramp.get("aprons"):
        from shapely.ops import unary_union
        reg = region.difference(unary_union(fixed_ramp["aprons"]))
    target = rules.ext_ratio + margin if rules.ext_ratio > 0 else 0

    minx, miny, maxx, maxy = frame.bounds
    h = 0.1 if (maxy - miny) < 90 else 0.2
    pre = _tables(frame, rules, reg, h) + (h,)

    def run(lam):
        return solve(frame, rules, pre, lam)

    rows, aisles, n, e = run(0.0)
    if target > 0 and n >= rules.ext_threshold and e < target * n:
        lo, hi = 0.0, 4.0
        best = run(hi)
        for _ in range(7):
            mid = (lo + hi) / 2
            res = run(mid)
            if res[3] >= target * res[2]:
                hi, best = mid, res
            else:
                lo = mid
        rows, aisles, n, e = best
    return DPY(frame, rules, 90, rows, aisles, "dp")
