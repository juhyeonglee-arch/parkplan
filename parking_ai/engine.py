"""지하주차장 배치 엔진.

생각의 틀
- 대지를 돌려서 차로가 x축과 나란하게 놓이게 한다(격자 방향 θ).
- y 방향으로 [주차줄][차로][주차줄] 모듈을 쌓는다(시작 위치 y0). 줄마다 일반/확장형을 정한다.
- x 방향으로 기둥 간격 P 의 칸(bay)을 깐다(시작 위치 x0). 기둥은 칸 경계, 주차줄의 차로 쪽 끝.
- 칸 하나를 통째로 비워 세로 연결 차로(회차로)를 만든다. 어느 칸을 비울지는 욕심쟁이 탐색으로 고른다.
- 경사로 입구와 이어진 차로에 앞이 닿은 구획만 센다.
- 대수가 가장 많은 안 → 같은 대수면 대당 면적이 작은 안.

빠른 탐색은 띠(band)마다 대지 밖 구간을 1차원 구간으로 바꿔 계산하고(직사각형 구획은 정확),
마지막 몇 안만 실제 도형으로 다시 확인하면서 경형 채우기·장애인 구획·경사로·기둥을 넣는다.
"""
import math
import time

import numpy as np
import shapely
from shapely import affinity
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

from . import circ, columns, fill
from .rules import Rules

EPS = 1e-4
TOL = 1e-6


# ------------------------------------------------------------------ 구간 도구
def _merge(iv):
    iv = sorted(iv)
    out = []
    for a, b in iv:
        if out and a <= out[-1][1] + TOL:
            out[-1][1] = max(out[-1][1], b)
        else:
            out.append([a, b])
    return out


def _complement(bad, lo, hi):
    good, cur = [], lo
    for a, b in _merge(bad):
        if a > cur + TOL:
            good.append((cur, min(a, hi)))
        cur = max(cur, b)
        if cur >= hi:
            break
    if cur < hi - TOL:
        good.append((cur, hi))
    return good


def _good_intervals(region, boxes, axis, lo, hi):
    """boxes(띠) 마다 region 안에 온전히 들어가는 좌표 구간(axis 0 = x, 1 = y) 목록."""
    if len(boxes) == 0:
        return []
    diffs = shapely.difference(np.asarray(boxes, dtype=object), region)
    out = []
    for g in diffs:
        bad = []
        if g is not None and not g.is_empty:
            for p in shapely.get_parts(g):
                if p.area > 1e-7:
                    b = p.bounds
                    bad.append((b[0], b[2]) if axis == 0 else (b[1], b[3]))
        out.append(_complement(bad, lo, hi))
    return out


def _lookup(starts, ends, a, b):
    """[a,b] 를 통째로 품는 구간 번호(없으면 -1). starts/ends 는 정렬된 배열."""
    if len(starts) == 0:
        return np.full(np.shape(a), -1, dtype=int)
    i = np.searchsorted(starts, a + TOL, side="right") - 1
    ok = (i >= 0)
    ic = np.clip(i, 0, len(starts) - 1)
    ok &= ends[ic] >= b - TOL
    return np.where(ok, ic, -1)


def _inter_iv(A, B):
    out = []
    for a0, a1 in A:
        for b0, b1 in B:
            lo, hi = max(a0, b0), min(a1, b1)
            if hi > lo + TOL:
                out.append((lo, hi))
    return sorted(out)


class UF:
    def __init__(self, n):
        self.p = list(range(n))

    def find(self, a):
        p = self.p
        while p[a] != a:
            p[a] = p[p[a]]
            a = p[a]
        return a

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[rb] = ra


# ------------------------------------------------------------------ 좌표 틀
class Frame:
    """격자 방향 θ 로 돌린 좌표계. region = 주차 가능 영역(경계 안쪽 − 코어 − 그린 경사로)."""

    def __init__(self, site, cores, ramps, obstacles, theta, rules: Rules):
        self.theta = theta
        c = site.centroid
        self.origin = (c.x, c.y)
        rot = lambda g: affinity.rotate(g, -theta, origin=self.origin)
        self.site = rot(site)
        usable = site.buffer(-rules.wall_offset, join_style=2) if rules.wall_offset > 0 else site
        holes = [g.buffer(rules.core_clear, join_style=2) if rules.core_clear > 0 else g for g in cores]
        holes += list(obstacles)
        self.cores = [rot(g) for g in cores]
        self.ramps = [rot(g) for g in ramps]          # 그린 경사로
        self.usable = rot(usable)                     # 코어 빼기 전
        region = usable.difference(unary_union(holes + list(ramps))) if (holes or ramps) else usable
        self.region = rot(region)
        shapely.prepare(self.region)
        self.bounds = self.region.bounds

    def to_world(self, g):
        return affinity.rotate(g, self.theta, origin=self.origin)


def grid_angles(site, n):
    """대지 변 방향(길이 가중)으로 격자 방향 후보. 차로가 그 변과 나란한 것과 수직인 것 둘 다."""
    coords = list(site.exterior.coords)
    w = {}
    for (x1, y1), (x2, y2) in zip(coords, coords[1:]):
        L = math.hypot(x2 - x1, y2 - y1)
        if L < 1e-6:
            continue
        a = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 90.0
        key = None
        for k in w:
            if min(abs(k - a), 90 - abs(k - a)) < 1.5:
                key = k
                break
        w[key if key is not None else a] = w.get(key if key is not None else a, 0) + L
    best = sorted(w, key=lambda k: -w[k])[:max(1, n)]
    out = []
    for a in best:
        out += [round(a, 3), round(a + 90.0, 3)]
    return out


# ------------------------------------------------------------------ 줄 배치(y)
def stall_dims(rules, ext):
    return (rules.ext_w, rules.ext_l) if ext else (rules.stall_w, rules.stall_l)


def row_depth(rules, ext, ang):
    W, L = stall_dims(rules, ext)
    if ang == 90:
        return L
    a = math.radians(ang)
    return L * math.sin(a) + W * math.cos(a)


MASKS = {
    "none": (0, ()), "all": (1, (0,)),
    "2a": (2, (0,)), "2b": (2, (1,)),
    "3a": (3, (0,)), "3b": (3, (1,)), "3c": (3, (2,)),
}


def _is_ext(mask, ri):
    q, ph = MASKS[mask]
    return q > 0 and (ri % q) in ph


class YLayout:
    def __init__(self, frame, rules, ang, mask, y0):
        self.ang, self.mask, self.y0 = ang, mask, y0
        a = rules.aisle_for(ang)
        self.aisle_w = a
        minx, miny, maxx, maxy = frame.bounds
        T = row_depth(rules, False, ang) * 2 + a
        y = miny - T + y0
        rows, aisles = [], []
        ri = 0
        while y < maxy:
            for side in (+1, -1):          # +1: 차로가 위(줄 앞이 위), -1: 차로가 아래
                ext = _is_ext(mask, ri)
                d = row_depth(rules, ext, ang)
                rows.append(dict(ya=y, yb=y + d, front=side, aisle=len(aisles) - (0 if side > 0 else 1),
                                 ext=ext, ri=ri))
                ri += 1
                y += d
                if side > 0:
                    aisles.append((y, y + a))
                    y += a
        keep_a = [i for i, (ya, yb) in enumerate(aisles) if yb > miny + 0.5 and ya < maxy - 0.5]
        remap = {old: new for new, old in enumerate(keep_a)}
        self.aisles = [aisles[i] for i in keep_a]
        self.rows = []
        for r in rows:
            if r["aisle"] in remap and r["yb"] > miny and r["ya"] < maxy:
                r["aisle"] = remap[r["aisle"]]
                self.rows.append(r)
        self.frame = frame
        self.rules = rules

    def solve_bands(self, region):
        minx, miny, maxx, maxy = self.frame.bounds
        lo, hi = minx - 1, maxx + 1
        rb = [box(lo, r["ya"] + EPS, hi, r["yb"] - EPS) for r in self.rows]
        ab = [box(lo, ya + EPS, hi, yb - EPS) for ya, yb in self.aisles]
        g = _good_intervals(region, rb + ab, 0, lo, hi)
        row_good = g[:len(rb)]
        aisle_good = [[iv for iv in gi if iv[1] - iv[0] >= self.rules.stall_w] for gi in g[len(rb):]]
        return row_good, aisle_good


# ------------------------------------------------------------------ 벽 따라 한 줄
def wall_spans(frame, rules):
    """wall_rows: 차로 끝쪽(프레임 x 양 끝) 벽을 따라 구획 한 줄 + 그 앞 가로 차로를 둔다.
    반환: (잘라 낼 벽 띠 도형 목록, 강제 연결 차로 (xl, xr) 목록)."""
    walls = tuple(getattr(rules, "wall_rows", ()) or ())
    if not walls:
        return [], []
    D = max(rules.stall_l, rules.ext_l)
    la = rules.link_aisle
    minx, miny, maxx, maxy = frame.bounds
    cuts, spans = [], []
    for w in walls:
        if w == "hi":
            cuts.append(box(maxx - D, miny - 1, maxx + 1, maxy + 1))
            spans.append((maxx - D - la, maxx - D))
        elif w == "lo":
            cuts.append(box(minx - 1, miny - 1, minx + D, maxy + 1))
            spans.append((minx + D, minx + D + la))
    return cuts, spans


_WR_CACHE = {}
_GI_CACHE = {}


def _gkey(g):
    return (id(g), round(g.area, 6), tuple(round(v, 6) for v in g.bounds))


def wall_region(frame, rules, region):
    cuts, _ = wall_spans(frame, rules)
    if not cuts:
        return region
    key = (_gkey(region), tuple(getattr(rules, "wall_rows", ()) or ()), round(max(rules.stall_l, rules.ext_l), 3))
    hit = _WR_CACHE.get(key)
    if hit is not None and hit[0] is region:
        return hit[1]
    if len(_WR_CACHE) > 64:
        _WR_CACHE.clear()
    out = region.difference(unary_union(cuts))
    _WR_CACHE[key] = (region, out)          # region 을 붙잡아 id 재사용을 막음
    return out


def _good_intervals_cached(region, spans, miny, maxy):
    """세로 띠(연결 차로 후보)마다 들어가는 y 구간. 같은 대지·같은 띠는 다시 계산하지 않음."""
    rk = _gkey(region)
    ent = _GI_CACHE.get(rk)
    if ent is None or ent[0] is not region:
        if len(_GI_CACHE) > 16:
            _GI_CACHE.clear()
        ent = (region, {})
        _GI_CACHE[rk] = ent
    memo = ent[1]
    keys = [(round(xl, 4), round(xr, 4)) for xl, xr in spans]
    miss = [i for i, k in enumerate(keys) if k not in memo]
    if miss:
        bx = [box(spans[i][0] + EPS, miny - 1, spans[i][1] - EPS, maxy + 1) for i in miss]
        for i, g in zip(miss, _good_intervals(region, bx, 1, miny - 1, maxy + 1)):
            memo[keys[i]] = g
        if len(memo) > 20000:
            memo.clear()
    return [memo.get(k) if k in memo else _good_intervals(region, [box(xl + EPS, miny - 1, xr - EPS, maxy + 1)], 1, miny - 1, maxy + 1)[0]
            for k, (xl, xr) in zip(keys, spans)]


# ------------------------------------------------------------------ 한 안 계산
class Layout:
    """(격자 방향, 주차 각도, 확장형 줄, y0, 기둥 간격 P, x0) 하나에 대한 빠른 계산."""

    def __init__(self, frame, rules, Y: YLayout, P, x0, region=None, bands=None, ramp=None):
        self.frame, self.rules, self.Y, self.P, self.x0 = frame, rules, Y, P, x0
        self.base_region = frame.region if region is None else region
        self.wall_cuts, self.wall_x = wall_spans(frame, rules)
        self.region = wall_region(frame, rules, self.base_region)
        self.ramp = ramp                    # dict(poly, mouth=(x,y)) 프레임 좌표
        row_good, aisle_good = bands if bands is not None else Y.solve_bands(self.region)
        self.row_good, self.aisle_good = row_good, aisle_good
        c = rules.column_w
        self.c = c
        minx, miny, maxx, maxy = frame.bounds
        base = minx - P + x0
        nb = int(math.ceil((maxx - base) / P)) + 1
        self.bx = base + np.arange(nb) * P          # 칸 시작(기둥 왼쪽 면)
        ang = Y.ang

        # 차로 조각(노드)
        self.nodes = []                             # (aisle idx, x0, x1)
        self.aisle_nodes = []
        for ai, ivs in enumerate(aisle_good):
            ids = []
            for a0, a1 in ivs:
                ids.append(len(self.nodes))
                self.nodes.append((ai, a0, a1))
            self.aisle_nodes.append(ids)
        self.ramp_node = len(self.nodes)
        n_ap = len(ramp.get("aprons") or []) if ramp is not None else 0
        self.n_ap = n_ap
        nn = self.ramp_node + max(1, n_ap)

        # 구획 칸(slot)
        R, X, E, F0, F1, BAY = [], [], [], [], [], []
        sa = math.sin(math.radians(ang))
        ca = math.cos(math.radians(ang))
        cw = P - c
        for ridx, r in enumerate(Y.rows):
            W, L = stall_dims(rules, r["ext"])
            if ang == 90 and c < TOL:
                # 1단계(기둥 없음): 줄마다 들어가는 구간(대지 안 ∩ 앞 차로) 시작점에 맞춰 끊김 없이
                e, f0, f1 = W, 0.0, W
                ivs = _inter_iv(row_good[ridx], aisle_good[r["aisle"]] if r["aisle"] < len(aisle_good) else [])
                parts = [a_ + np.arange(int(math.floor((b_ - a_) / W + 1e-7))) * W for a_, b_ in ivs]
                xs = np.concatenate(parts) if parts else np.zeros(0)
                ks = np.floor((xs - base) / P + 1e-9).astype(int)
            elif ang == 90:
                e, p, f0, f1 = W, W, 0.0, W
                if cw < e - TOL:
                    continue
                m = int(math.floor((cw - e) / p + 1e-9)) + 1
                xs = (self.bx[:, None] + c + np.arange(m)[None, :] * p).ravel()
                ks = np.repeat(np.arange(nb), m)
            else:
                # 비스듬한 주차는 줄을 따라 이어서 깔고, 기둥은 줄 뒤쪽 삼각 틈에 둔다
                e, p = W * sa + L * ca, W / sa
                lean = 1 if r["aisle"] % 2 == 0 else -1
                f0, f1 = (0.0, W * sa) if lean > 0 else (e - W * sa, e)
                n = int(math.ceil((maxx - base) / p)) + 1
                xs = base + x0 * 0 + np.arange(n) * p
                ks = np.floor((xs - base) / P).astype(int)
            R.append(np.full(len(xs), ridx)); X.append(xs); E.append(np.full(len(xs), e))
            F0.append(xs + f0); F1.append(xs + f1); BAY.append(ks)
        if R:
            self.s_row = np.concatenate(R); self.s_x = np.concatenate(X); self.s_e = np.concatenate(E)
            self.s_f0 = np.concatenate(F0); self.s_f1 = np.concatenate(F1); self.s_bay = np.concatenate(BAY)
        else:
            z = np.zeros(0)
            self.s_row = z.astype(int); self.s_x = self.s_e = self.s_f0 = self.s_f1 = z; self.s_bay = z.astype(int)
        ns = len(self.s_x)
        self.s_fit = np.zeros(ns, bool)
        self.s_node = np.full(ns, -1)
        self.s_ext = np.array([Y.rows[i]["ext"] for i in self.s_row], bool) if ns else np.zeros(0, bool)
        for ridx, r in enumerate(Y.rows):
            sel = np.nonzero(self.s_row == ridx)[0]
            if not len(sel):
                continue
            gi = row_good[ridx]
            st = np.array([a for a, b in gi]); en = np.array([b for a, b in gi])
            self.s_fit[sel] = _lookup(st, en, self.s_x[sel], self.s_x[sel] + self.s_e[sel]) >= 0
            ids = self.aisle_nodes[r["aisle"]]
            if ids:
                st = np.array([self.nodes[i][1] for i in ids]); en = np.array([self.nodes[i][2] for i in ids])
                j = _lookup(st, en, self.s_f0[sel], self.s_f1[sel])
                self.s_node[sel] = np.where(j >= 0, np.array(ids)[np.clip(j, 0, len(ids) - 1)], -1)
        self.s_ya = np.array([Y.rows[i]["ya"] for i in self.s_row]) if ns else np.zeros(0)
        self.s_yb = np.array([Y.rows[i]["yb"] for i in self.s_row]) if ns else np.zeros(0)

        # 경사로 입구 앞마당(apron): 구획을 두지 않고, 닿는 차로와 이어 줌
        self.base_links = []
        self.aprons = [a for a in (ramp.get("aprons") or [])] if ramp is not None else []
        self.apron_bufs = [a.buffer(0.1, join_style=2) for a in self.aprons]
        if self.aprons and ns:
            apu = unary_union(self.aprons).buffer(-1e-3)
            ab = apu.bounds
            cand = np.nonzero((self.s_x < ab[2]) & (self.s_x + self.s_e > ab[0]) &
                              (self.s_ya < ab[3]) & (self.s_yb > ab[1]))[0]
            if len(cand):
                bx_ = shapely.box(self.s_x[cand], self.s_ya[cand], self.s_x[cand] + self.s_e[cand], self.s_yb[cand])
                hit = shapely.intersects(bx_, apu)
                self.s_fit[cand[hit]] = False
        for j, apb in enumerate(self.apron_bufs):
            b = apb.bounds
            for nid, (ai, a0, a1) in enumerate(self.nodes):
                ya, yb = Y.aisles[ai]
                if a1 < b[0] or a0 > b[2] or yb < b[1] or ya > b[3]:
                    continue
                if apb.intersection(box(a0, ya, a1, yb)).area >= 0.3:
                    self.base_links.append((self.ramp_node + j, nid))

        # 연결 차로 후보: 칸 kc 개를 비움
        la = rules.link_aisle
        self.kc = max(1, int(math.ceil((la + c) / P - 1e-9)))
        self.conn = []
        boxes, spans = [], []
        for k in range(nb - self.kc + 1):
            xl = self.bx[k] + c
            xr = self.bx[k] + self.kc * P
            spans.append((xl, xr))
            boxes.append(box(xl + EPS, miny - 1, xr - EPS, maxy + 1))
        # 코어·경사로 옆에 붙인 연결 차로 후보(어차피 구획이 끊기는 자리라 손실이 적음)
        if c < TOL and rules.conn_snap:
            snaps = set()
            for g in list(frame.cores) + list(frame.ramps) + list(self.aprons):
                b = g.bounds
                snaps.add(round(b[0], 2)); snaps.add(round(b[2], 2))
            for e in sorted(snaps):
                for xl, xr in ((e - la, e), (e, e + la)):
                    if xl > minx - la and xr < maxx + la:
                        spans.append((xl, xr))
                        boxes.append(box(xl + EPS, miny - 1, xr - EPS, maxy + 1))
        n_reg = len(spans)
        for xl, xr in self.wall_x:
            spans.append((xl, xr))
            boxes.append(box(xl + EPS, miny - 1, xr - EPS, maxy + 1))
        goods = _good_intervals_cached(self.region, spans, miny, maxy)
        for k, ((xl, xr), gi) in enumerate(zip(spans, goods)):
            pieces = []
            for g0, g1 in gi:
                contacts, lo_y, hi_y = [], None, None
                for nid, (ai, a0, a1) in enumerate(self.nodes):
                    ya, yb = Y.aisles[ai]
                    if ya >= g0 - TOL and yb <= g1 + TOL and min(a1, xr) - max(a0, xl) >= 0.9 * min(la, xr - xl):
                        contacts.append(nid)
                        lo_y = ya if lo_y is None else min(lo_y, ya)
                        hi_y = yb if hi_y is None else max(hi_y, yb)
                has_ramp = []
                for j, apb in enumerate(self.apron_bufs):
                    b = apb.bounds
                    if b[2] < xl or b[0] > xr or b[3] < g0 or b[1] > g1:
                        continue
                    if apb.intersection(box(xl, g0, xr, g1)).area >= 0.3:
                        has_ramp.append(self.ramp_node + j)
                        lo_y = b[1] if lo_y is None else min(lo_y, b[1])
                        hi_y = b[3] if hi_y is None else max(hi_y, b[3])
                if len(contacts) + len(has_ramp) < 2:
                    continue
                lo_y, hi_y = max(lo_y, g0), min(hi_y, g1)
                rm = np.nonzero((self.s_x < xr - TOL) & (self.s_x + self.s_e > xl + TOL) &
                                (self.s_ya < hi_y - TOL) & (self.s_yb > lo_y + TOL))[0]
                pieces.append(dict(contacts=contacts, ramp=has_ramp, y0=lo_y, y1=hi_y, rm=rm, xl=xl, xr=xr))
            self.conn.append(pieces)
        # 벽 앞 가로 차로는 항상 둔다
        self.forced = [k for k in range(n_reg, len(spans)) if self.conn[k]]
        # 벽 띠에 들어갈 구획 수 어림(1단계 점수에 더함: 실제 배치는 realize 의 자투리 채우기가 함)
        self.wall_est = 0
        if self.wall_cuts:
            D = max(rules.stall_l, rules.ext_l)
            wb = []
            for (xl, xr), cut in zip(self.wall_x, self.wall_cuts):
                x0w = xr if cut.bounds[0] > xl else xl - D
                wb.append(box(x0w + EPS, miny - 1, x0w + D - EPS, maxy + 1))
            reg_w = self.base_region
            if self.aprons:
                reg_w = reg_w.difference(unary_union(self.aprons))
            for gi in _good_intervals(reg_w, wb, 1, miny - 1, maxy + 1):
                self.wall_est += sum(int(math.floor((b - a) / rules.stall_w + 1e-7)) for a, b in gi)
        self.nn = nn

    # ---------------------------------------------------------- 연결 차로 고르기
    def score(self, S):
        if self.forced:
            S = list(S) + [k for k in self.forced if k not in S]
        uf = UF(self.nn)
        for a, b in self.base_links:
            uf.union(a, b)
        removed = np.zeros(len(self.s_x), bool)
        for k in S:
            for pc in self.conn[k]:
                cs = list(pc["contacts"]) + pc["ramp"]
                for a in cs[1:]:
                    uf.union(cs[0], a)
                removed[pc["rm"]] = True
        alive = self.s_fit & ~removed & (self.s_node >= 0)
        roots = np.array([uf.find(i) for i in range(self.nn)])
        comp = np.where(self.s_node >= 0, roots[np.clip(self.s_node, 0, None)], -1)
        if self.ramp is not None:
            main = uf.find(self.ramp_node)
            # 경사로 양 끝(아래층으로 내려가는 쪽·위층에서 내려오는 쪽)이 모두 이어져야 함
            if any(uf.find(self.ramp_node + j) != main for j in range(1, self.n_ap)):
                alive = alive & (roots[np.clip(self.s_node, 0, None)] == main) & (self.s_node >= 0)
                return int(alive.sum()) - 10000, alive, main, roots
        else:
            if not alive.any():
                return 0, alive, -1, roots
            cnt = np.bincount(comp[alive], minlength=self.nn)
            main = int(np.argmax(cnt))
        alive = alive & (comp == main)
        return int(alive.sum()) + self.wall_est, alive, main, roots

    def _greedy_fast(self, cand):
        """score(S+[k]) 를 처음부터 다시 세지 않고, 지금 연결 상태(UF)에서 k 를 더했을 때만 따짐.
        결과(고르는 순서·값)는 예전 방식과 같고, 큰 대지에서 수십 배 빠름."""
        nn = self.nn
        base = self.s_fit & (self.s_node >= 0)
        uf = UF(nn)
        for a_, b_ in self.base_links:
            uf.union(a_, b_)
        removed = np.zeros(len(self.s_x), bool)

        def commit(k):
            for pc in self.conn[k]:
                cs = list(pc["contacts"]) + pc["ramp"]
                for a_ in cs[1:]:
                    uf.union(cs[0], a_)
                removed[pc["rm"]] = True

        for k in self.forced:
            commit(k)
        rm_alive = {}
        for k in cand:
            idx = np.concatenate([pc["rm"] for pc in self.conn[k]]) if self.conn[k] else np.zeros(0, int)
            idx = np.unique(idx.astype(int)) if len(idx) else idx.astype(int)
            rm_alive[k] = idx[base[idx]] if len(idx) else idx
        has_ramp = self.ramp is not None
        aps = [self.ramp_node + j for j in range(self.n_ap)] if has_ramp else []
        S = []

        def state():
            roots = np.fromiter((uf.find(i) for i in range(nn)), int, nn)
            alive = base & ~removed
            cnt = np.bincount(roots[self.s_node[alive]], minlength=nn).astype(np.int64)
            return roots, cnt

        def value(roots, cnt, k):
            new = cnt.copy()
            ri = self.s_node[rm_alive[k]]
            ri = ri[~removed[rm_alive[k]]]
            if len(ri):
                np.subtract.at(new, roots[ri], 1)
            # 조각마다 이어 붙는 뿌리 묶음(작은 UF)
            par = {}

            def f(x):
                while par.get(x, x) != x:
                    x = par[x]
                return x
            for pc in self.conn[k]:
                rs = [int(roots[m]) for m in list(pc["contacts"]) + pc["ramp"]]
                r0 = f(rs[0])
                for r in rs[1:]:
                    r1 = f(r)
                    if r1 != r0:
                        par[r1] = r0
            groups = {}
            for r in par:
                groups.setdefault(f(r), set()).add(r)
            for g0, G in groups.items():
                G.add(g0)
                tot = sum(int(new[r]) for r in G)
                for r in G:
                    new[r] = 0
                new[g0] = tot
            if has_ramp:
                mr = f(int(roots[self.ramp_node]))
                if any(f(int(roots[a_])) != mr for a_ in aps[1:]):
                    return int(new[mr]) - 10000
                return int(new[mr])
            return int(new.max()) if len(new) else 0

        best = self.score([])[0] - self.wall_est
        while True:
            roots, cnt = state()
            gain_k, gain_v = None, best
            for k in cand:
                if k in S:
                    continue
                v = value(roots, cnt, k)
                if v > gain_v:
                    gain_k, gain_v = k, v
            if gain_k is None:
                break
            S.append(gain_k)
            commit(gain_k)
            best = gain_v
        return S

    def solve(self):
        cand = [k for k in range(len(self.conn)) if self.conn[k] and k not in self.forced]
        S = self._greedy_fast(cand)
        best = self.score(S)[0]
        # 빼서 나아지는 것이 있으면 뺌
        improved = True
        while improved and S:
            improved = False
            for k in list(S):
                T = [x for x in S if x != k]
                v = self.score(T)[0]
                if v >= best:
                    S, best, improved = T, v, True
                    break
        return self.apply(S)

    def add_pair_connectors(self):
        """동선 다듬기용: 연결 차로를 이웃한 두 차로 사이만 잇는 짧은 토막으로도 후보에 넣는다."""
        Y = self.Y
        n0 = len(self.conn)
        seen = set()
        for k in range(n0):
            for pc in self.conn[k]:
                items = []
                for nid in pc["contacts"]:
                    ya, yb = Y.aisles[self.nodes[nid][0]]
                    items.append((ya, yb, nid, False))
                for rn in pc["ramp"]:
                    b = self.apron_bufs[rn - self.ramp_node].bounds
                    items.append((max(b[1], pc["y0"]), min(b[3], pc["y1"]), rn, True))
                items.sort()
                if len(items) <= 2:
                    continue
                for a, b in zip(items, items[1:]):
                    y0, y1 = min(a[0], b[0]), max(a[1], b[1])
                    key = (round(pc["xl"], 3), round(y0, 3), round(y1, 3))
                    if key in seen:
                        continue
                    seen.add(key)
                    rm = np.nonzero((self.s_x < pc["xr"] - TOL) & (self.s_x + self.s_e > pc["xl"] + TOL) &
                                    (self.s_ya < y1 - TOL) & (self.s_yb > y0 + TOL))[0]
                    self.conn.append([dict(contacts=[t[2] for t in (a, b) if not t[3]],
                                           ramp=[t[2] for t in (a, b) if t[3]], y0=y0, y1=y1, rm=rm,
                                           xl=pc["xl"], xr=pc["xr"])])

    def apply(self, S):
        self.S = list(S) + [k for k in getattr(self, "forced", []) if k not in S]
        self.count, self.alive, self.main, self.roots = self.score(self.S)
        self.n_ext = int((self.alive & self.s_ext).sum())
        return self.count


# ------------------------------------------------------------------ 경사로 후보
def ramp_size(rules, two_lane):
    w = (rules.ramp_two_lane if two_lane else rules.ramp_lane) + 2 * rules.ramp_wall
    L = rules.floor_h / rules.ramp_slope + rules.ramp_extra
    return w, L


def make_apron(mid, n, width, depth, region):
    """경사로 입구 변(가운데 mid, 바깥 방향 n) 앞에 너비 width, 깊이 depth 의 앞마당."""
    nx, ny = n
    tx, ty = -ny, nx
    mx, my = mid
    h = width / 2
    p = Polygon([(mx - tx * h, my - ty * h), (mx + tx * h, my + ty * h),
                 (mx + tx * h + nx * depth, my + ty * h + ny * depth), (mx - tx * h + nx * depth, my - ty * h + ny * depth)])
    a = p.intersection(region)
    if a.is_empty or a.area < 0.5 * p.area:
        return None
    return a


def _with_apron(cd, w, L, rules, region):
    """아래쪽 입구(이 층에서 아래층으로)와 먼 끝(위층에서 내려와 닿는 곳) 둘 다 앞마당을 둔다.
    먼 끝이 대지 경계에 붙으면 지상으로 나가는 경사로로 보고 앞마당 하나만."""
    dx, dy = cd["dir"]
    ap = make_apron(cd["mouth_mid"], (-dx, -dy), w, rules.aisle_90, region)
    if ap is None:
        return None
    aps = [ap]
    far = (cd["mouth_mid"][0] + dx * L, cd["mouth_mid"][1] + dy * L)
    ap2 = make_apron(far, (dx, dy), w, rules.aisle_90, region)
    if ap2 is not None:
        aps.append(ap2)
    cd["aprons"] = aps
    return cd


def ramp_candidates(lay: Layout, w, L):
    """칸에 맞춘 세로 경사로(입구가 차로 쪽)와 줄 뒤쪽에 붙인 가로 경사로."""
    fr, Y, rules = lay.frame, lay.Y, lay.rules
    region = lay.region
    out = []
    c, P = lay.c, lay.P
    nk = max(1, int(math.ceil((w + c) / P - 1e-9)))
    for k in range(len(lay.bx) - nk + 1):
        xl = lay.bx[k] + c
        xr = lay.bx[k] + nk * P
        cx = (xl + xr) / 2
        x0, x1 = cx - w / 2, cx + w / 2
        ys = []
        for ya, yb in Y.aisles:
            ys.append((yb, +1))       # 차로 위 가장자리에서 위로
            ys.append((ya, -1))       # 아래 가장자리에서 아래로
        for y, d in ys:
            poly = box(x0, y, x1, y + L) if d > 0 else box(x0, y - L, x1, y)
            out.append(dict(poly=poly, mouths=[(cx, y - d * 1.0)], dir=(0, d), mouth_mid=(cx, y)))
        # 경계에 붙이고 입구는 칸 안쪽(연결 차로로 이어짐)
        g = _good_intervals(region, [box(x0 + EPS, fr.bounds[1] - 1, x1 - EPS, fr.bounds[3] + 1)], 1,
                            fr.bounds[1] - 1, fr.bounds[3] + 1)[0]
        for g0, g1 in g:
            if g1 - g0 < L + 3:
                continue
            out.append(dict(poly=box(x0, g1 - L, x1, g1), mouths=[(cx, g1 - L - 1.0)], dir=(0, -1),
                            mouth_mid=(cx, g1 - L)))
            out.append(dict(poly=box(x0, g0, x1, g0 + L), mouths=[(cx, g0 + L + 1.0)], dir=(0, 1),
                            mouth_mid=(cx, g0 + L)))
    # 가로 경사로: 줄 뒤쪽 벽에 붙임
    minx, maxx = fr.bounds[0] - 1, fr.bounds[2] + 1
    for r in Y.rows:
        if r["front"] > 0:
            y0, y1 = r["ya"], r["ya"] + w
        else:
            y0, y1 = r["yb"] - w, r["yb"]
        g = _good_intervals(region, [box(minx, y0 + EPS, maxx, y1 - EPS)], 0, minx, maxx)[0]
        ym = (y0 + y1) / 2
        for g0, g1 in g:
            if g1 - g0 < L + 3:
                continue
            xs = [g0] + [b + c for b in lay.bx if g0 <= b + c <= g1 - L]
            for xa in xs[:40]:
                out.append(dict(poly=box(xa, y0, xa + L, y1), mouths=[(xa + L + 1.0, ym)], dir=(1, 0),
                                mouth_mid=(xa + L, ym)))
            xs = [g1] + [b for b in lay.bx if g0 + L <= b <= g1]
            for xb in xs[:40]:
                out.append(dict(poly=box(xb - L, y0, xb, y1), mouths=[(xb - L - 1.0, ym)], dir=(-1, 0),
                                mouth_mid=(xb - L, ym)))
    ok = []
    for cnd in out:
        if region.contains(cnd["poly"]):
            cnd = _with_apron(cnd, w, L, rules, region.difference(cnd["poly"]))
            if cnd is not None:
                ok.append(cnd)
    return ok


def drawn_ramp(frame, rules):
    """그린 경사로: 각 변 가운데에서 바깥 1m 점을 입구 후보로."""
    if not frame.ramps:
        return None
    mouths, mids, aprons = [], [], []
    for rp in frame.ramps:
        cs = list(rp.exterior.coords)
        rr = rp.minimum_rotated_rectangle
        rc = list(rr.exterior.coords)
        short = min(math.hypot(rc[1][0] - rc[0][0], rc[1][1] - rc[0][1]),
                    math.hypot(rc[2][0] - rc[1][0], rc[2][1] - rc[1][1]))
        for (x1, y1), (x2, y2) in zip(cs, cs[1:]):
            L = math.hypot(x2 - x1, y2 - y1)
            # 입구는 경사로 짧은 변(폭 쪽)만. 긴 옆변은 벽.
            if L < 2.5 or L > short * 1.25 + 0.01:
                continue
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            nx, ny = (y2 - y1) / L, -(x2 - x1) / L
            for s in (1, -1):
                px, py = mx + s * nx * 1.0, my + s * ny * 1.0
                if not rp.contains(Point(px, py)) and frame.region.contains(Point(px, py)):
                    ap = make_apron((mx, my), (s * nx, s * ny), L, rules.aisle_90, frame.region)
                    if ap is not None:
                        mouths.append((px, py))
                        mids.append((mx, my))
                        aprons.append(ap)
    return dict(poly=unary_union(frame.ramps), mouths=mouths, mids=mids, aprons=aprons, drawn=True,
                mouth_mid=mids[0] if mids else None)


# ------------------------------------------------------------------ 탐색
EFFORT = {
    "fast": dict(n_ang=1, ystep=1.0, xstep=1.0, top=6, final=3, ramp_top=10),
    "normal": dict(n_ang=2, ystep=0.5, xstep=0.6, top=14, final=5, ramp_top=18),
    "thorough": dict(n_ang=3, ystep=0.25, xstep=0.3, top=30, final=8, ramp_top=30),
    # 경사로를 끌어 옮길 때: 격자 방향·주차 각도를 고정하고 빠르게
    "live": dict(n_ang=1, ystep=1.0, xstep=1.5, top=4, final=1, ramp_top=6),
}


def bay_pitches(rules):
    """1단계 칸 간격: 기둥이 없으므로 연결 차로 너비 하나(연결 차로를 놓을 수 있는 자리의 간격)."""
    return [round(rules.link_aisle, 3)]


def phase1_rules(rules):
    """1단계는 기둥 없이 계산한다."""
    import dataclasses
    return dataclasses.replace(rules, column_w=0.0)


def ext_ok(rules, count, n_ext, floors):
    if count * floors < rules.ext_threshold:
        return True
    return count > 0 and n_ext >= rules.ext_ratio * count - 1e-9


def _key(lay, rules, floors):
    feas = ext_ok(rules, lay.count, lay.n_ext, floors)
    return (1 if feas else 0, lay.count, -len(lay.S))


# ---- 병렬 작업 단위 (프로세스마다 Frame 을 다시 만든다: 가볍고 피클이 쉬움)
_FR_CACHE = {}


def _frame_for(ctx, th):
    key = (ctx[6] if len(ctx) > 6 else id(ctx), th)     # id() 는 재사용될 수 있어 고유 표식을 씀
    if key not in _FR_CACHE:
        site, cores, ramps, obstacles, rules = ctx[:5]
        if len(_FR_CACHE) > 16:
            _FR_CACHE.clear()
        fr = Frame(site, cores, ramps if rules.ramp_mode == "drawn" else [], obstacles, th, rules)
        fixed = drawn_ramp(fr, rules) if rules.ramp_mode == "drawn" else None
        gates = ctx[7] if len(ctx) > 7 else None
        if gates:
            # 구역 나누기: 이웃 구역과 같이 쓰는 차로(대지 밖 띠)를 '앞마당'처럼 붙여 연결 판정
            gs = [affinity.rotate(g, -th, origin=fr.origin) for g in gates]
            if fixed is None:
                fixed = dict(poly=Polygon(), mouths=[], mids=[], aprons=[], drawn=True, mouth_mid=None)
                fixed["mids"] = []
            mids = list(fixed.get("mids") or [])
            while len(mids) < len(fixed["aprons"]):
                c = fixed["aprons"][len(mids)].centroid
                mids.append((c.x, c.y))
            for g in gs:
                c = g.centroid
                fixed["aprons"].append(g)
                mids.append((c.x, c.y))
            fixed["mids"] = mids
            fixed["gates"] = len(gs)
        _FR_CACHE[key] = (fr, fixed)
    return _FR_CACHE[key]


_DP_CACHE = {}


def _ylayout(ctx, th, fr, fixed, rules, ang, mask, y0):
    """mask 'dp' 면 동적계획법으로 쌓은 줄·차로, 아니면 같은 모듈 반복."""
    if mask == "dp":
        key = (ctx[6] if len(ctx) > 6 else id(ctx), th, ang)
        if key not in _DP_CACHE:
            from . import bands
            if len(_DP_CACHE) > 16:
                _DP_CACHE.clear()
            _DP_CACHE[key] = bands.dp_layout(fr, rules, wall_region(fr, rules, fr.region), fixed)
        return _DP_CACHE[key]
    return YLayout(fr, rules, ang, mask, y0)


def _job_coarse(ctx, th, ang, mask, ystep, xstep, keep):
    rules = ctx[4]
    floors = max(1, rules.floors)
    fr, fixed = _frame_for(ctx, th)
    T = row_depth(rules, False, ang) * 2 + rules.aisle_for(ang)
    out = []
    for y0 in (np.arange(0, T, ystep) if mask != "dp" else [0.0]):
        Y = _ylayout(ctx, th, fr, fixed, rules, ang, mask, float(y0))
        if not Y.rows:
            continue
        bands = Y.solve_bands(wall_region(fr, rules, fr.region))
        for P in bay_pitches(rules):
            for x0 in np.arange(0, P, xstep):
                lay = Layout(fr, rules, Y, P, float(x0), bands=bands, ramp=fixed)
                lay.solve()
                out.append((_key(lay, rules, floors), th, ang, mask, float(y0), P, float(x0)))
    out.sort(key=lambda t: t[0], reverse=True)
    return out[:keep], len(out)


def _job_refine(ctx, item, ystep, xstep):
    rules = ctx[4]
    floors = max(1, rules.floors)
    key, th, ang, mask, y0, P, x0 = item[:7]
    fr, fixed = _frame_for(ctx, th)
    best = item
    masks = list(MASKS) if rules.ext_ratio > 0 else ["none"]
    ys = [y0 + d for d in np.arange(-ystep, ystep + 1e-9, ystep / 4)]
    xs = [x0 + d for d in np.arange(-xstep, xstep + 1e-9, xstep / 3)]
    if mask == "dp":
        masks, ys = ["dp"], [y0]
    elif fr.region.area > 20000 and len(masks) > 3:
        # 큰 대지: 확장형 줄 바꿔 보기는 지금 것 + 대표 2개만
        masks = list(dict.fromkeys([mask, "2a", "all"]))
    if rules.effort == "live":
        masks = [mask]
        ys = [y0 + d for d in np.arange(-ystep / 2, ystep / 2 + 1e-9, ystep / 4)]
        xs = [x0 + d for d in np.arange(-xstep / 2, xstep / 2 + 1e-9, xstep / 4)]
    for m in masks:
        for yy in ys:
            Y = _ylayout(ctx, th, fr, fixed, rules, ang, m, float(yy))
            if not Y.rows:
                continue
            bands = Y.solve_bands(wall_region(fr, rules, fr.region))
            for xx in xs:
                lay = Layout(fr, rules, Y, P, float(xx), bands=bands, ramp=fixed)
                lay.solve()
                k = _key(lay, rules, floors)
                if k > best[0]:
                    best = (k, th, ang, m, float(yy), P, float(xx)) + tuple(item[7:])
    return best


MASK_FRAC = {"none": 0, "3a": 1 / 3, "3b": 1 / 3, "3c": 1 / 3, "2a": 0.5, "2b": 0.5, "all": 1}


def _final_one(ctx, fr, fixed, ang, mask, y0, P, x0, ramp_top):
    rules, full = ctx[4], ctx[5]
    floors = max(1, rules.floors)
    Y = _ylayout(ctx, fr.theta, fr, fixed, rules, ang, mask, y0)
    lay = Layout(fr, rules, Y, P, x0, ramp=fixed)
    lay.solve()
    if rules.ramp_mode == "auto":
        lay = place_ramp(lay, ramp_top, floors)
    # 1단계 마무리: 동선 감점을 넣어 연결 차로(회차로)를 다시 고름
    lay.count_only = lay.count
    lay.add_pair_connectors()
    S = circ.optimize(lay, rules, floors, None, max_iter=14)
    lay.apply(S)
    # 2단계: 기둥 배열 → 실제 도형
    return realize(lay, floors, full)


def _job_est(ctx, item):
    """기둥까지 넣었을 때 대수를 빠르게 어림(최종 후보 고르기용): 기둥 줄 DP + 채우기 개수만."""
    rules, full = ctx[4], ctx[5]
    key, th, ang, mask, y0, P, x0 = item[:7]
    fr, fixed = _frame_for(ctx, th)
    Y = _ylayout(ctx, th, fr, fixed, rules, ang, mask, y0)
    lay = Layout(fr, rules, Y, P, x0, ramp=fixed)
    lay.solve()
    if ang != 90 or lay.count < 0:
        return (key[0], lay.count), item
    rows_U = columns.usable_rows(lay, lay.S, lay.roots, lay.main)
    conn_x = [(pc["xl"], pc["xr"]) for k in lay.S for pc in lay.conn[k]]
    lines, free, _ = columns.solve(lay, rows_U, full, conn_x)
    placed, _ = columns.pack(rows_U, full, Y, lines)
    n_ext = sum(1 for t in placed if t[3] == "E")
    ok = ext_ok(full, len(placed) + lay.wall_est, n_ext, max(1, rules.floors))
    return (1 if ok else 0, len(placed) + lay.wall_est), item


def _job_final(ctx, item, ramp_top):
    res = _job_final0(ctx, item, ramp_top)
    res["stats"]["walls"] = list(getattr(ctx[4], "wall_rows", ()) or ())
    return res


def _job_final0(ctx, item, ramp_top):
    key, th, ang, mask, y0, P, x0 = item[:7]
    fr, fixed = _frame_for(ctx, th)
    res = _final_one(ctx, fr, fixed, ang, mask, y0, P, x0, ramp_top)
    # 기둥 때문에 확장형 줄이 더 많이 줄어 비율(30%)이 모자라면 확장형 줄을 늘려 다시
    if not _ext_ok(res):
        for m in sorted(MASKS, key=lambda k: MASK_FRAC[k]):
            if mask == "dp" or MASK_FRAC[m] <= MASK_FRAC[mask]:
                continue
            r2 = _final_one(ctx, fr, fixed, ang, m, y0, P, x0, ramp_top)
            if _ext_ok(r2):
                return r2
    return res


def _ext_ok(res):
    return all(c["ok"] for c in res["stats"]["checks"] if c["item"] == "확장형 비율")


def _executor(workers):
    import concurrent.futures as cf
    import multiprocessing as mp
    if workers <= 1:
        return None
    try:
        return cf.ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn"))
    except Exception:
        return None


def _run(ex, fn, argsets, stop=None, on_done=None):
    if ex is None:
        out = []
        for a in argsets:
            if stop and stop():
                break
            out.append(fn(*a))
            if on_done:
                on_done(len(out), len(argsets))
        return out
    futs = [ex.submit(fn, *a) for a in argsets]
    out = [None] * len(futs)
    import concurrent.futures as cf
    idx = {f: i for i, f in enumerate(futs)}
    done = 0
    for f in cf.as_completed(futs):
        out[idx[f]] = f.result()
        done += 1
        if on_done:
            on_done(done, len(futs))
        if stop and stop():
            for g in futs:
                g.cancel()
            break
    return [o for o in out if o is not None]


def search(site, cores, ramps=(), obstacles=(), rules: Rules = None, log=print, stop=None, workers=None,
           progress=None, executor=None, gates=None):
    import os
    rules = rules or Rules()
    eff = dict(EFFORT.get(rules.effort, EFFORT["normal"]))
    t0 = time.time()
    # 큰 대지: 1단계 넓게 훑기는 성기게(다듬기 단계가 근처를 촘촘히 다시 봄). 1만㎡ 기준 √(면적) 비율, 최대 3배.
    big = max(1.0, min(3.0, math.sqrt(site.area / 10000.0)))
    if rules.effort != "live" and big > 1.0:
        eff["ystep"] = eff["ystep"] * big
        eff["xstep"] = eff["xstep"] * big
    eff["big"] = big
    thetas = list(rules.grid_angles) or grid_angles(site, eff["n_ang"])
    pitches = bay_pitches(rules)
    log(f"격자 방향 {len(thetas)}개 · 주차 각도 {list(rules.park_angles)} · 1단계는 기둥 없이 주차·동선부터")
    full = rules
    rules = phase1_rules(full)
    import uuid
    import dataclasses
    # 둘레(차로 끝 벽) 한 줄 조합: 없음 / 한쪽 / 다른 쪽 / 양쪽 — 안쪽 줄 방향과 둘레 줄을 섞은 배치까지 함께 찾음
    if rules.wall_rows:
        variants = [tuple(rules.wall_rows)]
    elif rules.wall_search and rules.effort != "live":
        variants = [(), ("lo",), ("hi",), ("lo", "hi")]
    else:
        variants = [()]
    ctxs = []
    for wv in variants:
        rv, fv = dataclasses.replace(rules, wall_rows=wv), dataclasses.replace(full, wall_rows=wv)
        ctxs.append((site, list(cores), list(ramps), list(obstacles), rv, fv, uuid.uuid4().hex, list(gates or [])))
    ctx = ctxs[0]
    workers = workers if workers is not None else max(1, min(8, (os.cpu_count() or 2)))
    own = executor is None
    ex = _executor(workers) if own else executor
    prog = progress or (lambda stage, i, n: None)
    try:
        # 1단계: 넓게
        base_masks = ["none", "all", "2a", "3a"] if rules.ext_ratio > 0 else ["none"]
        if rules.effort == "live" and rules.ext_ratio > 0:
            base_masks = ["2a", "3a", "all"]
        def coarse(cis, ths):
            jobs, jci = [], []
            for ci in cis:
                for th in ths:
                    for ang in rules.park_angles:
                        for m in base_masks + (["dp"] if ang == 90 and rules.band_dp else []):
                            jobs.append((ctxs[ci], th, ang, m, eff["ystep"], eff["xstep"], 40))
                            jci.append(ci)
            res = _run(ex, _job_coarse, jobs, stop, lambda i, n: prog(1, i, n))
            out, cnt = [], 0
            for (items, n), ci in zip(res, jci):
                out += [it + (ci,) for it in items]
                cnt += n
            return out, cnt

        # 먼저 둘레 줄 없이 모든 격자 방향 → 좋은 방향 2개에만 둘레 줄 조합(없음/한쪽/양쪽)
        pool, n_all = coarse([0], thetas)
        if len(ctxs) > 1 and pool:
            best_th = {}
            for it in pool:
                best_th[it[1]] = max(best_th.get(it[1], it[0]), it[0])
            th_w = [th for th, _ in sorted(best_th.items(), key=lambda t: t[1], reverse=True)[:2]]
            more, n2 = coarse(list(range(1, len(ctxs))), th_w)
            pool += more
            n_all += n2
        pool.sort(key=lambda t: t[0], reverse=True)
        seen, top = set(), []
        quota = {}
        per_v = max(2, eff["top"] // max(1, len(ctxs)))
        for item in pool:
            sig = (item[7], item[1], item[2], item[3], item[5], round(item[4] / 2), round(item[6] / 2))
            if sig in seen:
                continue
            seen.add(sig)
            qk = (item[7], int(round((item[1] % 180) / 15)) % 12)
            if len(top) >= eff["top"] and (quota.get(qk, 0) >= 1 or item[0][1] < 0.9 * pool[0][0][1]):
                continue
            if quota.get(qk, 0) >= per_v and len(top) >= eff["top"] // 2:
                continue
            top.append(item)
            quota[qk] = quota.get(qk, 0) + 1
        if not top:
            raise RuntimeError("주차 구획을 하나도 놓을 수 없습니다. 대지·코어 레이어와 단위를 확인하세요.")
        log(f"1단계(기둥 없이 주차): {n_all}안을 살펴 상위 {len(top)}안 · 최고 {top[0][0][1]}대/층 · {time.time() - t0:.1f}초")

        # 2단계: 근처를 촘촘히 + 확장형 줄 바꿔 보기
        refined = _run(ex, _job_refine, [(ctxs[it[7]], it, eff["ystep"], eff["xstep"]) for it in top], stop,
                       lambda i, n: prog(2, i, n))
        refined.sort(key=lambda t: t[0], reverse=True)
        log(f"1단계 다듬기: 촘촘히 · 최고 {refined[0][0][1]}대/층 · {time.time() - t0:.1f}초")

        # 기둥까지 넣은 대수로 다시 줄 세우기(1단계 대수가 많아도 기둥에서 많이 잃는 안을 거름)
        if rules.effort != "live":
            seen, cand = set(), []
            for it in refined:
                sig = (it[7], it[1], it[2], it[3], round(it[4], 1), it[5], round(it[6], 1))
                if sig not in seen:
                    seen.add(sig)
                    cand.append(it)
            est = _run(ex, _job_est, [(ctxs[it[7]], it) for it in cand], stop)
            est.sort(key=lambda t: t[0], reverse=True)
            refined = [it for _, it in est]
            if est:
                log(f"기둥까지 넣은 어림: 최고 {est[0][0][1]}면/층")
        # 동선 평가로 연결 차로 다시 고르기 → 2단계 기둥 배열 → 실제 도형(경형·장애인·전기차)
        log("동선 평가(주행거리·막다른 차로·교차·순환)로 연결 차로를 고르고, 기둥 줄을 배열합니다")
        seen, picks = set(), []
        n_pick = eff["final"] + (3 if rules.circ_weight > 0 and rules.effort != "live" else 1)
        for it in refined:
            sig = (it[7], it[1], it[2], it[3], round(it[4], 1), it[5], round(it[6], 1))
            if sig not in seen:
                seen.add(sig)
                picks.append(it)
            if len(picks) >= n_pick:
                break
        # 둘레 조합마다 가장 좋은 것 하나는 끝까지 계산(서로 비교·장점 섞기 확인용)
        ocls = (lambda it: (it[7], int(round((it[1] % 180) / 15)) % 12)) if eff["big"] <= 1.5 else (lambda it: (it[7], it[1]))
        have = set(ocls(p) for p in picks)
        best_n = max((it[0][1] for it in refined), default=0)
        for it in refined:
            if ocls(it) not in have and rules.effort != "live" and it[0][1] >= 0.9 * best_n:
                have.add(ocls(it))
                picks.append(it)
        finals = _run(ex, _job_final, [(ctxs[it[7]], it, eff["ramp_top"]) for it in picks], stop,
                      lambda i, n: prog(3, i, n))
    finally:
        if ex is not None and own:
            ex.shutdown(wait=False, cancel_futures=True)
    finals.sort(key=lambda r: r["rank"], reverse=True)
    for i, r in enumerate(finals):
        st = r["stats"]
        cs = st["circulation"]
        log(f"  안 {i + 1}: {st['park_angle']}° 주차 · B1 {st['per_floor'][0]['legal']}대 · 법정 합계 {st['total_legal']}대 · "
            f"기둥 없이 {st['phase']['no_columns']} → 기둥 넣고 {st['phase']['with_columns']}면 · "
            f"평균 주행 {cs['avg_drive']}m · 막다른 차로 {cs['dead_ends']} · 순환 {cs['loops']}"
            + ("" if r["rank"][0] else " (법규 미충족)"))
    log(f"끝: {time.time() - t0:.1f}초")
    return finals


def place_ramp(lay: Layout, topn, floors):
    rules = lay.rules
    two = lay.count * floors >= 50
    w, L = ramp_size(rules, two)
    cands = ramp_candidates(lay, w, L)
    if not cands:
        return lay
    # 가벼운 손실 추정으로 거름: 지금 살아 있는 구획 중 경사로에 덮이는 수
    alive = lay.alive
    xs0, xs1 = lay.s_x, lay.s_x + lay.s_e
    scored = []
    for cd in cands:
        b = cd["poly"].bounds
        hit = alive & (xs0 < b[2]) & (xs1 > b[0]) & (lay.s_ya < b[3]) & (lay.s_yb > b[1])
        scored.append((int(hit.sum()), cd))
    scored.sort(key=lambda t: t[0])
    best = None
    for _, cd in scored[:topn]:
        reg = lay.frame.region.difference(cd["poly"])
        lay2 = Layout(lay.frame, rules, lay.Y, lay.P, lay.x0, region=reg, ramp=cd)
        lay2.solve()
        lay2.ramp_w, lay2.ramp_L, lay2.ramp_two = w, L, two
        k = _key(lay2, rules, floors)
        if best is None or k > best[0]:
            best = (k, lay2)
    return best[1] if best else lay


# ------------------------------------------------------------------ 실제 도형으로
def _stall_poly(x, y_front, side, W, L, ang, lean):
    """앞선(차로 쪽) y_front. side=+1 이면 줄이 앞선 아래(차로가 위). x 는 구획이 차지하는 x 범위의 시작."""
    if ang == 90:
        return box(x, y_front - L, x + W, y_front) if side > 0 else box(x, y_front, x + W, y_front + L)
    a = math.radians(ang)
    ux, uy = lean * math.cos(a), -side * math.sin(a)          # 앞에서 안쪽으로 (차가 들어가는 방향)
    vx, vy = math.sin(a), side * lean * math.cos(a)           # 너비 방향
    pts = [(0, 0), (W * vx, W * vy), (W * vx + L * ux, W * vy + L * uy), (L * ux, L * uy)]
    mx = min(q[0] for q in pts)
    yref = max(q[1] for q in pts) if side > 0 else min(q[1] for q in pts)
    return Polygon([(q[0] - mx + x, q[1] - yref + y_front) for q in pts])


def _into(side, ang, lean):
    """구획에 차가 들어가는 방향(프레임 좌표)."""
    if ang == 90:
        return (0.0, -1.0) if side > 0 else (0.0, 1.0)
    a = math.radians(ang)
    return (lean * math.cos(a), -side * math.sin(a))


def _front(r):
    return r["yb"] if r["front"] > 0 else r["ya"]


def _mk(kind, poly, ridx, seg, x, W, L, side, ang=90, lean=1, piece=None):
    return dict(kind=kind, poly=poly, row=int(ridx), bay=int(seg), x=float(x), W=W, L=L, lean=lean,
                into=_into(side, ang, lean), ev=False, piece=piece)


def _place_90(lay, full, drive_u):
    """2단계(직각 주차): 기둥 줄 DP → 구획 채우기 → 경형 채우기 → 기둥."""
    fr, Y = lay.frame, lay.Y
    region = lay.region
    rows = Y.rows
    rows_U = columns.usable_rows(lay, lay.S, lay.roots, lay.main)
    conn_x = [(pc["xl"], pc["xr"]) for k in lay.S for pc in lay.conn[k]]
    lines, free, _ = columns.solve(lay, rows_U, full, conn_x)
    placed, rest = columns.pack(rows_U, full, Y, lines)
    stalls = []
    for ridx, seg, x, kind, W, lo, hi in placed:
        r = rows[ridx]
        L = full.ext_l if kind == "E" else full.stall_l
        stalls.append(_mk(kind, _stall_poly(x, _front(r), r["front"], W, L, 90, 1), ridx, seg, x, W, L, r["front"],
                          piece=(ridx, seg, round(lo, 4), round(hi, 4))))
    if stalls:
        ok = shapely.contains(region, np.array([s_["poly"] for s_ in stalls], dtype=object))
        stalls = [s_ for s_, o in zip(stalls, ok) if o]
    after_cols = len(stalls)

    # 경형: 일반이 안 들어가는 얕은 자리 + 기둥 사이 남는 조각
    if full.compact_fill:
        minx, miny, maxx, maxy = fr.bounds
        lo_, hi_ = minx - 1, maxx + 1
        bands = []
        for r in rows:
            yf = _front(r)
            bands.append(box(lo_, yf - full.compact_l + EPS, hi_, yf - EPS) if r["front"] > 0
                         else box(lo_, yf + EPS, hi_, yf + full.compact_l - EPS))
        good = _good_intervals(region, bands, 0, lo_, hi_)
        Uc = columns.usable_rows(lay, lay.S, lay.roots, lay.main, row_good=good)
        extra = [columns._sub(uc, u) for uc, u in zip(Uc, rows_U)]
        cw_rules = _CompactRules(full)
        cplaced, _ = columns.pack(extra, cw_rules, Y, lines)
        cands = [(ridx, x) for ridx, seg, x, kind, W, lo, hi in cplaced]
        cands += [(ridx, x) for ridx, seg, x, left, lo, hi in rest if left >= full.compact_w - TOL]
        cpolys = [_stall_poly(x, _front(rows[ridx]), rows[ridx]["front"], full.compact_w, full.compact_l, 90, 1)
                  for ridx, x in cands]
        if cpolys:
            arr = np.array(cpolys, dtype=object)
            ok = shapely.contains(region, arr)
            if lay.ramp is not None:
                blk = unary_union([lay.ramp["poly"]] + list(lay.aprons)).buffer(-1e-3)
                ok &= ~shapely.intersects(arr, blk)
            for (ridx, x), pg, o in zip(cands, cpolys, ok):
                if o:
                    stalls.append(_mk("C", pg, ridx, -1, x, full.compact_w, full.compact_l, rows[ridx]["front"]))

    # 기둥: 기둥 줄 × 주차줄의 차로 쪽 끝
    c, d = full.column_w, full.column_d
    cols, col_ys = [], set()
    if c > 0 and lines:
        cand, ys = [], []
        for r in rows:
            if r["front"] > 0:
                yy = (r["yb"] - full.column_inset - d, r["yb"] - full.column_inset)
            else:
                yy = (r["ya"] + full.column_inset, r["ya"] + full.column_inset + d)
            for p in lines:
                cand.append(box(p, yy[0], p + c, yy[1]))
                ys.append(round((yy[0] + yy[1]) / 2, 4))
        arr = np.array(cand, dtype=object)
        ok = shapely.contains(fr.usable, arr)
        blockers = [drive_u] + list(fr.cores) + ([lay.ramp["poly"]] if lay.ramp is not None else [])
        bl = unary_union(blockers).buffer(-1e-3)
        ok &= ~shapely.intersects(arr, bl)
        if stalls:
            sp = unary_union([s_["poly"] for s_ in stalls]).buffer(-1e-3)
            ok &= ~shapely.intersects(arr, sp)
        cols = [g for g, o in zip(cand, ok) if o]
        col_ys = sorted(set(y for y, o in zip(ys, ok) if o))
    spans = [round(b - a, 2) for a, b in zip(lines, lines[1:])]
    grid = dict(xs=[p + c / 2 for p in lines], ys=list(col_ys), spans=spans)
    return stalls, cols, grid, dict(free=free, after=after_cols)


class _CompactRules:
    """columns.pack 에 경형 너비를 넘기기 위한 얇은 감싸개."""

    def __init__(self, full):
        self.column_w = full.column_w
        self.ext_w = full.compact_w
        self.stall_w = full.compact_w


def _place_angled(lay, full, drive_u):
    """비스듬한 주차: 1단계 칸 그대로, 기둥은 줄 뒤쪽 삼각 틈에."""
    fr, Y = lay.frame, lay.Y
    region = lay.region
    ang = Y.ang
    rows = Y.rows
    stalls = []
    for i in np.nonzero(lay.alive)[0]:
        r = rows[lay.s_row[i]]
        W, L = stall_dims(full, r["ext"])
        lean = 1 if r["aisle"] % 2 == 0 else -1
        poly = _stall_poly(lay.s_x[i], _front(r), r["front"], W, L, ang, lean)
        stalls.append(_mk("E" if r["ext"] else "N", poly, lay.s_row[i], lay.s_bay[i], lay.s_x[i], W, L,
                          r["front"], ang, lean))
    if stalls:
        ok = shapely.contains(region, np.array([s_["poly"] for s_ in stalls], dtype=object))
        stalls = [s_ for s_, o in zip(stalls, ok) if o]
    free = len(stalls)
    c, d = full.column_w, full.column_d
    cols = []
    xs_lines = []
    if c > 0:
        pitch = full.stall_w / math.sin(math.radians(ang))
        Pa = max(pitch, math.floor((full.span_max - c) / pitch) * pitch + c)
        minx, miny, maxx, maxy = fr.bounds
        xs_lines = list(np.arange(minx - Pa + lay.x0 % Pa, maxx + Pa, Pa))
        cand = []
        for r in rows:
            yy = (r["ya"], r["ya"] + d) if r["front"] > 0 else (r["yb"] - d, r["yb"])
            for bx in xs_lines:
                cand.append(box(bx, yy[0], bx + c, yy[1]))
        arr = np.array(cand, dtype=object)
        ok = shapely.contains(fr.usable, arr)
        blockers = [drive_u] + list(fr.cores) + ([lay.ramp["poly"]] if lay.ramp is not None else [])
        bl = unary_union(blockers).buffer(-1e-3)
        ok &= ~shapely.intersects(arr, bl)
        cols = [g for g, o in zip(cand, ok) if o]
        if stalls and cols:
            sp = unary_union([s_["poly"] for s_ in stalls]).buffer(-1e-3)
            shapely.prepare(sp)
            moved = []
            for g in cols:
                best = None
                for dd in sorted(np.arange(-pitch / 2, pitch / 2 + 1e-9, 0.05), key=abs):
                    gg = affinity.translate(g, dd, 0)
                    if not sp.intersects(gg) and fr.usable.contains(gg) and not bl.intersects(gg):
                        best = gg
                        break
                moved.append(best if best is not None else g)
            cols = []
            for g in moved:
                if not any(g.distance(h) < 0.4 for h in cols):
                    cols.append(g)
            cu = unary_union(cols).buffer(-1e-3)
            hit = shapely.intersects(np.array([s_["poly"] for s_ in stalls], dtype=object), cu)
            stalls = [s_ for s_, h in zip(stalls, hit) if not h]
    grid = dict(xs=[], ys=[], spans=[])
    return stalls, cols, grid, dict(free=free, after=len(stalls))


def _counts(sts):
    n = {"N": 0, "E": 0, "C": 0, "A": 0, "V": 0, "VN": 0, "VE": 0}
    for s_ in sts:
        n[s_["kind"]] += 1
        if s_.get("ev"):
            n["V"] += 1
            n["V" + s_["kind"]] += 1
    return n


def _accessible(stalls, lay, full, floors, rows):
    """장애인 구획: 코어에 가까운 조각부터, 구획을 3.3m 구획으로 바꿈 (B1 에만)."""
    fr = lay.frame
    groups = {}
    for idx, s_ in enumerate(stalls):
        if s_["kind"] in ("N", "E") and s_.get("piece"):
            groups.setdefault(s_["piece"], []).append(idx)
    core_u = unary_union(fr.cores) if fr.cores else None
    glist = []
    for key, idxs in groups.items():
        ridx, seg, lo, hi = key
        ext = rows[ridx]["ext"]
        W = full.ext_w if ext else full.stall_w
        x0 = min(stalls[i]["x"] for i in idxs)
        ext_len = hi - x0
        k_acc = int(math.floor(ext_len / full.acc_w + 1e-9))
        if k_acc < 1:
            continue
        g = unary_union([stalls[i]["poly"] for i in idxs])
        dist = g.distance(core_u) if core_u is not None else 0.0
        glist.append((ext, dist, ridx, idxs, x0, ext_len, W, key))
    glist.sort(key=lambda t: (t[0], t[1]))
    base = _counts(stalls)
    cur = list(stalls)
    n_acc = 0
    for ext, dist, ridx, idxs, x0, ext_len, W, key in glist:
        tot = legal_count(_counts(cur), full) + (floors - 1) * legal_count(base, full)
        need = math.ceil(full.acc_ratio * tot - 1e-9) if tot >= full.acc_min_total else 0
        if n_acc >= need:
            break
        r = rows[ridx]
        k_acc = int(math.floor(ext_len / full.acc_w + 1e-9))
        take = min(k_acc, need - n_acc)
        rm = set(id(stalls[i]) for i in idxs)
        cur = [s_ for s_ in cur if id(s_) not in rm]
        for j in range(take):
            x = x0 + j * full.acc_w
            cur.append(_mk("A", _stall_poly(x, _front(r), r["front"], full.acc_w, full.acc_l, 90, 1), ridx, key[1], x,
                           full.acc_w, full.acc_l, r["front"]))
        x_rest = x0 + take * full.acc_w
        L = full.ext_l if ext else full.stall_l
        for j in range(int(math.floor((ext_len - take * full.acc_w) / W + 1e-9))):
            x = x_rest + j * W
            cur.append(_mk("E" if ext else "N", _stall_poly(x, _front(r), r["front"], W, L, 90, 1), ridx, key[1], x,
                           W, L, r["front"]))
        n_acc += take
    return cur


def _mark_ev(sts, full, ramp_pt, need):
    """전기차 전용구획: 경사로 가까운 확장형부터(없으면 일반). 같은 도형, 표시만."""
    out = [dict(s_) for s_ in sts]
    if need <= 0:
        return out
    cand = [s_ for s_ in out if s_["kind"] in ("E", "N")]
    if ramp_pt is not None:
        cand.sort(key=lambda s_: (0 if s_["kind"] == "E" else 1, s_["poly"].centroid.distance(ramp_pt)))
    for s_ in cand[:need]:
        s_["ev"] = True
    return out


def realize(lay: Layout, floors, full: Rules = None):
    """1단계 결과(기둥 없는 배치·동선) → 2단계 기둥 배열 → 실제 도형·법규 확인·동선 지표."""
    full = full or lay.rules
    fr, Y = lay.frame, lay.Y
    region = lay.region
    ang = Y.ang
    rows = Y.rows
    main_nodes = set(i for i in range(len(lay.nodes)) if lay.roots[i] == lay.roots[lay.main]) if lay.main >= 0 else set()

    # 차로 면
    drive = []
    for nid in main_nodes:
        ai, a0, a1 = lay.nodes[nid]
        ya, yb = Y.aisles[ai]
        drive.append(box(a0, ya, a1, yb))
    links = []
    for k in lay.S:
        for pc in lay.conn[k]:
            links.append(box(pc["xl"], pc["y0"], pc["xr"], pc["y1"]))
    links += list(lay.aprons)
    drive_u = unary_union(drive + links).intersection(lay.base_region) if (drive or links) else Polygon()

    if ang == 90:
        stalls, cols, grid, ph = _place_90(lay, full, drive_u)
    else:
        stalls, cols, grid, ph = _place_angled(lay, full, drive_u)

    # 자투리 채우기: 연결 차로 옆·코어·경사로 옆·모서리
    n_main = len(stalls)
    if full.edge_fill:
        lanes = unary_union(drive + [box(pc["xl"], pc["y0"], pc["xr"], pc["y1"]) for k in lay.S for pc in lay.conn[k]])
        lanes = lanes.intersection(lay.base_region) if not lanes.is_empty else lanes
        blk = [s_["poly"] for s_ in stalls] + list(cols) + list(fr.cores) + list(lay.aprons) + [drive_u]
        if lay.ramp is not None:
            blk.append(lay.ramp["poly"])
        cnt = _counts(stalls)
        sizes = {"N": (full.stall_w, full.stall_l), "E": (full.ext_w, full.ext_l), "C": (full.compact_w, full.compact_l)}
        if not full.compact_fill:
            sizes["C"] = (1e9, 1e9)
        for k, poly, into, W, L in fill.fill(lay.base_region, lanes, blk, sizes, len(stalls), cnt["E"], full.ext_ratio,
                                             full.ext_threshold, step=0.1 if ang == 90 else 0.3):
            c = poly.centroid
            stalls.append(dict(kind=k, poly=poly, row=10 ** 6, bay=-1, x=float(c.x), W=W, L=L, lean=1, into=into,
                               ev=False, piece=None))
    ph["fill"] = len(stalls) - n_main

    # 일방통행(비스듬한 주차 + 일방 차로 너비): 경사로에서 들어가 다시 나갈 수 없는 구획은 뺌
    arrows, flow_info = [], None
    if ang != 90:
        from . import oneway
        keep, arrows, flow_info = oneway.check_flow(lay, lay.S, stalls, rows)
        if full.two_entrances:
            stalls = [s_ for s_, k in zip(stalls, keep) if k]
        else:
            arrows = []          # 양방 차로면 화살표 없음

    acc_stalls = _accessible(stalls, lay, full, floors, rows) if (full.acc_ratio > 0 and ang == 90) else list(stalls)

    # 전기차
    ramp_pt = None
    if lay.aprons:
        ramp_pt = lay.aprons[0].centroid
    elif lay.ramp is not None:
        ramp_pt = lay.ramp["poly"].centroid
    n1, nb = _counts(acc_stalls), _counts(stalls)
    ev1 = math.ceil(full.ev_ratio * legal_count(n1, full) - 1e-9) if full.ev_ratio > 0 else 0
    evb = math.ceil(full.ev_ratio * legal_count(nb, full) - 1e-9) if full.ev_ratio > 0 else 0
    acc_stalls = _mark_ev(acc_stalls, full, ramp_pt, ev1)
    stalls = _mark_ev(stalls, full, ramp_pt, evb)

    # 동선 지표 (최종 구획 기준 주행거리)
    def fronts_of(sts):
        out = []
        for s_ in sts:
            if s_["row"] >= len(rows):
                continue
            r = rows[s_["row"]]
            ai = r["aisle"]
            xf = s_["poly"].centroid.x
            for nid in main_nodes:
                a_i, a0, a1 = lay.nodes[nid]
                if a_i == ai and a0 - 0.5 <= xf <= a1 + 0.5:
                    out.append((nid, min(max(xf, a0), a1)))
                    break
        return out
    cm = circ.analyze(lay, lay.S, lay.alive, lay.roots, lay.main, full, fronts=fronts_of(acc_stalls))
    flows = circ.flow_lines(cm)
    dead_marks = [(x["x"], x["y"], x["sgn"]) for x in cm.get("dead", []) if x["bad"]]

    n1 = _counts(acc_stalls)
    nb = _counts(stalls)
    per_floor = [n1] + [nb] * (floors - 1)
    legal = [legal_count(n, full) for n in per_floor]
    phys = [n["N"] + n["E"] + n["C"] + n["A"] for n in per_floor]
    tot_legal = sum(legal)
    tot_phys = sum(phys)
    ext_tot = sum(n["E"] for n in per_floor)
    acc_tot = sum(n["A"] for n in per_floor)
    ev_tot = sum(n["V"] for n in per_floor)
    park_area = fr.usable.area - sum(cc.area for cc in fr.cores)

    checks = []
    if tot_phys >= full.ext_threshold:
        ok = ext_tot >= full.ext_ratio * tot_phys - 1e-9
        checks.append(dict(item="확장형 비율", need=f"{full.ext_ratio * 100:.0f}% 이상 ({math.ceil(full.ext_ratio * tot_phys)}대)",
                           have=f"{ext_tot}대 ({ext_tot / max(tot_phys, 1) * 100:.1f}%)", ok=ok))
    else:
        checks.append(dict(item="확장형 비율", need=f"총 {full.ext_threshold}대 미만 → 해당 없음", have=f"{ext_tot}대", ok=True))
    need_acc = math.ceil(full.acc_ratio * tot_legal - 1e-9) if tot_legal >= full.acc_min_total else 0
    checks.append(dict(item="장애인전용 구획", need=f"{full.acc_ratio * 100:.0f}% → {need_acc}대",
                       have=f"{acc_tot}대 (B1 코어 근처)", ok=acc_tot >= need_acc))
    if full.ev_ratio > 0:
        need_ev = math.ceil(full.ev_ratio * tot_legal - 1e-9)
        checks.append(dict(item="전기차 전용구획", need=f"{full.ev_ratio * 100:.0f}% → {need_ev}대 (층마다 경사로 가까이)",
                           have=f"{ev_tot}대", ok=ev_tot >= need_ev))
    ctot = sum(n["C"] for n in per_floor)
    checks.append(dict(item="경형 구획 인정 한도", need=f"전체의 {full.compact_legal_ratio * 100:.0f}%까지 법정 대수 인정",
                       have=f"경형 {ctot}대 중 {ctot - (tot_phys - tot_legal)}대 인정", ok=True))
    checks.append(dict(item="차로 너비", need=f"{ang}° 주차 {Y.aisle_w:.1f}m 이상 · 연결 차로 {full.link_aisle:.1f}m 이상",
                       have=f"{Y.aisle_w:.1f}m / {min(full.link_aisle, lay.kc * lay.P - lay.c):.1f}m 이상", ok=True))
    if grid["spans"]:
        checks.append(dict(item="기둥 간격", need=f"{full.span_min:.1f}~{full.span_max:.1f}m (기둥 {full.column_w * 1000:.0f}×{full.column_d * 1000:.0f})",
                           have=f"{min(grid['spans']):.1f}~{max(grid['spans']):.1f}m", ok=max(grid["spans"]) <= full.span_max + 1e-6))
    if lay.ramp is not None:
        if lay.ramp.get("drawn"):
            con = lay.count >= 0 and (lay.base_links or any(pc["ramp"] for k in lay.S for pc in lay.conn[k]))
            checks.append(dict(item="경사로", need=f"그린 경사로 · 입구 {lay.n_ap}곳이 차로에 이어짐",
                               have="이어짐" if con else "이어지지 않음 — 경사로 위치·입구를 확인하세요", ok=bool(con)))
        else:
            w_need = full.ramp_two_lane if lay.ramp_two else full.ramp_lane
            checks.append(dict(item="경사로", need=f"직선 {full.ramp_slope * 100:.0f}% 이하 · 폭 {w_need:.1f}m"
                                                     + (" (50대 이상 2차로)" if lay.ramp_two else ""),
                               have=f"길이 {lay.ramp_L:.1f}m (층고 {full.floor_h}m ÷ {full.ramp_slope}) + 폭 {w_need:.1f}m", ok=True))
    if ang != 90 and flow_info is not None:
        if full.two_entrances:
            checks.append(dict(item="일방통행 흐름", need=f"{ang}° 일방 차로 {Y.aisle_w:.1f}m: 차로마다 구획이 기운 쪽으로만 달림 · "
                                                         "경사로에서 들어가 다시 경사로로 나갈 수 있어야 함",
                               have=(f"모두 됨" if not flow_info["dropped"] else
                                     f"거꾸로 들어가야 하는 {flow_info['dropped']}면은 뺌"), ok=True))
        else:
            checks.append(dict(item="일방통행 흐름", need=f"{ang}° 양방 차로 {Y.aisle_w:.1f}m (출입구 하나)",
                               have="양방향이라 해당 없음" + (f" · 일방으로 바꾸면 {flow_info['dropped']}면 못 씀"
                                                            if flow_info["dropped"] else ""), ok=True))
    if fr.ramps and lay.ramp is not None and lay.ramp.get("drawn"):
        from . import oneway
        checks += oneway.ramp_checks(fr.ramps, tot_legal, full)
    csum = circ.summary(cm, full.circ_weight)
    score = tot_legal - full.circ_weight * cm.get("penalty", 0.0) * floors
    # 그린 경사로 모양(너비·반경)은 배치로 바꿀 수 없어 순위에는 넣지 않고 표시만
    rank = (1 if all(ch["ok"] for ch in checks if ch["item"] != "전기차 전용구획" and not ch.get("ramp")) else 0,
            round(score, 3), tot_legal, -len(cols))
    stats = dict(
        grid_angle=round(fr.theta, 2), park_angle=ang, bay=(round(sum(grid["spans"]) / len(grid["spans"]), 2) if grid["spans"] else 0),
        ext_mask=Y.mask, aisle_w=Y.aisle_w, floors=floors,
        per_floor=[dict(floor=f"B{i + 1}", 일반=n["N"] - n["VN"], 확장=n["E"] - n["VE"],
                        경형=n["C"], 장애인=n["A"], 전기차=n["V"], physical=phys[i], legal=legal[i])
                   for i, n in enumerate(per_floor)],
        per_floor_legal=legal[-1] if floors > 1 else legal[0],
        total_legal=tot_legal, total_physical=tot_phys, total_ev=ev_tot,
        floor_area=round(fr.usable.area, 1), park_area=round(park_area, 1),
        area_per_stall=round(fr.usable.area / max(legal[0], 1), 2),
        columns=len(cols), links=len(lay.S), spans=grid["spans"],
        phase=dict(no_columns=ph["free"], with_columns=ph["after"], column_loss=ph["free"] - ph["after"], edge_fill=ph.get("fill", 0)),
        circulation=csum, score=round(score, 1),
        checks=checks,
        need_floors=(math.ceil(full.target_total / max(legal[-1], 1)) if full.target_total else None),
    )
    return dict(frame=fr, layout=lay, rules=full, stalls_b1=acc_stalls, stalls=stalls, columns=cols,
                drive=drive_u, ramp=lay.ramp, stats=stats, rank=rank, grid=grid, flows=flows, dead=dead_marks, arrows=arrows)


def legal_count(n, rules):
    non_c = n["N"] + n["E"] + n["A"]
    c = n["C"]
    cap = int(math.floor(rules.compact_legal_ratio * (non_c + c) + 1e-9))
    return non_c + min(c, cap)
