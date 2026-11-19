"""2단계: 정해진 주차 배치 위에 기둥 줄을 배열.

1단계에서 기둥 없이 구획·차로·연결 차로를 정한 뒤, 차로와 나란한 방향(x)으로 기둥 줄 위치를 고른다.
- 기둥 줄은 평면 전체를 가로지르는 한 줄(격자선). 줄마다 각 주차줄의 차로 쪽 끝에 기둥이 하나씩 선다.
- 이웃한 기둥 줄 사이 간격은 span_min ~ span_max 안에서 자유롭게(0.1m 단위) — 정형(예: 3대×2.6+0.7=8.5m)이
  아니어도 된다. 벽에서 첫 기둥까지도 span_max 이하.
- 연결 차로(회차로) 위에는 기둥을 세우지 않는다.
- 목표: 기둥 때문에 잃는 구획이 가장 적게. 같으면 기둥 줄이 적고 간격이 정형에 가까운 안.
동적 계획법(DP): best[i] = 기둥 줄이 위치 i 에 있을 때 그 왼쪽에 들어가는 구획 수의 최댓값.
"""
import math

import numpy as np

TOL = 1e-6


def _sub(ivs, cuts):
    """구간 목록 ivs 에서 cuts 구간들을 뺌."""
    out = list(ivs)
    for c0, c1 in cuts:
        nxt = []
        for a, b in out:
            if c1 <= a + TOL or c0 >= b - TOL:
                nxt.append((a, b))
                continue
            if c0 > a + TOL:
                nxt.append((a, c0))
            if c1 < b - TOL:
                nxt.append((c1, b))
        out = nxt
    return out


def _inter(A, B):
    out = []
    for a0, a1 in A:
        for b0, b1 in B:
            lo, hi = max(a0, b0), min(a1, b1)
            if hi > lo + TOL:
                out.append((lo, hi))
    return sorted(out)


def usable_rows(lay, S, roots, main, row_good=None):
    """주차줄마다 구획을 둘 수 있는 x 구간(대지 안 · 주 동선 차로에 앞이 닿음 · 연결 차로·앞마당 밖)."""
    Y = lay.Y
    rm = roots[main] if main is not None and 0 <= main < len(roots) else None
    node_iv = {}
    for nid, (ai, a0, a1) in enumerate(lay.nodes):
        if rm is not None and roots[nid] == rm:
            node_iv.setdefault(ai, []).append((a0, a1))
    cuts_conn = []
    for k in S:
        for pc in lay.conn[k]:
            cuts_conn.append((pc["xl"], pc["xr"], pc["y0"], pc["y1"]))
    out = []
    for ridx, r in enumerate(Y.rows):
        base = [(a, b) for a, b in (row_good if row_good is not None else lay.row_good)[ridx]]
        U = _inter(base, sorted(node_iv.get(r["aisle"], [])))
        cuts = [(x0, x1) for x0, x1, y0, y1 in cuts_conn if y0 < r["yb"] - TOL and y1 > r["ya"] + TOL]
        for ap in lay.aprons:
            b = ap.bounds
            if b[1] < r["yb"] - 1e-3 and b[3] > r["ya"] + 1e-3:
                cuts.append((b[0], b[2]))
        out.append(_sub(U, cuts))
    return out


def _count(U, W, a, b):
    """구간 U 안, (a, b) 에 들어가는 너비 W 구획 수. a, b 는 배열."""
    tot = np.zeros(np.shape(a), dtype=np.int64)
    for i0, i1 in U:
        L = np.minimum(b, i1) - np.maximum(a, i0)
        tot += np.floor(np.clip(L, 0, None) / W + 1e-7).astype(np.int64)
    return tot


def solve(lay, rows_U, rules, conn_x):
    """기둥 줄 왼쪽 면 x 목록과 (기둥 없을 때 대수, 기둥 넣은 뒤 대수)."""
    Y = lay.Y
    c = rules.column_w
    Ws = [(rules.ext_w if r["ext"] else rules.stall_w) for r in Y.rows]
    allU = [iv for U in rows_U for iv in U]
    free = sum(int(_count(U, W, np.array([-1e9]), np.array([1e9]))[0]) for U, W in zip(rows_U, Ws))
    if not allU or c <= 0:
        return [], free, free
    X0 = min(a for a, b in allU)
    X1 = max(b for a, b in allU)
    st = rules.span_step
    smin, smax = rules.span_min, rules.span_max
    if X1 - X0 <= smax + TOL:
        return [], free, free
    pos = np.arange(X0 - smax, X1 + st, st)
    N = len(pos)
    spans = np.arange(math.ceil(smin / st), math.floor(smax / st + 1e-9) + 1)       # 칸 수
    # 기둥 놓을 수 없는 위치(연결 차로 위)
    bad = np.zeros(N, bool)
    for xl, xr in conn_x:
        bad |= (pos < xr - TOL) & (pos + c > xl + TOL)
    # 경계 값
    start = np.zeros(N)
    end = np.zeros(N)
    M = np.zeros((len(spans), N))
    for U, W in zip(rows_U, Ws):
        if not U:
            continue
        start += _count(U, W, np.full(N, -1e9), pos)
        end += _count(U, W, pos + c, np.full(N, 1e9))
        for si, s in enumerate(spans):
            M[si] += _count(U, W, pos - s * st + c, pos)
    n_pref = rules.stalls_per_bay[0] if rules.stalls_per_bay else 3
    pref = n_pref * rules.ext_w + c
    irr = 0.002 * np.abs(spans * st - pref)
    COL = 0.01
    NEG = -1e18
    best = np.full(N, NEG)
    prev = np.full(N, -1, dtype=np.int64)
    # 연결 차로 위는 크게 감점(어쩔 수 없을 때만 — 그 자리 기둥은 세우지 않고 간격이 길어짐)
    BAD = 50.0
    okstart = pos - X0 <= smax + TOL
    best[okstart] = start[okstart] - COL - BAD * bad[okstart]
    for i in range(N):
        j = i - spans
        m = j >= 0
        if not m.any():
            continue
        jj = j[m]
        v = best[jj] + M[m, i] - irr[m] - COL - (BAD if bad[i] else 0.0)
        v[best[jj] <= NEG / 2] = NEG
        k = int(np.argmax(v))
        if v[k] > best[i] and v[k] > NEG / 2:
            best[i] = v[k]
            prev[i] = jj[k]
    okend = (X1 - (pos + c) <= smax + TOL) & (best > NEG / 2)
    if not okend.any():
        return [], free, free
    tot = np.where(okend, best + end, NEG)
    i = int(np.argmax(tot))
    lines = []
    while i >= 0:
        lines.append(float(pos[i]))
        i = int(prev[i])
    lines.sort()
    return lines, free, None


def pack(rows_U, rules, Y, lines):
    """기둥 줄 사이(와 벽 사이)에 구획을 왼쪽부터 채움. [(ridx, seg, x, kind, W)] 와 남는 조각."""
    c = rules.column_w
    bounds = [-1e9] + lines + [1e9]
    out, rest = [], []
    for ridx, U in enumerate(rows_U):
        r = Y.rows[ridx]
        W = rules.ext_w if r["ext"] else rules.stall_w
        for seg in range(len(bounds) - 1):
            a = bounds[seg] + (c if seg > 0 else 0.0)
            b = bounds[seg + 1]
            for i0, i1 in U:
                lo, hi = max(a, i0), min(b, i1)
                if hi - lo < TOL:
                    continue
                n = int(math.floor((hi - lo) / W + 1e-7))
                for j in range(n):
                    out.append((ridx, seg, lo + j * W, "E" if r["ext"] else "N", W, lo, hi))
                left = hi - lo - n * W
                if left > TOL:
                    rest.append((ridx, seg, lo + n * W, left, lo, hi))
    return out, rest
