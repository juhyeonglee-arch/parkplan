"""차량 동선 평가.

1단계(기둥 없이 주차 배치)에서 고른 차로·연결 차로를 그래프로 만들고 다음을 잰다.

- 주행거리: 경사로 입구 앞마당에서 각 구획 앞까지 차로 중심선을 따라 가는 최단거리 (평균·최대)
- 막다른 차로: 다른 차로와 이어지지 않고 끝나는 차로 끝. 끝 길이가 dead_end_ok(기본 6m, 구획 하나 깊이)를
  넘으면 회차가 필요한 막다른 차로로 센다. (싱가포르 LTA 지침: 막다른 통로는 끝에서 주차면 하나 길이만큼 더 넓힘)
- 교차: 세 갈래(T)·네 갈래(+) 만나는 곳 수. 많을수록 충돌 지점이 늘어남
- 순환: 차로 그래프의 고리 수(E − V + 1). 0 이면 한 바퀴 돌아 나갈 수 없음(왔던 길로 되돌아 나감)
- 주 동선/부 동선: 경사로에서의 최단 경로 나무에서 각 차로 토막을 지나는 구획 수(부하)로 나눔

감점은 "주차 몇 대만큼"으로 환산해 대수와 바로 비교한다(circ_weight 배). 이 환산값은 연구 근거가 있는
수치가 아니라 이 프로그램의 기본값이며 화면에서 가중치로 조절한다.
"""
import heapq
import math

import numpy as np

# 감점(주차 대수 환산) 기본값
PEN_DIST = 0.10        # 평균 주행거리 1m 당
PEN_DEAD = 1.5         # 회차가 필요한 막다른 차로 1곳
PEN_DEAD_LEN = 0.08    # 막다른 차로의 허용 길이를 넘는 1m 당
PEN_DEAD_STALL = 0.1   # 막다른 차로에 붙은 구획 1면 당(되돌아 나와야 함)
PEN_T = 0.25           # 세 갈래 교차 1곳
PEN_X = 0.5            # 네 갈래 교차 1곳
PEN_NOLOOP = 2.0       # 순환 고리가 없을 때(구획 60면 이상)
MOVES_CAP = 10 ** 9    # 연결 차로 다듬기에서 한 번에 동선을 계산해 볼 수의 상한(지금은 제한 없음)


class Net:
    def __init__(self):
        self.pts = []          # (x, y)
        self.key = {}
        self.adj = []          # [(j, length, kind)] kind: 'a' 차로, 'c' 연결 차로, 'r' 경사로 앞
        self.ramp = []         # 경사로 앞마당 정점

    def v(self, x, y):
        k = (round(float(x) * 50), round(float(y) * 50))     # 2cm 격자
        if k not in self.key:
            self.key[k] = len(self.pts)
            self.pts.append((x, y))
            self.adj.append([])
        return self.key[k]

    def e(self, a, b, kind):
        if a == b:
            return
        if any(j == b for j, _, _ in self.adj[a]):
            return
        (x1, y1), (x2, y2) = self.pts[a], self.pts[b]
        L = math.hypot(x2 - x1, y2 - y1)
        self.adj[a].append((b, L, kind))
        self.adj[b].append((a, L, kind))

    def edges(self):
        for a, lst in enumerate(self.adj):
            for b, L, k in lst:
                if a < b:
                    yield a, b, L, k


def build(lay, S, roots, main):
    """lay(Layout) 와 고른 연결 차로 S 로 동선 그래프."""
    Y = lay.Y
    net = Net()
    if main is None or main < 0:
        return net, {}
    rm = roots[main] if main < len(roots) else main
    main_nodes = [i for i in range(len(lay.nodes)) if roots[i] == rm]
    yc = {nid: (Y.aisles[lay.nodes[nid][0]][0] + Y.aisles[lay.nodes[nid][0]][1]) / 2 for nid in main_nodes}
    jx = {nid: [lay.nodes[nid][1], lay.nodes[nid][2]] for nid in main_nodes}
    ramp_pts = {}
    mids = (lay.ramp or {}).get("mids") or []
    for j, ap in enumerate(lay.aprons):
        if j < len(mids):
            ramp_pts[lay.ramp_node + j] = tuple(mids[j])      # 경사로 입구 변 가운데
        elif lay.ramp is not None and lay.ramp.get("mouth_mid") is not None and j == 0:
            ramp_pts[lay.ramp_node + j] = tuple(lay.ramp["mouth_mid"])
        else:
            c = ap.centroid
            ramp_pts[lay.ramp_node + j] = (c.x, c.y)
    vert = []      # 연결 차로 세로 사슬
    for k in S:
        for pc in lay.conn[k]:
            cs = [n for n in pc["contacts"] if n in yc]
            if not cs and not pc["ramp"]:
                continue
            xc = (pc["xl"] + pc["xr"]) / 2
            chain = []
            for n in cs:
                chain.append((yc[n], ("n", n)))
                jx[n].append(xc)
            for rn in pc["ramp"]:
                if rn in ramp_pts and (rn == lay.ramp_node or roots[rn] == rm):
                    ry = min(max(ramp_pts[rn][1], pc["y0"]), pc["y1"])
                    chain.append((ry, ("r", rn)))
            if len(chain) >= 2 or (chain and chain[0][1][0] == "r"):
                vert.append((xc, sorted(chain)))
    side = []      # 앞마당 → 차로
    for a, b in lay.base_links:
        rn, nid = (a, b) if a >= lay.ramp_node else (b, a)
        if nid in yc and rn in ramp_pts:
            a0, a1 = lay.nodes[nid][1], lay.nodes[nid][2]
            x = min(max(ramp_pts[rn][0], a0), a1)
            jx[nid].append(x)
            side.append((rn, nid, x))
    # 정점·간선
    rv = {}
    for rn, (x, y) in ramp_pts.items():
        if rn == lay.ramp_node or (rn < len(roots) and roots[rn] == rm):
            rv[rn] = net.v(x, y)
            net.ramp.append(rv[rn])
    aisle_v = {}
    for nid in main_nodes:
        xs = []
        for x in sorted(jx[nid]):
            if not xs or x - xs[-1] > 1e-3:
                xs.append(float(x))
        ids = [net.v(x, yc[nid]) for x in xs]
        aisle_v[nid] = (np.array(xs), ids)
        for a, b in zip(ids, ids[1:]):
            net.e(a, b, "a")
    for xc, chain in vert:
        ids = []
        for y, (t, ref) in chain:
            if t == "n":
                ids.append(net.v(xc, y))
            else:
                vid = net.v(xc, y)
                if ref in rv:
                    net.e(vid, rv[ref], "r")
                ids.append(vid)
        for a, b in zip(ids, ids[1:]):
            net.e(a, b, "c")
    for rn, nid, x in side:
        if rn in rv:
            net.e(rv[rn], net.v(x, yc[nid]), "r")
    return net, aisle_v


def dijkstra(net):
    n = len(net.pts)
    d = [math.inf] * n
    par = [-1] * n
    h = []
    for r in net.ramp:
        d[r] = 0.0
        h.append((0.0, r))
    heapq.heapify(h)
    while h:
        dv, v = heapq.heappop(h)
        if dv > d[v]:
            continue
        for j, L, _ in net.adj[v]:
            nd = dv + L
            if nd < d[j] - 1e-9:
                d[j] = nd
                par[j] = v
                heapq.heappush(h, (nd, j))
    if not net.ramp and n:
        # 경사로가 없으면 그래프 가운데(아무 정점)에서
        return dijkstra_from(net, 0)
    return np.array(d), par


def dijkstra_from(net, s):
    net.ramp = [s]
    return dijkstra(net)


def stall_access(aisle_v, d, nid, xf):
    """차로 nid 위 x=xf 의 구획까지 거리와 들어오는 정점."""
    xs, ids = aisle_v[nid]
    i = int(np.searchsorted(xs, xf))
    best = (math.inf, -1)
    if i > 0:
        best = min(best, (d[ids[i - 1]] + (xf - xs[i - 1]), ids[i - 1]))
    if i < len(xs):
        best = min(best, (d[ids[i]] + (xs[i] - xf), ids[i]))
    return best


def _access(aisle_v, d, nids, xfs):
    """구획 앞(nid, xf) 들의 경사로부터 거리와 들어오는 정점(배열 연산). 반환: {nid: (xf[], dist[], v[])}."""
    out = {}
    if not len(nids):
        return out
    dv = np.asarray(d, float)
    order = np.argsort(nids, kind="stable")
    ns, xs_all = nids[order], xfs[order]
    cuts = np.nonzero(np.diff(ns))[0] + 1
    for grp_n, grp_x in zip(np.split(ns, cuts), np.split(xs_all, cuts)):
        nid = int(grp_n[0])
        if nid not in aisle_v:
            continue
        xs, ids = aisle_v[nid]
        ids = np.asarray(ids, int)
        i = np.searchsorted(xs, grp_x)
        il = np.clip(i - 1, 0, len(xs) - 1)
        ir = np.clip(i, 0, len(xs) - 1)
        dl = np.where(i > 0, dv[ids[il]] + (grp_x - xs[il]), np.inf)
        dr = np.where(i < len(xs), dv[ids[ir]] + (xs[ir] - grp_x), np.inf)
        right = (dr < dl) | ((dr == dl) & (ids[ir] < ids[il]))
        dd = np.where(right, dr, dl)
        vv = np.where(right, ids[ir], ids[il])
        ok = np.isfinite(dd)
        out[nid] = (grp_x[ok], dd[ok], vv[ok])
    return out


def analyze(lay, S, alive, roots, main, rules, fronts=None, light=False):
    """fronts: [(nid, xf)] 구획 앞 중심. 없으면 lay 의 살아 있는 칸에서.
    light=True 는 감점만(동선 굵기·부하 계산 생략, 연결 차로 다듬기에서 수천 번 부름)."""
    net, aisle_v = build(lay, S, roots, main)
    if not net.pts:
        return dict(ok=False, penalty=0.0, n=0)
    d, par = dijkstra(net)
    if fronts is None:
        idx = np.nonzero(alive)[0]
        nids = lay.s_node[idx].astype(int)
        xfs = ((lay.s_f0[idx] + lay.s_f1[idx]) / 2).astype(float)
    else:
        nids = np.array([f[0] for f in fronts], int)
        xfs = np.array([f[1] for f in fronts], float)
    accd = _access(aisle_v, d, nids, xfs)
    dist = np.concatenate([v[1] for v in accd.values()]) if accd else np.zeros(0)
    n = len(dist)
    deg = [len(a) for a in net.adj]
    rset = set(net.ramp)
    E = sum(len(a) for a in net.adj) // 2
    V = len(net.pts)
    loops = max(0, E - V + 1)
    t3 = sum(1 for i in range(V) if deg[i] == 3 and i not in rset)
    x4 = sum(1 for i in range(V) if deg[i] >= 4 and i not in rset)
    # 막다른 차로 끝
    dead = []
    for nid, (xs, ids) in aisle_v.items():
        if len(ids) < 2:
            continue
        for end, nxt, sgn in ((0, 1, -1), (len(ids) - 1, len(ids) - 2, 1)):
            v = ids[end]
            if deg[v] == 1 and v not in rset:
                L = abs(xs[end] - xs[nxt])
                n_on = int(np.count_nonzero((accd[nid][0] - xs[nxt]) * sgn > 0)) if nid in accd else 0
                dead.append(dict(nid=nid, x=float(xs[end]), y=net.pts[v][1], len=float(L), stalls=n_on, sgn=sgn,
                                 bad=L > rules.dead_end_ok + 1e-6))
    avg = float(np.mean(dist)) if n else 0.0
    mx = float(np.max(dist)) if n else 0.0
    bad = [x for x in dead if x["bad"]]
    pen = (PEN_DIST * avg + sum(PEN_DEAD + PEN_DEAD_LEN * (x["len"] - rules.dead_end_ok) + PEN_DEAD_STALL * x["stalls"] for x in bad)
           + PEN_T * t3 + PEN_X * x4 + (PEN_NOLOOP if loops == 0 and n >= 60 else 0.0))
    res = dict(ok=True, n=n, avg=avg, max=mx, loops=loops, t3=t3, x4=x4, dead=dead, dead_bad=len(bad),
               dead_len=sum(x["len"] for x in bad), dead_stalls=sum(x["stalls"] for x in bad),
               penalty=float(pen), net=net, d=d, par=par, aisle_v=aisle_v)
    if light:
        return res
    # 부하(경로 나무)
    load = np.zeros(V)
    for _, (_, _, vv) in accd.items():
        np.add.at(load, vv, 1)
    order = np.argsort(-d)
    for v in order:
        p = par[v]
        if p >= 0 and math.isfinite(d[v]):
            load[p] += load[v]
    eload = {}
    for a, b, L, k in net.edges():
        if par[a] == b:
            eload[(a, b)] = load[a]
        elif par[b] == a:
            eload[(a, b)] = load[b]
        else:
            eload[(a, b)] = 0.0
    res["eload"] = eload
    return res


def summary(m, w):
    if not m.get("ok"):
        return dict(avg_drive=0, max_drive=0, loops=0, t_junctions=0, crossings=0, dead_ends=0, dead_end_len=0,
                    dead_end_stalls=0, penalty=0, weight=w)
    return dict(avg_drive=round(m["avg"], 1), max_drive=round(m["max"], 1), loops=m["loops"],
                t_junctions=m["t3"], crossings=m["x4"], dead_ends=m["dead_bad"],
                dead_end_len=round(m["dead_len"], 1), dead_end_stalls=m["dead_stalls"],
                penalty=round(m["penalty"] * w, 1), weight=w)


def flow_lines(m, main_frac=0.2):
    """[(p0, p1, 'main'|'sub')] 프레임 좌표. 경사로 앞마당 연결선 포함."""
    if not m.get("ok"):
        return []
    net, eload = m["net"], m["eload"]
    n = max(1, m["n"])
    out = []
    thr = max(6.0, main_frac * n)
    for (a, b), ld in eload.items():
        k = "main" if ld >= thr else "sub"
        out.append((net.pts[a], net.pts[b], k))
    return out


def optimize(lay, rules, floors, key_fn, max_iter=14):
    """연결 차로 S 를 '대수 − 가중치 × 동선 감점'이 커지도록 더하고 빼며 고친다."""
    w = rules.circ_weight

    def J(S):
        cnt, alive, main, roots = lay.score(S)
        if cnt < 0:
            return -1e9, None
        m = analyze(lay, S, alive, roots, main, rules, light=True)
        return cnt - w * m["penalty"], m

    S = list(lay.S)
    if w <= 0:
        return S
    best, _ = J(S)
    forced = set(getattr(lay, "forced", []))
    cand = [k for k in range(len(lay.conn)) if lay.conn[k] and k not in forced]
    for _ in range(max_iter):
        moves = [S + [k] for k in cand if k not in S] + [[x for x in S if x != k] for k in S if k not in forced]
        # 동선 감점 ≥ 0 이므로 J(T) ≤ 대수(T). 대수만으로 지금 최고를 못 넘는 수는 동선 계산(느림)을 건너뜀.
        # 대수가 큰 순서로 보며, 대수가 지금 최고 이하가 되면 멈춤 — 결과는 모두 계산한 것과 같음.
        ub = []
        for i, T in enumerate(moves):
            c = lay.score(T)[0]
            if c >= 0 and c > best + 1e-6:
                ub.append((c, i, T))
        ub.sort(key=lambda t: (-t[0], t[1]))
        # 큰 대지: 대수 상한이 높은 순으로 일부만 동선 계산(작은 대지는 전부 → 결과 같음)
        if len(ub) > MOVES_CAP:
            ub = ub[:MOVES_CAP]
        bv, bS, bi = best, None, None
        for c, i, T in ub:
            if c < bv - 1e-6 or (bS is None and c <= bv + 1e-6):
                break
            v, _ = J(T)
            if v > bv + 1e-6 or (bS is not None and abs(v - bv) <= 1e-6 and i < bi):
                bv, bS, bi = v, T, i
        if bS is None:
            break
        S, best = bS, bv
    return S
