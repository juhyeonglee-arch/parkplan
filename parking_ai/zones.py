"""구역별 차로 방향: 대지를 같이 쓰는 가로 차로(연결 차로) 하나로 두 구역으로 나누고,
구역마다 격자·차로 방향을 따로 찾아 한 안으로 합친다.

- 나누는 자리: 대지 긴 방향에 수직인 띠(연결 차로 너비). 코어·경사로 끝선에 맞춘 자리 + 1/3·1/2·2/3.
  경사로를 가로지르거나 한쪽 구역이 너무 작으면 뺌.
- 구역 탐색: 같이 쓰는 차로(띠 − 코어)를 그 구역의 '앞마당'으로 붙여, 구역 차로가 그 띠에 이어져야 하게 함
  (경사로 없는 구역은 띠를 통해 경사로가 있는 구역으로 나감).
- 합치기: 두 구역 결과를 A 구역 격자 좌표로 옮겨 한 결과로. 띠 옆 남은 자리는 한 번 더 채움.
"""
import dataclasses
import math
import time

from shapely import affinity
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

from . import engine, fill
from .engine import Frame, legal_count, _counts


def _axis(site):
    rr = site.minimum_rotated_rectangle
    cs = list(rr.exterior.coords)
    e1 = (cs[1][0] - cs[0][0], cs[1][1] - cs[0][1])
    e2 = (cs[2][0] - cs[1][0], cs[2][1] - cs[1][1])
    e = e1 if math.hypot(*e1) >= math.hypot(*e2) else e2
    L = math.hypot(*e)
    return (e[0] / L, e[1] / L)


def _proj_range(g, u):
    cs = []
    for gg in getattr(g, "geoms", [g]):
        cs += list(gg.exterior.coords)
    ts = [x * u[0] + y * u[1] for x, y in cs]
    return min(ts), max(ts)


def _band(t0, t1, u, site):
    """u 방향 좌표가 t0~t1 인 띠(대지를 덮는 큰 사각형)."""
    v = (-u[1], u[0])
    minx, miny, maxx, maxy = site.bounds
    R = math.hypot(maxx - minx, maxy - miny) + 10
    c = site.centroid
    sv = c.x * v[0] + c.y * v[1]
    pts = []
    for t, s in ((t0, sv - R), (t1, sv - R), (t1, sv + R), (t0, sv + R)):
        pts.append((u[0] * t + v[0] * s, u[1] * t + v[1] * s))
    return Polygon(pts)


def _single(g):
    if g.is_empty:
        return None
    if g.geom_type == "Polygon":
        return g
    ps = [p for p in getattr(g, "geoms", []) if p.geom_type == "Polygon"]
    if not ps:
        return None
    ps.sort(key=lambda p: p.area, reverse=True)
    # 큰 조각 하나만 있어야 함(나머지가 크면 이 자리는 안 씀)
    if len(ps) > 1 and ps[1].area > 0.05 * ps[0].area:
        return None
    return ps[0]


def cuts(site, cores, ramps, obstacles, rules, max_n=5):
    """나누는 자리 후보: [(zoneA, zoneB, gate, t)]"""
    u = _axis(site)
    t_lo, t_hi = _proj_range(site, u)
    span = t_hi - t_lo
    w = rules.link_aisle
    ts = [t_lo + span * f for f in (0.5, 1 / 3, 2 / 3)]
    for g in list(cores) + list(ramps):
        a, b = _proj_range(g, u)
        ts += [a - w / 2, b + w / 2]
    ts.sort(key=lambda t: abs(t - (t_lo + t_hi) / 2))
    uniq = []
    for t in ts:
        if all(abs(t - s) > 3.0 for s in uniq):
            uniq.append(t)
    out = []
    blk = unary_union(list(cores) + list(obstacles)) if (cores or obstacles) else None
    for t in uniq:
        if t - w / 2 < t_lo + 0.15 * span or t + w / 2 > t_hi - 0.15 * span:
            continue
        strip = _band(t - w / 2, t + w / 2, u, site).intersection(site)
        if strip.is_empty or any(r.intersects(strip) for r in ramps):
            continue
        za = _single(site.intersection(_band(t_lo - 1, t - w / 2, u, site)))
        zb = _single(site.intersection(_band(t + w / 2, t_hi + 1, u, site)))
        if za is None or zb is None:
            continue
        if min(za.area, zb.area) < max(500.0, 0.15 * site.area):
            continue
        gate = strip.difference(blk) if blk is not None else strip
        if blk is not None and strip.intersection(blk).area > 0.3 * strip.area:
            continue
        gate = _single(gate)
        if gate is None or gate.area < 0.5 * strip.area:
            continue
        out.append((za, zb, gate, t))
        if len(out) >= max_n:
            break
    return out, u


def _zone_inputs(zone, cores, ramps, obstacles):
    zc = [c for c in cores if c.intersects(zone)]
    zr = [r for r in ramps if r.intersects(zone)]
    zo = [o for o in obstacles if o.intersects(zone)]
    return zc, zr, zo


def _theta_of(u):
    return math.degrees(math.atan2(u[1], u[0])) % 180


# ------------------------------------------------------------------ 합치기
def _xf(frB, frA):
    def f(g):
        return affinity.rotate(frB.to_world(g), -frA.theta, origin=frA.origin)
    return f


def _xp(frB, frA, p):
    q = _xf(frB, frA)(Point(p))
    return (q.x, q.y)


def _rot(v, deg):
    a = math.radians(deg)
    return (v[0] * math.cos(a) - v[1] * math.sin(a), v[0] * math.sin(a) + v[1] * math.cos(a))


def _move_stalls(sts, frB, frA, row_off):
    f = _xf(frB, frA)
    d = frB.theta - frA.theta
    out = []
    for s in sts:
        s2 = dict(s)
        s2["poly"] = f(s["poly"])
        s2["into"] = _rot(s["into"], d)
        s2["row"] = s["row"] + row_off if s["row"] < 10 ** 6 else s["row"]
        s2["x"] = float(s2["poly"].centroid.x)
        out.append(s2)
    return out


def _dir_word(res):
    """그 구역 차로가 화면에서 어느 방향으로 달리는지 (가로/세로/비스듬)."""
    th = res["frame"].theta % 180
    if th < 20 or th > 160:
        return "가로 차로"
    if 70 < th < 110:
        return "세로 차로"
    return f"{th:.0f}° 차로"


def _route_gate_flows(flows, gate_f):
    """구역 탐색은 같이 쓰는 차로를 한 점(띠 가운데)으로 봐서 흐름선이 코어를 질러 갈 수 있음 →
    그 점으로 가는 선은 띠 중심선에 수직으로 붙이고, 띠 중심선을 주 동선으로 그림."""
    gc = gate_f.centroid
    rr = gate_f.minimum_rotated_rectangle
    cs = list(rr.exterior.coords)
    e = max(((cs[i], cs[i + 1]) for i in range(4)), key=lambda ab: math.hypot(ab[1][0] - ab[0][0], ab[1][1] - ab[0][1]))
    dx, dy = e[1][0] - e[0][0], e[1][1] - e[0][1]
    L = math.hypot(dx, dy) or 1.0
    ux, uy = dx / L, dy / L
    proj = lambda p: (p[0] - gc.x) * ux + (p[1] - gc.y) * uy
    out, ts = [], []
    for a, b, k in flows:
        da = math.hypot(a[0] - gc.x, a[1] - gc.y) < 0.6
        db = math.hypot(b[0] - gc.x, b[1] - gc.y) < 0.6
        if da or db:
            o = b if da else a
            t = proj(o)
            q = (gc.x + ux * t, gc.y + uy * t)
            ts.append(t)
            if math.hypot(q[0] - o[0], q[1] - o[1]) > 0.3:
                out.append((o, q, k))
            continue
        out.append((a, b, k))
    if len(ts) >= 2 or ts:
        t0, t1 = min(ts), max(ts)
        if t1 - t0 > 0.5:
            ln_ = LineString([(gc.x + ux * t0, gc.y + uy * t0), (gc.x + ux * t1, gc.y + uy * t1)]).intersection(
                gate_f.buffer(0.05))
            for g in getattr(ln_, "geoms", [ln_]):
                if g.geom_type == "LineString" and g.length > 0.5:
                    out.append((g.coords[0], g.coords[-1], "main"))
    return out


def merge(site, cores, ramps, obstacles, full, parts, gate, log=print):
    """parts = [res_A, res_B] (각 구역 탐색 결과) → 대지 전체 한 결과."""
    A = parts[0]
    frA0 = A["frame"]
    frA = Frame(site, cores, ramps if full.ramp_mode == "drawn" else [], obstacles, frA0.theta, full)
    floors = full.floors
    b1, bb, cols, drive, flows, dead, ramp_polys, mouth = [], [], [], [], [], [], [], None
    grid_world = []
    aprons = []
    arrows = []
    for zi, R in enumerate(parts):
        fr = R["frame"]
        f = _xf(fr, frA)
        off = zi * 10 ** 5
        b1 += _move_stalls(R["stalls_b1"], fr, frA, off)
        bb += _move_stalls(R["stalls"], fr, frA, off)
        cols += [f(c) for c in R["columns"]]
        if R["drive"] is not None and not R["drive"].is_empty:
            drive.append(f(R["drive"]))
        flows += [(_xp(fr, frA, a), _xp(fr, frA, b), k) for a, b, k in R.get("flows", [])]
        dead += [(*_xp(fr, frA, (x, y)), sg) for x, y, sg in R.get("dead", [])]
        for x, y, dx, dy in R.get("arrows", []):
            q = _xp(fr, frA, (x, y))
            v = _rot((dx, dy), fr.theta - frA.theta)
            arrows.append((q[0], q[1], v[0], v[1]))
        if R.get("ramp") is not None and not R["ramp"]["poly"].is_empty:
            ramp_polys.append(f(R["ramp"]["poly"]))
            if mouth is None and R["ramp"].get("mouth_mid"):
                mouth = _xp(fr, frA, R["ramp"]["mouth_mid"])
        aprons += [f(a) for a in R["layout"].aprons]
        g = R.get("grid") or {}
        minx, miny, maxx, maxy = fr.site.bounds
        grp = dict(x=[fr.to_world(LineString([(x, miny - 4), (x, maxy + 4)])) for x in g.get("xs", [])],
                   y=[fr.to_world(LineString([(minx - 4, y), (maxx + 4, y)])) for y in g.get("ys", [])])
        grid_world.append(grp)
    gate_f = affinity.rotate(gate, -frA.theta, origin=frA.origin)
    flows = _route_gate_flows(flows, gate_f)
    drive_u = unary_union(drive + [gate_f]).intersection(frA.usable)

    # 같이 쓰는 차로 옆 자투리 채우기(구역 경계에 생긴 빈자리)
    n_before = len(bb)
    if full.edge_fill:
        blk = [s["poly"] for s in bb] + cols + list(frA.cores) + aprons + ramp_polys + [drive_u]
        cnt = _counts(bb)
        sizes = {"N": (full.stall_w, full.stall_l), "E": (full.ext_w, full.ext_l), "C": (full.compact_w, full.compact_l)}
        if not full.compact_fill:
            sizes["C"] = (1e9, 1e9)
        try:
            add = fill.fill(frA.region, gate_f, blk, sizes, len(bb), cnt["E"], full.ext_ratio, full.ext_threshold, step=0.1)
        except Exception:
            add = []
        for k, poly, into, W, L in add:
            c = poly.centroid
            s = dict(kind=k, poly=poly, row=10 ** 6, bay=-1, x=float(c.x), W=W, L=L, lean=1, into=into,
                     ev=False, piece=None)
            bb.append(s)
            b1.append(dict(s))
    n_fill = len(bb) - n_before

    n1, nb = _counts(b1), _counts(bb)
    per_floor = [n1] + [nb] * (floors - 1)
    legal = [legal_count(n, full) for n in per_floor]
    phys = [n["N"] + n["E"] + n["C"] + n["A"] for n in per_floor]
    tot_legal, tot_phys = sum(legal), sum(phys)
    ext_tot = sum(n["E"] for n in per_floor)
    acc_tot = sum(n["A"] for n in per_floor)
    ev_tot = sum(n["V"] for n in per_floor)

    checks = []
    if tot_phys >= full.ext_threshold:
        checks.append(dict(item="확장형 비율", need=f"{full.ext_ratio * 100:.0f}% 이상 ({math.ceil(full.ext_ratio * tot_phys)}대)",
                           have=f"{ext_tot}대 ({ext_tot / max(tot_phys, 1) * 100:.1f}%)",
                           ok=ext_tot >= full.ext_ratio * tot_phys - 1e-9))
    else:
        checks.append(dict(item="확장형 비율", need=f"총 {full.ext_threshold}대 미만 → 해당 없음", have=f"{ext_tot}대", ok=True))
    need_acc = math.ceil(full.acc_ratio * tot_legal - 1e-9) if tot_legal >= full.acc_min_total else 0
    checks.append(dict(item="장애인전용 구획", need=f"{full.acc_ratio * 100:.0f}% → {need_acc}대",
                       have=f"{acc_tot}대 (B1 코어 근처)", ok=acc_tot >= need_acc))
    if full.ev_ratio > 0:
        need_ev = math.ceil(full.ev_ratio * tot_legal - 1e-9)
        checks.append(dict(item="전기차 전용구획", need=f"{full.ev_ratio * 100:.0f}% → {need_ev}대 (층마다 경사로 가까이)",
                           have=f"{ev_tot}대", ok=ev_tot >= need_ev))
    skip = {"확장형 비율", "장애인전용 구획", "전기차 전용구획"}
    for zi, R in enumerate(parts):
        for ch in R["stats"]["checks"]:
            if ch["item"] in skip or ch.get("ramp"):
                continue
            c2 = dict(ch)
            c2["item"] = f"{'AB'[zi]} 구역 · {ch['item']}"
            if ch["item"] == "경사로" and (R.get("ramp") is None or R["ramp"]["poly"].is_empty):
                c2["item"] = f"{'AB'[zi]} 구역 · 같이 쓰는 차로로 나감"
                c2["need"] = "경사로가 없는 구역: 구역 차로가 같이 쓰는 차로에 이어져 경사로 쪽으로 나감"
            checks.append(c2)
    if frA.ramps:
        from . import oneway
        checks += oneway.ramp_checks(frA.ramps, tot_legal, full)
    checks.append(dict(item="구역 나누기", need=f"같이 쓰는 차로 {full.link_aisle:.1f}m로 두 구역이 이어짐",
                       have=f"A {parts[0]['stats']['grid_angle']:.0f}° · B {parts[1]['stats']['grid_angle']:.0f}° 격자",
                       ok=True))

    cs = [dict(R["stats"]["circulation"]) for R in parts]
    # 경사로가 없는 구역의 주행거리는 같이 쓰는 차로에서 잰 것 → 경사로 입구에서 그 차로까지를 더함
    if mouth is not None:
        gc = gate_f.centroid
        for c_, R in zip(cs, parts):
            if R.get("ramp") is None or R["ramp"]["poly"].is_empty:
                extra = math.hypot(gc.x - mouth[0], gc.y - mouth[1])
                c_["avg_drive"] = round(c_["avg_drive"] + extra, 1)
                c_["max_drive"] = round(c_["max_drive"] + extra, 1)
    wts = [max(1, R["stats"]["total_legal"]) for R in parts]
    csum = dict(
        avg_drive=round(sum(c["avg_drive"] * w for c, w in zip(cs, wts)) / sum(wts), 1),
        max_drive=max(c["max_drive"] for c in cs),
        **{k: sum(c[k] for c in cs) for k in ("loops", "t_junctions", "crossings", "dead_ends", "dead_end_stalls")},
        dead_end_len=round(sum(c["dead_end_len"] for c in cs), 1),
        penalty=round(sum(c["penalty"] for c in cs), 1), weight=cs[0]["weight"])
    # 구역 점수(대수 − 동선 감점)의 감점 몫을 그대로 더함
    score = tot_legal - sum(R["stats"]["total_legal"] - R["stats"]["score"] for R in parts)
    spans = sorted(set(s for R in parts for s in R["stats"]["spans"]))
    allspans = [s for R in parts for s in R["stats"]["spans"]]
    ph = [R["stats"]["phase"] for R in parts]
    rank = (1 if all(ch["ok"] for ch in checks if ch["item"] != "전기차 전용구획" and not ch.get("ramp")) else 0, round(score, 3), tot_legal,
            -len(cols))
    words = [_dir_word(R) for R in parts]
    zone_text = f"A {words[0]} + B {words[1]}"
    stats = dict(
        grid_angle=A["stats"]["grid_angle"], park_angle=A["stats"]["park_angle"],
        bay=(round(sum(allspans) / len(allspans), 2) if allspans else 0),
        ext_mask=A["stats"]["ext_mask"], aisle_w=A["stats"]["aisle_w"], floors=floors,
        per_floor=[dict(floor=f"B{i + 1}", 일반=n["N"] - n["VN"], 확장=n["E"] - n["VE"],
                        경형=n["C"], 장애인=n["A"], 전기차=n["V"], physical=phys[i], legal=legal[i])
                   for i, n in enumerate(per_floor)],
        per_floor_legal=legal[-1] if floors > 1 else legal[0],
        total_legal=tot_legal, total_physical=tot_phys, total_ev=ev_tot,
        floor_area=round(frA.usable.area, 1), park_area=round(frA.usable.area - sum(c.area for c in frA.cores), 1),
        area_per_stall=round(frA.usable.area / max(legal[0], 1), 2),
        columns=len(cols), links=sum(R["stats"]["links"] for R in parts), spans=spans,
        phase=dict(no_columns=sum(p["no_columns"] for p in ph), with_columns=sum(p["with_columns"] for p in ph),
                   column_loss=sum(p["column_loss"] for p in ph), edge_fill=sum(p["edge_fill"] for p in ph) + n_fill),
        circulation=csum, score=round(score, 1), checks=checks,
        need_floors=(math.ceil(full.target_total / max(legal[-1], 1)) if full.target_total else None),
        walls=[], zones=[dict(grid_angle=R["stats"]["grid_angle"], park_angle=R["stats"]["park_angle"],
                              total=R["stats"]["total_legal"], dir=w) for R, w in zip(parts, words)],
        zone_text=zone_text,
    )
    ramp = None
    if ramp_polys:
        ramp = dict(poly=unary_union(ramp_polys), mouth_mid=mouth, drawn=True)

    class _Lay:
        pass
    lay = _Lay()
    lay.aprons = aprons + [gate_f]
    lay.frame = frA
    return dict(frame=frA, layout=lay, rules=full, stalls_b1=b1, stalls=bb, columns=cols, drive=drive_u, ramp=ramp,
                stats=stats, rank=rank, grid={"xs": [], "ys": [], "spans": allspans}, grid_world=grid_world,
                flows=flows, dead=dead, arrows=arrows, zone_gate=gate, zone_parts=[R["stats"]["grid_angle"] for R in parts])


# ------------------------------------------------------------------ 찾기
def search_zones(site, cores, ramps, obstacles, rules, base_total=None, log=print, stop=None, executor=None,
                 max_cuts=None):
    """두 구역으로 나눈 안 중 가장 좋은 것 하나 (없으면 None)."""
    if rules.ramp_mode != "drawn" or not ramps:
        return None
    t0 = time.time()
    if max_cuts is None:
        max_cuts = 2 if site.area > 20000 else 4
    cs, u = cuts(site, cores, ramps, obstacles, rules, max_n=max_cuts)
    if not cs:
        log("구역 나누기: 나눌 자리가 없음(경사로·코어가 가로막거나 대지가 작음)")
        return None
    th = _theta_of(u)
    small = site.area <= 20000
    gas = (round(th, 3), round((th + 90) % 180, 3))
    r_fast = dataclasses.replace(rules, effort="fast", grid_angles=gas, wall_rows=(), wall_search=False)

    def run_cut(cut, r_z):
        za, zb, gate, t = cut
        parts = []
        for z in (za, zb):
            zc, zr, zo = _zone_inputs(z, cores, ramps, obstacles)
            try:
                res = engine.search(z, zc, zr, zo, r_z, log=lambda m: None, stop=stop, executor=executor, gates=[gate])
            except Exception as e:
                log(f"  구역 탐색 실패: {e}")
                res = None
            if not res or res[0]["rank"][0] != 1:
                return None
            parts.append(res)
        # 두 구역 방향이 다른 조합 중 대수 합이 가장 큰 것 (같은 방향이면 기존 안과 다를 게 없음)
        cands = []
        for ra in parts[0][:4]:
            for rb in parts[1][:4]:
                if abs(ra["stats"]["grid_angle"] - rb["stats"]["grid_angle"]) % 180 > 1:
                    cands.append((ra["stats"]["total_legal"] + rb["stats"]["total_legal"], ra, rb))
        if not cands:
            return None
        _, ra, rb = max(cands, key=lambda c: c[0])
        try:
            m = merge(site, cores, ramps, obstacles, rules, [ra, rb], gate, log=log)
        except Exception as e:
            import traceback
            traceback.print_exc()
            log(f"  합치기 실패: {e}")
            return None
        log(f"  나눈 자리 {t:.0f}: A {ra['stats']['total_legal']} + B {rb['stats']['total_legal']} → "
            f"{m['stats']['total_legal']}대 ({m['stats']['zone_text']})")
        return m

    # 1) 나눌 자리마다 빠르게 → 2) 가장 좋은 자리만 꼼꼼히(작은 대지: 보통 + 벽 한 줄 조합)
    best, best_cut = None, None
    for cut in cs:
        if stop and stop():
            break
        m = run_cut(cut, r_fast)
        if m is not None and (best is None or m["rank"] > best["rank"]):
            best, best_cut = m, cut
    if best_cut is not None and small and rules.effort != "fast" and not (stop and stop()):
        r_full = dataclasses.replace(rules, effort="normal" if rules.effort != "thorough" else "normal",
                                     grid_angles=gas, wall_rows=(), wall_search=True)
        m = run_cut(best_cut, r_full)
        if m is not None and m["rank"] > best["rank"]:
            best = m
    if best is not None and base_total and best["stats"]["total_legal"] < base_total - max(2, 0.03 * base_total):
        log(f"구역별 방향: 최고 {best['stats']['total_legal']}대로 안 1({base_total}대)보다 많이 적어 대안으로 넣지 않음")
        best = None
    if best is not None:
        d = best["stats"]["total_legal"] - (base_total or 0)
        best["label"] = "대안_구역나누기"
        best["stats"]["tab"] = "대안: 구역별 방향"
        best["stats"]["alt_note"] = (f"대지를 같이 쓰는 차로로 두 구역으로 나눠 방향을 따로 찾은 결과 ({best['stats']['zone_text']})"
                                     + (f" · 안 1 대비 {d:+d}대" if base_total else ""))
        log(f"구역별 방향: {best['stats']['total_legal']}대 · {time.time() - t0:.0f}초")
    return best
