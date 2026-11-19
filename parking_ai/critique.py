"""결과 피드백 + 피드백을 반영한 대안.

배치 하나를 보고 손실이 어디서 나는지 짚는다:
  - 대당 면적 (잘 짠 지하주차장 28~33㎡/대, 35 넘으면 손실이 큼 — 조사 자료의 어림값)
  - 벽 활용: 대지 변마다 구획이 등을 대고 있는 길이 / 쓸 수 있는 길이
  - 기둥 손실, 막다른 차로, 쓰지 않은 땅
그리고 고칠 수 있는 것(지금은 '차로 끝 벽을 따라 한 줄')은 그 수정을 넣어 다시 찾아 대안으로 낸다.
"""
import dataclasses
import math

from shapely.geometry import LineString, Point
from shapely.ops import unary_union

BENCH_GOOD, BENCH_BAD = 33.0, 38.0


def _dir_name(nx, ny, theta):
    a = math.degrees(math.atan2(ny, nx)) + theta
    a = (a + 360) % 360
    if a < 30 or a >= 330:
        return "오른쪽"
    if 60 <= a < 120:
        return "위쪽"
    if 150 <= a < 210:
        return "왼쪽"
    if 240 <= a < 300:
        return "아래쪽"
    return {0: "오른쪽 위", 1: "왼쪽 위", 2: "왼쪽 아래", 3: "오른쪽 아래"}[int(((a - 30) % 360) // 90) % 4]


def _merge(iv):
    iv = sorted(iv)
    out = []
    for a, b in iv:
        if out and a <= out[-1][1] + 0.05:
            out[-1][1] = max(out[-1][1], b)
        else:
            out.append([a, b])
    return out


def _proj(seg, g, L):
    (x1, y1), (x2, y2) = seg
    tx, ty = (x2 - x1) / L, (y2 - y1) / L
    ts = []
    geoms = getattr(g, "geoms", [g])
    for gg in geoms:
        cs = list(gg.exterior.coords) if gg.geom_type == "Polygon" else list(gg.coords)
        ts += [(x - x1) * tx + (y - y1) * ty for x, y in cs]
    if not ts:
        return None
    return max(0.0, min(ts)), min(L, max(ts))


def wall_usage(res):
    """대지 변마다 (이름, 길이, 쓸 수 있는 길이, 구획이 붙은 길이, 끝 종류 lo/hi/None)."""
    fr = res["frame"]
    usable = fr.usable
    stalls = [s["poly"] for s in res["stalls_b1"]]
    lay = res["layout"]
    block = list(fr.cores) + list(lay.aprons)
    if res.get("ramp") is not None:
        block.append(res["ramp"]["poly"])
    block_u = unary_union(block) if block else None
    cx, cy = usable.centroid.x, usable.centroid.y
    minx, miny, maxx, maxy = fr.bounds
    out = []
    cs = list(usable.exterior.coords)
    for a, b in zip(cs, cs[1:]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        if L < 8:
            continue
        seg = (a, b)
        strip = LineString(seg).buffer(2.6, cap_style=2).intersection(usable)
        nx, ny = (b[1] - a[1]) / L, -(b[0] - a[0]) / L
        mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        if (mx - cx) * nx + (my - cy) * ny < 0:
            nx, ny = -nx, -ny
        blocked = []
        if block_u is not None:
            bi = block_u.buffer(0.5).intersection(strip)
            if not bi.is_empty:
                for gg in getattr(bi, "geoms", [bi]):
                    p = _proj(seg, gg, L)
                    if p and p[1] > p[0]:
                        blocked.append(p)
        cov = []
        for sp in stalls:
            if sp.intersects(strip):
                gi = sp.intersection(strip)
                if gi.area < 0.3:
                    continue
                p = _proj(seg, gi, L)
                if p and p[1] > p[0]:
                    cov.append(p)
        blk = sum(b_ - a_ for a_, b_ in _merge(blocked))
        cvd = sum(b_ - a_ for a_, b_ in _merge(cov))
        avail = max(0.0, L - blk)
        end = None
        if abs(b[0] - a[0]) / L < 0.15:          # 프레임에서 세로 변 = 차로가 끝나는 쪽 벽
            if abs(mx - minx) < 1.0:
                end = "lo"
            elif abs(mx - maxx) < 1.0:
                end = "hi"
        out.append(dict(name=_dir_name(nx, ny, fr.theta), length=round(L, 1), avail=round(avail, 1),
                        covered=round(min(cvd, avail), 1), end=end))
    return out


def critique(res):
    st = res["stats"]
    fr = res["frame"]
    rules = res["rules"]
    items = []
    b1 = st["per_floor"][0]["legal"]
    aps = fr.usable.area / max(b1, 1)
    lvl = "good" if aps <= BENCH_GOOD else ("warn" if aps <= BENCH_BAD else "bad")
    items.append(dict(kind="area", level=lvl, title=f"대당 면적 {aps:.1f}㎡",
                      detail=f"층 면적 {fr.usable.area:,.0f}㎡ ÷ {b1}대. 잘 짠 지하주차장은 보통 28~33㎡/대이고, "
                             f"35㎡를 넘으면 어딘가에서 구획을 잃고 있다는 뜻입니다 (대지 모양·코어에 따라 달라짐)."))

    walls_now = set(getattr(rules, "wall_rows", ()) or ())
    for w in wall_usage(res):
        if w["avail"] < 12:
            continue
        lost = (w["avail"] - w["covered"]) / rules.stall_w
        ratio = w["covered"] / max(w["avail"], 1e-6)
        if ratio >= 0.7 or lost < 3:
            continue
        fix = None
        if st.get("zones"):
            how = "구역별 방향 안이라 벽 쪽 줄은 그 구역 방향을 따릅니다. 이 벽을 더 쓰려면 다른 안의 벽 한 줄 대안을 보세요."
        elif w["end"] and w["end"] not in walls_now:
            fix = dict(wall_rows=tuple(sorted(walls_now | {w["end"]})), text=f"{w['name']} 벽을 따라 한 줄")
            how = "차로를 벽 앞에서 끊고 벽 앞에 가로 차로 + 벽에 붙인 구획 한 줄을 두는 대안을 만들어 봤습니다."
        elif w["end"]:
            how = "이미 벽 따라 한 줄을 둔 안입니다. 남은 부분은 코어·기둥·모서리 때문입니다."
        else:
            how = "차로와 나란한 벽이라, 줄 쌓기 시작 위치를 바꾸는 쪽(이미 탐색함)으로만 고칠 수 있습니다."
        items.append(dict(kind="wall", level="bad" if ratio < 0.4 else "warn",
                          title=f"{w['name']} 벽 {w['avail']:.0f}m 중 {w['covered']:.0f}m만 주차",
                          detail=f"벽에 등을 댄 구획이 적어 약 {lost:.0f}대 자리가 비어 있습니다. " + how, fix=fix))

    # 대지 깊이(차로에 수직) ↔ 양쪽 주차 모듈(줄+차로+줄)
    if st["park_angle"] == 90 and not st.get("zones"):
        Dn = rules.stall_l + rules.aisle_90 + rules.stall_l
        De = rules.ext_l + rules.aisle_90 + rules.ext_l
        minx, miny, maxx, maxy = fr.usable.bounds
        depth = maxy - miny
        k = int(depth // Dn)
        rest = depth - k * Dn
        one = rules.stall_l + rules.aisle_90          # 한쪽 주차 차로
        if rest >= one:
            txt = f"남는 {rest:.1f}m에 한쪽 주차 차로(줄+차로 {one:.0f}m)가 더 들어갑니다."
        else:
            txt = (f"남는 {rest:.1f}m는 주차로 못 씁니다. 깊이가 {k * Dn + one - depth:.1f}m 더 있으면 한쪽 주차 줄이, "
                   f"{(k + 1) * Dn - depth:.1f}m 더 있으면 모듈 하나가 더 들어갑니다.")
        items.append(dict(kind="depth", level="info" if rest < 3 or rest >= one else "warn",
                          title=f"차로에 수직인 깊이 {depth:.1f}m = 양쪽 주차 모듈 {Dn:.0f}m × {k} + {rest:.1f}m",
                          detail=f"모듈 = 줄 {rules.stall_l:g} + 차로 {rules.aisle_90:g} + 줄 {rules.stall_l:g}m "
                                 f"(확장형이면 {De:.1f}m). " + txt + " 코어가 줄 자리에 들어가면 손실이 적습니다."))

    ph = st["phase"]
    if ph["no_columns"]:
        loss = ph["column_loss"] / ph["no_columns"]
        if loss >= 0.08:
            items.append(dict(kind="column", level="warn" if loss < 0.15 else "bad",
                              title=f"기둥 때문에 {ph['column_loss']}면 줄어듦 ({loss * 100:.0f}%)",
                              detail=f"기둥 없이 {ph['no_columns']}면 → 기둥 넣고 {ph['with_columns']}면. 줄이 짧게 끊길수록 "
                                     f"기둥 사이 남는 폭이 많아집니다. 기둥 간격 최대값({rules.span_max:.1f}m)을 구조 검토로 늘리면 줄어듭니다."))
    cs = st["circulation"]
    if cs.get("dead_ends"):
        items.append(dict(kind="dead", level="warn", title=f"막다른 차로 {cs['dead_ends']}곳",
                          detail=f"'회차' 표시가 있는 곳은 끝까지 들어가서 돌아 나와야 합니다. 동선 가중치를 2로 올리면 줄이는 쪽으로 찾습니다."))

    # 쓰지 않은 땅
    used = [s["poly"] for s in res["stalls_b1"]] + list(res["columns"]) + list(fr.cores)
    if res.get("drive") is not None and not res["drive"].is_empty:
        used.append(res["drive"])
    if res.get("ramp") is not None:
        used.append(res["ramp"]["poly"])
    used += list(res["layout"].aprons)
    left = fr.usable.difference(unary_union(used).buffer(0.05))
    big = [g for g in getattr(left, "geoms", [left]) if g.area >= 12.5]
    area = sum(g.area for g in big)
    if area >= 0.05 * fr.usable.area:
        items.append(dict(kind="left", level="warn", title=f"쓰지 않은 땅 {area:,.0f}㎡ ({area / fr.usable.area * 100:.0f}%)",
                          detail=f"구획·차로·코어 어디에도 안 쓰인 조각(12.5㎡ 이상) {len(big)}곳. "
                                 f"그림에서 흰 빈칸으로 보이는 곳입니다."))
    order = {"bad": 0, "warn": 1, "good": 2, "info": 3}
    items.sort(key=lambda it: (order[it["level"]], it["kind"] != "wall"))
    return items


def improve(site, cores, ramps, obstacles, rules, finals, log=print, stop=None, executor=None, max_runs=3):
    """1안의 피드백 중 고칠 수 있는 것을 넣어 다시 찾고 대안으로 돌려줌. 반환: 대안 결과 목록."""
    from . import engine
    if not finals:
        return []
    base = finals[0]
    fb = critique(base)
    base["stats"]["feedback"] = fb
    fixes = [it for it in fb if it.get("fix")]
    if not fixes:
        return []
    combos = [(it["fix"]["wall_rows"], it["fix"]["text"], [it]) for it in fixes]
    ends = sorted(set(e for it in fixes for e in it["fix"]["wall_rows"]))
    if len(fixes) >= 2 and len(ends) == 2:
        combos.append((tuple(ends), "양쪽 끝 벽을 따라 한 줄씩", fixes))
    st = base["stats"]
    alts = []
    if site.area > 20000:
        max_runs = 0        # 큰 대지: 다시 찾지 않고, 이미 찾은 안 중 같은 조합만 연결(시간)
    runs = 0
    for walls, text, src in combos:
        if stop and stop():
            break
        # 이미 같은 조합으로 찾은 안이 있으면 그것을 대안으로 가리킴
        same = [(i, f) for i, f in enumerate(finals) if f["stats"]["grid_angle"] == st["grid_angle"]
                and f["stats"]["park_angle"] == st["park_angle"] and sorted(f["stats"].get("walls", ())) == sorted(walls)]
        if same:
            i, f = same[0]
            d = f["stats"]["total_legal"] - st["total_legal"]
            for it in src:
                it.setdefault("alts", []).append(dict(index=i, text=text, total=f["stats"]["total_legal"], diff=d))
            continue
        if runs >= max_runs:
            continue
        runs += 1
        r2 = dataclasses.replace(rules, grid_angles=(st["grid_angle"],), park_angles=(st["park_angle"],),
                                 wall_rows=walls)
        log(f"피드백 반영: {text} → 다시 찾는 중")
        try:
            res = engine.search(site, cores, ramps, obstacles, r2, log=lambda m: None, stop=stop, executor=executor)
        except Exception as e:  # 대안 하나가 안 돼도 본 결과는 살림
            log(f"  {text}: 실패 ({e})")
            continue
        if not res:
            continue
        a = res[0]
        a["rules"] = dataclasses.replace(a["rules"], effort=rules.effort)
        d = a["stats"]["total_legal"] - st["total_legal"]
        a["label"] = "대안_" + text.replace(" ", "")[:16]
        a["stats"]["tab"] = "대안: " + text
        a["stats"]["alt_note"] = f"안 1에 '{text}'를 넣어 다시 찾은 결과 · 안 1 대비 {d:+d}대"
        a["stats"]["feedback"] = critique(a)
        log(f"  {text}: {a['stats']['total_legal']}대 (안 1 대비 {d:+d})")
        for it in src:
            it.setdefault("alts", []).append(dict(index=len(finals) + len(alts), text=text,
                                                  total=a["stats"]["total_legal"], diff=d))
        alts.append(a)
    return alts


def wall_text(res):
    """이 안에 둔 '벽 따라 한 줄'을 사람이 읽는 말로 (예: '아래쪽 벽 한 줄')."""
    walls = res["stats"].get("walls") or []
    if not walls:
        return ""
    names = []
    for w in wall_usage(res):
        if w["end"] in walls and w["name"] not in names:
            names.append(w["name"])
    return " · ".join(f"{n} 벽 한 줄" for n in names) or "끝 벽 한 줄"


def annotate(results):
    """모든 결과에 피드백·탭 이름을 붙임 (대안 연결은 improve 가 붙인 1안 것 유지)."""
    usage = []
    for r in results:
        try:
            usage.append({w["name"]: w for w in wall_usage(r) if w["avail"] >= 12})
        except Exception:
            usage.append({})
    for i, r in enumerate(results):
        st = r["stats"]
        if "feedback" not in st:
            st["feedback"] = critique(r)
        st["wall_text"] = st.get("zone_text") or wall_text(r)
        # 다른 안의 장점: 같은 벽을 훨씬 잘 쓰는 안
        for name, w in usage[i].items():
            best = None
            for j, u in enumerate(usage):
                if j == i or name not in u:
                    continue
                g = u[name]["covered"] - w["covered"]
                if g >= max(7.5, 0.3 * w["avail"]) and (best is None or g > best[1]):
                    best = (j, g)
            if best:
                j, g = best
                tab = results[j]["stats"].get("tab") or f"안 {j + 1}"
                st["feedback"].append(dict(
                    kind="borrow", level="info",
                    title=f"{tab}은 {name} 벽을 {usage[j][name]['covered']:.0f}m 씀 (이 안 {w['covered']:.0f}m)",
                    detail=f"그 벽 쪽은 {tab}의 줄 배치가 약 {g / 2.5:.0f}대 더 넣습니다. 두 안 전체 대수는 "
                           f"{results[j]['stats']['total_legal']}대 / {st['total_legal']}대.",
                    alts=[dict(index=j, text=tab, total=results[j]["stats"]["total_legal"],
                               diff=results[j]["stats"]["total_legal"] - st["total_legal"])]))
