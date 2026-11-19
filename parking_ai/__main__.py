"""python -m parking_ai            → 화면 열기
python -m parking_ai 도면.dxf      → 화면 없이 바로 찾고 결과 폴더에 저장
  옵션: --floors 3 --effort thorough --ramp auto --angles 90,60 --circ 1 --out 폴더
"""
import argparse
import os
import sys


def main():
    ap = argparse.ArgumentParser(prog="parking_ai")
    ap.add_argument("drawing", nargs="?")
    ap.add_argument("--floors", type=int)
    ap.add_argument("--floor-h", type=float)
    ap.add_argument("--effort", choices=["fast", "normal", "thorough"])
    ap.add_argument("--ramp", choices=["drawn", "auto", "none"])
    ap.add_argument("--angles", help="주차 각도 예: 90,60")
    ap.add_argument("--circ", type=float, help="동선 가중치 0(대수만)~2(동선 중시), 기본 1")
    ap.add_argument("--out")
    ap.add_argument("--port", type=int, default=8766)
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    if not a.drawing:
        from .app import serve
        serve(a.port, not a.no_browser)
        return
    from . import engine, export, io_input
    from .rules import Rules
    src = io_input.load(a.drawing)
    mapping = {i["name"]: i["role"] for i in src["info"]}
    print("레이어:", ", ".join(f"{i['name']}→{i['role']}" for i in src["info"]), f"(단위 {src['unit']})")
    site, cores, ramps, obs = io_input.pick(src, mapping)
    r = Rules()
    if a.floors: r.floors = a.floors
    if a.floor_h: r.floor_h = a.floor_h
    if a.effort: r.effort = a.effort
    if a.ramp: r.ramp_mode = a.ramp
    if a.circ is not None: r.circ_weight = a.circ
    if a.angles: r.park_angles = tuple(int(x) for x in a.angles.split(","))
    if r.ramp_mode == "drawn" and not ramps:
        r.ramp_mode = "none"
    res = engine.search(site, cores, ramps, obs, r)
    from . import critique
    res = res + critique.improve(site, cores, ramps, obs, r, res)
    if r.zone_search:
        from . import zones
        z = zones.search_zones(site, cores, ramps, obs, r, base_total=res[0]["stats"]["total_legal"])
        if z is not None:
            res.append(z)
    critique.annotate(res)
    best = max(res, key=lambda x: x["rank"])
    print("피드백 (안 1):")
    for f_ in res[0]["stats"]["feedback"]:
        print(f"  - {f_['title']}" + "".join(f" → {a_['text']} {a_['total']}대({a_['diff']:+d})" for a_ in f_.get("alts", [])))
    out = a.out or os.path.splitext(a.drawing)[0] + "_주차배치"
    os.makedirs(out, exist_ok=True)
    ins = {"mm": 4, "cm": 5, "m": 6}.get(src["unit"], 6)
    export.plan_png(best, os.path.join(out, "B1_평면.png"), 0)
    export.write_dxf(best, os.path.join(out, "배치.dxf"), src["scale"], ins)
    export.write_3dm(best, os.path.join(out, "모델.3dm"), src["scale"], src["unit"])
    print(export.write_stats(best, out, "결과"))
    print("저장:", out)


if __name__ == "__main__":
    main()
