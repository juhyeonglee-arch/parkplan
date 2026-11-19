"""v1.1 시험: 예시 대지 → 1단계(기둥 없이·동선) → 2단계(기둥) → 내보내기."""
import os, sys, time
from shapely.geometry import Polygon, box
from parking_ai.engine import search
from parking_ai.rules import Rules
from parking_ai import export

if __name__ == "__main__":
    site = Polygon([(0, 0), (70, 0), (76, 30), (68, 50), (0, 50)])
    cores = [box(20, 20, 28, 30), box(48, 8, 55, 16)]
    ramp = [box(0, 27, 6.4, 50)]
    w = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    r = Rules(effort=sys.argv[1] if len(sys.argv) > 1 else "fast", circ_weight=w)
    t = time.time()
    res = search(site, cores, ramps=ramp, rules=r, log=print)
    b = res[0]
    st = b["stats"]
    print({k: st[k] for k in ("total_legal", "per_floor", "phase", "circulation", "spans", "columns", "score")})
    for c in st["checks"]:
        print(" ", c)
    out = sys.argv[3] if len(sys.argv) > 3 else os.path.join(os.environ.get("OUT", "out"), "t6")
    os.makedirs(out, exist_ok=True)
    export.plan_png(b, os.path.join(out, "B1.png"), 0)
    export.write_dxf(b, os.path.join(out, "plan.dxf"), unit_scale=1000, insunits=4)
    export.write_3dm(b, os.path.join(out, "model.3dm"), unit_scale=1000, unit="mm")
    print("time", round(time.time() - t, 1))
