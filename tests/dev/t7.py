"""v1.1 시험: 큰 대지(3개 층), 60° 주차, 자동 경사로."""
import os, sys, time
from shapely.geometry import Polygon, box
from parking_ai.engine import search
from parking_ai.rules import Rules
from parking_ai import export

if __name__ == "__main__":
    out = os.path.join(os.environ.get("OUT", "out"), "t7")
    os.makedirs(out, exist_ok=True)
    site = Polygon([(0, 0), (120, 0), (130, 40), (115, 80), (0, 80)])
    cores = [box(30, 30, 40, 42), box(80, 50, 90, 60), box(95, 10, 103, 18)]
    ramp = [box(0, 50, 6.4, 80)]
    for name, kw, rp in (("big", dict(floors=3), ramp), ("a60", dict(park_angles=(60,)), ramp), ("auto", dict(ramp_mode="auto"), [])):
        t = time.time()
        r = Rules(effort="fast", **kw)
        res = search(site, cores, ramps=rp, rules=r, log=lambda m: None)
        b = res[0]; st = b["stats"]
        print(name, st["total_legal"], st["phase"], st["circulation"]["avg_drive"], st["circulation"]["dead_ends"],
              st["columns"], [c["item"] for c in st["checks"] if not c["ok"]], round(time.time() - t, 1))
        export.plan_png(b, os.path.join(out, name + ".png"), 0)
        export.write_dxf(b, os.path.join(out, name + ".dxf"), unit_scale=1000, insunits=4)
        export.write_3dm(b, os.path.join(out, name + ".3dm"), unit_scale=1000, unit="mm")
        export.write_stats(b, out, name)
