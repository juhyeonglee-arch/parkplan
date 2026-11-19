"""일방통행 흐름 + 경사로 모양 검사 시험."""
import os, sys, time, math
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from shapely.geometry import Polygon
from bench import CASES
from parking_ai.engine import search
from parking_ai.rules import Rules
from parking_ai import export

OUT = os.path.join(os.environ.get("OUT", "out"), "t13")
if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    site, cores, ramps, _ = CASES["big"]
    for ang in (60, 45):
        for two in (False, True):
            r = Rules(effort="fast", floors=1, park_angles=(ang,), two_entrances=two, zone_search=False)
            t = time.time()
            fin = search(site, cores, ramps=ramps, rules=r, log=lambda m: None)
            b = fin[0]
            print(ang, "일방" if two else "양방", b["stats"]["total_legal"], f"{time.time() - t:.0f}s",
                  [(c["item"], c["have"], c["ok"]) for c in b["stats"]["checks"] if c["item"] in ("일방통행 흐름",) or c.get("ramp")],
                  "arrows", len(b["arrows"]), flush=True)
            export.plan_png(b, os.path.join(OUT, f"a{ang}_{int(two)}.png"), 0)
            export.write_dxf(b, os.path.join(OUT, f"a{ang}_{int(two)}.dxf"))
            export.to_json(b)
    # 곡선 경사로(내변 4.5m — 반경 미달)
    ri, ro = 4.5, 8.2
    pts = [(ri * math.cos(a), ri * math.sin(a)) for a in [i * math.pi / 2 / 12 for i in range(13)]]
    pts += [(ro * math.cos(a), ro * math.sin(a)) for a in [i * math.pi / 2 / 12 for i in range(12, -1, -1)]]
    curve = Polygon([(x + 10, y + 10) for x, y in pts])
    r = Rules(effort="fast", floors=2, zone_search=False)
    fin = search(site, cores, ramps=[curve], rules=r, log=lambda m: None)
    print("curve", fin[0]["stats"]["total_legal"], fin[0]["rank"][0],
          [(c["item"], c["need"], c["have"], c["ok"]) for c in fin[0]["stats"]["checks"] if c.get("ramp")])
