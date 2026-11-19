"""큰 대지 속도 시험: 약 6.3만㎡, 코어 10개, 그린 경사로 3개."""
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from shapely.geometry import Polygon, box
from parking_ai.engine import search
from parking_ai.rules import Rules

SITE = Polygon([(0, 0), (300, 0), (310, 60), (300, 210), (40, 215), (0, 180)])
CORES = [box(30 + 55 * i, 40, 40 + 55 * i, 54) for i in range(5)] + [box(45 + 52 * i, 140, 55 + 52 * i, 154) for i in range(5)]
RAMP = [box(0.5, 90, 6.9, 120), box(150, 0.5, 180, 6.9), box(296, 120, 302.4, 150)]

if __name__ == "__main__":
    eff = sys.argv[1] if len(sys.argv) > 1 else "fast"
    print("area", round(SITE.area))
    t = time.time()
    res = search(SITE, CORES, ramps=RAMP, rules=Rules(effort=eff, floors=1),
                 log=lambda m: print(f"[{time.time() - t:6.1f}s] {m}", flush=True))
    print("best", res[0]["stats"]["total_legal"], "max", max(r["stats"]["total_legal"] for r in res), [(r["stats"]["total_legal"], r["stats"]["score"], r["stats"]["walls"]) for r in res], round(time.time() - t, 1))
