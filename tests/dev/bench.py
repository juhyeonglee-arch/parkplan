"""효율 비교(동선 가중치 0 = 대수만): 설정 끄고 켜기."""
import sys, time
from shapely.geometry import Polygon, box
from parking_ai.engine import search
from parking_ai.rules import Rules
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from t8_user_site import SITE as USITE, CORES as UCORES, RAMP as URAMP

CASES = {
    "sample": (Polygon([(0, 0), (70, 0), (76, 30), (68, 50), (0, 50)]), [box(20, 20, 28, 30), box(48, 8, 55, 16)], [box(0, 27, 6.4, 50)], {}),
    "user": (USITE, UCORES, URAMP, {}),
    "big": (Polygon([(0, 0), (120, 0), (130, 40), (115, 80), (0, 80)]), [box(30, 30, 40, 42), box(80, 50, 90, 60), box(95, 10, 103, 18)], [box(0, 50, 6.4, 80)], {}),
    "auto": (Polygon([(0, 0), (120, 0), (130, 40), (115, 80), (0, 80)]), [box(30, 30, 40, 42), box(80, 50, 90, 60), box(95, 10, 103, 18)], [], dict(ramp_mode="auto")),
}
if __name__ == "__main__":
    confs = {"v1.2": dict(wall_search=False), "v1.3": {}}
    for name, (site, cores, ramps, kw) in CASES.items():
        line = [name]
        for cn, ck in confs.items():
            t = time.time()
            r = Rules(effort="normal", circ_weight=0, floors=1, **kw, **ck)
            res = search(site, cores, ramps=ramps, rules=r, log=lambda m: None)
            line.append(f"{cn}={res[0]['stats']['total_legal']}({time.time() - t:.0f}s)")
        print(" ".join(line), flush=True)
