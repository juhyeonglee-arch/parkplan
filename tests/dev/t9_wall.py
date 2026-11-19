"""벽 따라 한 줄(wall_rows) 시험: 사용자 대지 근사, 세로 차로(격자 90°)."""
import os, sys, time, dataclasses
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from t8_user_site import SITE, CORES, RAMP
from parking_ai.engine import search
from parking_ai.rules import Rules
from parking_ai import export

if __name__ == "__main__":
    out = os.path.join(os.environ.get("OUT", "out"), "t9")
    os.makedirs(out, exist_ok=True)
    w = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    for walls in [(), ("lo",), ("hi",), ("lo", "hi")]:
        t = time.time()
        r = Rules(effort="normal", circ_weight=w, floors=1, grid_angles=(90.0,), wall_rows=walls)
        res = search(SITE, CORES, ramps=RAMP, rules=r, log=lambda m: None)
        b = res[0]; st = b["stats"]
        tag = "_".join(walls) or "base"
        print(tag, st["total_legal"], st["phase"], st["area_per_stall"], "dead", st["circulation"]["dead_ends"],
              round(time.time() - t, 1), "s", flush=True)
        export.plan_png(b, os.path.join(out, f"{tag}.png"), 0)
