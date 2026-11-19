"""피드백 + 대안 시험 (사용자 대지 근사)."""
import os, sys, time, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from t8_user_site import SITE, CORES, RAMP
from parking_ai.engine import search
from parking_ai.rules import Rules
from parking_ai import critique, export

if __name__ == "__main__":
    out = os.path.join(os.environ.get("OUT", "out"), "t10")
    os.makedirs(out, exist_ok=True)
    ga = tuple(float(x) for x in sys.argv[1].split(",")) if len(sys.argv) > 1 and sys.argv[1] else ()
    t = time.time()
    r = Rules(effort="normal", circ_weight=1.0, floors=1, grid_angles=ga)
    fin = search(SITE, CORES, ramps=RAMP, rules=r, log=lambda m: None)
    print("base", fin[0]["stats"]["total_legal"], round(time.time() - t, 1), "s", flush=True)
    alts = critique.improve(SITE, CORES, RAMP, [], r, fin, log=print)
    critique.annotate(fin + alts)
    for it in fin[0]["stats"]["feedback"]:
        print(" -", it["level"], it["title"], "|", it.get("alts"))
    for i, x in enumerate(fin):
        print("안", i + 1, x["stats"]["total_legal"], x["stats"]["wall_text"])
    for a in alts:
        print(a["stats"]["tab"], a["stats"]["total_legal"])
        for it in a["stats"]["feedback"]:
            print("    -", it["level"], it["title"])
    export.plan_png(fin[0], os.path.join(out, "base.png"), 0)
    for i, a in enumerate(alts):
        export.plan_png(a, os.path.join(out, f"alt{i}.png"), 0)
    print(round(time.time() - t, 1), "s")
