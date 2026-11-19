"""구역별 차로 방향 시험."""
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench import CASES
from parking_ai.engine import search
from parking_ai.rules import Rules
from parking_ai import zones, export, critique

OUT = os.path.join(os.environ.get("OUT", "out"), "t12")
if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    names = sys.argv[1:] or ["sample", "user", "big"]
    for name in names:
        site, cores, ramps, kw = CASES[name]
        r = Rules(effort="normal", circ_weight=1.0, floors=2, **kw)
        t = time.time()
        fin = search(site, cores, ramps=ramps, rules=r, log=lambda m: None)
        t1 = time.time() - t
        z = zones.search_zones(site, cores, ramps, [], r, base_total=fin[0]["stats"]["total_legal"], log=print)
        print(name, "base", fin[0]["stats"]["total_legal"], f"{t1:.0f}s", "zones",
              z and z["stats"]["total_legal"], z and z["stats"]["zone_text"], f"{time.time() - t - t1:.0f}s", flush=True)
        if z:
            critique.annotate(fin + [z])
            export.plan_png(z, os.path.join(OUT, f"{name}_zones.png"), 0)
            export.write_dxf(z, os.path.join(OUT, f"{name}_zones.dxf"))
            export.write_3dm(z, os.path.join(OUT, f"{name}_zones.3dm"))
            export.to_json(z)
            print("  feedback:", [f["title"] for f in z["stats"]["feedback"]])
            print("  checks:", [(c["item"], c["ok"]) for c in z["stats"]["checks"]])
