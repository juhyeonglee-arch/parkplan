from shapely.geometry import Polygon, box
from parking_ai.engine import search
from parking_ai.rules import Rules
from parking_ai import export
if __name__ == "__main__":
    site = Polygon([(0,0),(70,0),(76,30),(68,50),(0,50)])
    cores = [box(20,20,28,30), box(48,8,55,16)]
    for a in (60,45):
        r = Rules(effort="fast", ramp_mode="none", park_angles=(a,), two_entrances=True)
        res = search(site, cores, rules=r, log=lambda *x: None)
        print(a, res[0]['stats']['per_floor'])
        export.plan_png(res[0], f"out/ang{a}.png", 0)
