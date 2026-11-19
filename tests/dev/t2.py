import sys, time
from shapely.geometry import Polygon, box
from parking_ai.engine import search
from parking_ai.rules import Rules
from parking_ai import export
if __name__ == "__main__":
    site = Polygon([(0,0),(70,0),(76,30),(68,50),(0,50)])
    cores = [box(20,20,28,30), box(48,8,55,16)]
    ramp = [box(0,27,6.4,50)]
    r = Rules(effort=sys.argv[1] if len(sys.argv)>1 else "fast")
    t=time.time()
    res = search(site, cores, ramps=ramp, rules=r)
    print("time", time.time()-t)
    b = res[0]
    export.plan_png(b, "out/b1.png", 0)
    export.plan_png(b, "out/b2.png", 1)
    export.write_dxf(b, "out/t.dxf")
    export.write_3dm(b, "out/t.3dm")
    print(export.write_stats(b, "out", "t"))
