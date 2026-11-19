import sys, time
from shapely.geometry import Polygon, box
from shapely import affinity
from parking_ai.engine import search
from parking_ai.rules import Rules
from parking_ai import export
if __name__ == "__main__":
    site = Polygon([(0,0),(70,0),(76,30),(68,50),(0,50)])
    cores = [box(20,20,28,30), box(48,8,55,16)]
    rot = lambda g: affinity.rotate(g, 23, origin=(0,0))
    site = rot(site); cores=[rot(c) for c in cores]
    r = Rules(effort="fast", ramp_mode="auto", park_angles=(90,60,45))
    t=time.time()
    res = search(site, cores, rules=r)
    b = res[0]
    export.plan_png(b, "out/auto.png", 0)
    for x in res:
        if x['stats']['park_angle']!=90:
            export.plan_png(x, "out/angled.png", 0); break
