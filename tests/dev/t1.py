import time, sys
from shapely.geometry import Polygon, box
from parking_ai.engine import search
from parking_ai.rules import Rules
site = Polygon([(0,0),(70,0),(76,30),(68,50),(0,50)])
cores = [box(20,20,28,30), box(48,8,55,16)]
ramp = [box(0,27,6.4,50)]
r = Rules(effort=sys.argv[1] if len(sys.argv)>1 else "fast")
t=time.time()
res = search(site, cores, ramps=ramp, rules=r)
b=res[0]
print(b['stats'])
print(time.time()-t)
for x in res: print(x['stats']['ext_mask'], x['rank'], [ (c['item'],c['have'],c['ok']) for c in x['stats']['checks'][:2]])
