import time
from shapely.geometry import Polygon, box
from parking_ai.engine import search
from parking_ai.rules import Rules
from parking_ai import export
if __name__=="__main__":
    site=Polygon([(0,0),(118,-12),(130,60),(40,85),(-8,55)])
    cores=[box(30,20,40,32),box(80,30,90,42),box(55,55,63,66)]
    ramp=[box(118.5,10,124.9,33)]
    t=time.time(); res=search(site,cores,ramp,rules=Rules(effort="normal",floors=3),log=lambda *a:None)
    print(time.time()-t, res[0]['stats']['per_floor'], res[0]['stats']['checks'][-1])
    export.plan_png(res[0],"out/big.png",0)
