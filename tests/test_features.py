"""구역별 차로 방향, 일방통행·경사로 검사."""
import math
import os

from shapely.geometry import Polygon

from conftest import record
from sites import CASES
from parking_ai import critique, export, zones
from parking_ai.engine import search
from parking_ai.rules import Rules


def test_zones_sample(art):
    site, cores, ramps, kw = CASES["sample"]
    r = Rules(effort="normal", circ_weight=1.0, floors=2, **kw)
    fin = search(site, cores, ramps=ramps, rules=r, log=lambda m: None)
    z = zones.search_zones(site, cores, ramps, [], r, base_total=fin[0]["stats"]["total_legal"])
    assert z is not None
    critique.annotate(fin + [z])
    export.plan_png(z, os.path.join(art, "zones_sample.png"), 0)
    record("zones_sample", total=z["stats"]["total_legal"], baseline=220)
    assert z["stats"]["total_legal"] >= 220


def test_oneway_big(art):
    site, cores, ramps, _ = CASES["big"]
    got = {}
    for ang in (60, 45):
        for two in (False, True):
            r = Rules(effort="fast", floors=1, park_angles=(ang,), two_entrances=two, zone_search=False)
            b = search(site, cores, ramps=ramps, rules=r, log=lambda m: None)[0]
            got[(ang, two)] = b["stats"]["total_legal"]
            assert b["stats"]["total_legal"] > 0
            if two:
                assert len(b["arrows"]) > 0
            export.plan_png(b, os.path.join(art, f"oneway_{ang}_{'일방' if two else '양방'}.png"), 0)
    record("oneway_big", **{f"{a}_{'일방' if t else '양방'}": v for (a, t), v in got.items()})
    # 일방통행은 양방향보다 많을 수 없다(경사로로 되돌아 나올 수 없는 구획을 뺌)
    assert got[(60, True)] <= got[(60, False)]
    assert got[(45, True)] <= got[(45, False)]


def test_curved_ramp_radius_check():
    site, cores, _, _ = CASES["big"]
    ri, ro = 4.5, 8.2   # 내변 4.5m → 반경 미달이어야 함
    angs = [i * math.pi / 2 / 12 for i in range(13)]
    pts = [(ri * math.cos(a), ri * math.sin(a)) for a in angs] + [(ro * math.cos(a), ro * math.sin(a)) for a in reversed(angs)]
    curve = Polygon([(x + 10, y + 10) for x, y in pts])
    fin = search(site, cores, ramps=[curve], rules=Rules(effort="fast", floors=2, zone_search=False), log=lambda m: None)
    ramp_checks = [c for c in fin[0]["stats"]["checks"] if c.get("ramp")]
    assert ramp_checks
    assert any(not c["ok"] for c in ramp_checks), ramp_checks
