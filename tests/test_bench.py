"""대수 회귀 시험: 1층, 보통, 동선 가중치 0. 기준보다 적으면 실패."""
import os
import time

import pytest

from conftest import record
from sites import BASELINE, CASES
from parking_ai import export
from parking_ai.engine import search
from parking_ai.rules import Rules


@pytest.mark.parametrize("name", list(CASES))
def test_bench(name, art):
    site, cores, ramps, kw = CASES[name]
    t = time.time()
    res = search(site, cores, ramps=ramps, rules=Rules(effort="normal", circ_weight=0, floors=1, **kw), log=lambda m: None)
    total = res[0]["stats"]["total_legal"]
    sec = round(time.time() - t, 1)
    export.plan_png(res[0], os.path.join(art, f"bench_{name}.png"), 0, title=f"{name}: {total}대")
    record(f"bench_{name}", total=total, baseline=BASELINE[name], sec=sec)
    assert total >= BASELINE[name], f"{name}: {total}대 < 기준 {BASELINE[name]}대"
