"""초대형 대지(6.4만㎡, 코어 10, 경사로 3) 속도·대수 시험. RUN_HUGE=1 일 때만."""
import os
import time

import pytest

from conftest import record
from sites import HUGE_CORES, HUGE_RAMP, HUGE_SITE
from parking_ai import export
from parking_ai.engine import search
from parking_ai.rules import Rules

pytestmark = pytest.mark.skipif(not os.environ.get("RUN_HUGE"), reason="RUN_HUGE=1 일 때만")


def test_huge(art):
    t = time.time()
    res = search(HUGE_SITE, HUGE_CORES, ramps=HUGE_RAMP, rules=Rules(effort="fast", floors=1), log=lambda m: None)
    total = res[0]["stats"]["total_legal"]
    sec = round(time.time() - t, 1)
    export.plan_png(res[0], os.path.join(art, "huge.png"), 0, title=f"huge: {total}대")
    record("huge", total=total, baseline=2410, sec=sec)
    assert total >= 2410
    assert sec < 900
