"""사용자 그림(2026-11-03)에서 어림잡은 대지: 모서리를 깎은 긴 대지 + 위쪽 가로 경사로 + 코어 2개.
그림 픽셀 → m (구획 너비 2.5m ≈ 33px 로 환산). 실제 도면이 아니라 비교용 근사."""
import os, sys, time
from shapely.geometry import Polygon, box
from parking_ai.engine import search
from parking_ai.rules import Rules
from parking_ai import export

K = 13.2
def P(px, py):
    return ((px - 82) / K, (612 - py) / K)

SITE = Polygon([P(130, 125), P(1250, 128), P(1310, 190), P(1306, 535), P(1225, 612), P(170, 612), P(82, 515), P(88, 185)])
CORES = [Polygon([P(282, 237), P(383, 237), P(383, 418), P(282, 418)]), Polygon([P(1013, 236), P(1118, 236), P(1118, 420), P(1013, 420)])]
RAMP = [Polygon([P(755, 137), P(1197, 137), P(1197, 226), P(755, 226)])]

if __name__ == "__main__":
    eff = sys.argv[1] if len(sys.argv) > 1 else "normal"
    w = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    tag = sys.argv[3] if len(sys.argv) > 3 else "now"
    out = os.path.join(os.environ.get("OUT", "out"), "t8")
    os.makedirs(out, exist_ok=True)
    t = time.time()
    res = search(SITE, CORES, ramps=RAMP, rules=Rules(effort=eff, circ_weight=w, floors=1), log=lambda m: None)
    b = res[0]; st = b["stats"]
    print(tag, "site", round(SITE.area), "㎡ B1", st["total_legal"], "대", st["phase"], "㎡/대", st["area_per_stall"],
          "avg", st["circulation"]["avg_drive"], "dead", st["circulation"]["dead_ends"], round(time.time() - t, 1), "s")
    export.plan_png(b, os.path.join(out, f"{tag}.png"), 0)
