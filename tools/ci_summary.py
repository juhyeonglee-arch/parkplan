"""시험 요약(summary.json) → 마크다운 표. 사용: python tools/ci_summary.py 폴더 [그림 주소 앞부분]"""
import json
import os
import sys

NAMES = {"bench_sample": "예시 대지", "bench_user": "사용자 대지 근사", "bench_big": "큰 대지", "bench_auto": "자동 경사로",
         "zones_sample": "구역별 방향(예시, 2개 층)", "huge": "초대형 대지(6.4만㎡)", "oneway_big": "일방통행(큰 대지)"}


def main():
    folder = sys.argv[1]
    base = sys.argv[2] if len(sys.argv) > 2 else None
    p = os.path.join(folder, "summary.json")
    d = json.load(open(p, encoding="utf-8")) if os.path.exists(p) else {}
    lines = ["| 시험 | 대수 | 기준 | 시간 | 결과 |", "|---|---:|---:|---:|:---:|"]
    for k, v in d.items():
        if "total" in v:
            ok = "✅" if v["total"] >= v.get("baseline", 0) else "❌"
            sec = f"{v['sec']}초" if "sec" in v else ""
            lines.append(f"| {NAMES.get(k, k)} | {v['total']} | {v.get('baseline', '')} | {sec} | {ok} |")
        else:
            lines.append(f"| {NAMES.get(k, k)} | " + ", ".join(f"{a} {b}" for a, b in v.items()) + " | | | ✅ |")
    if base:
        pngs = sorted(f for f in os.listdir(folder) if f.endswith(".png"))
        pick = [f for f in pngs if f.startswith(("화면_2", "bench_", "zones_", "huge"))]
        if pick:
            lines += ["", "<details open><summary>확인 그림</summary>", ""]
            for f in pick:
                lines.append(f"**{f[:-4]}**<br><img src=\"{base}/{f}\" width=\"640\">")
                lines.append("")
            lines.append("</details>")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
