"""명령줄로 예시 도면을 넣어 결과 파일이 다 나오는지."""
import os
import subprocess
import sys

from conftest import ROOT

EXAMPLE = os.path.join(ROOT, "examples", "예시_대지.dxf")


def test_cli_example(art):
    out = os.path.join(art, "cli_예시")
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    p = subprocess.run([sys.executable, "-m", "parking_ai", EXAMPLE, "--floors", "2", "--effort", "fast", "--out", out],
                       cwd=ROOT, env=env, capture_output=True, text=True, encoding="utf-8", timeout=900)
    print(p.stdout[-3000:], p.stderr[-3000:])
    assert p.returncode == 0
    names = os.listdir(out)
    for need in ("B1_평면.png", "배치.dxf", "모델.3dm"):
        assert need in names, names
        assert os.path.getsize(os.path.join(out, need)) > 1000
    assert any(n.endswith("_통계.json") for n in names), names
