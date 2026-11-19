import json
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ART = os.path.abspath(os.environ.get("ARTIFACT_DIR", os.path.join(ROOT, "test-artifacts")))
os.makedirs(ART, exist_ok=True)


@pytest.fixture
def art():
    """시험 결과 그림·파일을 모아 두는 폴더(Actions 산출물로 올라감)."""
    return ART


def record(name, **values):
    """요약표(summary.json)에 한 줄 남긴다. PR 설명과 Actions 요약에 쓰인다."""
    p = os.path.join(ART, "summary.json")
    d = {}
    if os.path.exists(p):
        with open(p, encoding="utf-8") as f:
            d = json.load(f)
    d[name] = values
    with open(p, "w", encoding="utf-8") as f:
        json.dump(d, f, ensure_ascii=False, indent=1)
