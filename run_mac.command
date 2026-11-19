#!/bin/bash
# 맥: 이 파일을 더블클릭하면 실행됩니다. (처음 한 번은 오른쪽 클릭 → 열기)
cd "$(dirname "$0")"
if [ -f .venv/installed.txt ] && ! .venv/bin/python -c "import parking_ai.app, rhino3dm" 2>/dev/null; then
  echo "이 Mac에 맞게 다시 설치합니다."; rm -rf .venv
fi
if [ ! -f .venv/installed.txt ]; then
  echo "[처음 한 번] 필요한 프로그램을 설치합니다. 3~5분 걸릴 수 있습니다."
  if ! command -v python3 >/dev/null; then
    echo "파이썬이 없습니다. https://www.python.org/downloads/ 에서 Python 3.12 를 설치한 뒤 다시 실행하세요."
    read -p "엔터를 누르면 닫힙니다"; exit 1
  fi
  python3 -m venv .venv && .venv/bin/python -m pip install --upgrade pip && .venv/bin/python -m pip install -r requirements.txt \
    && echo ok > .venv/installed.txt || { echo "설치 중 문제가 생겼습니다. 위 메시지를 캡처해 보내 주세요."; read -p ""; exit 1; }
fi
echo "브라우저에 화면이 열립니다. 이 창을 닫으면 프로그램이 꺼집니다."
.venv/bin/python -m parking_ai "$@"
