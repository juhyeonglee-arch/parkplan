@echo off
chcp 65001 >nul
cd /d "%~dp0"
title 지하주차장 배치

rem 다른 PC에서 복사해 온 .venv 는 쓸 수 없으므로, 실제로 실행되는지 확인하고 안 되면 새로 만든다
if exist ".venv\installed.txt" (
  ".venv\Scripts\python.exe" -c "import parking_ai.app, rhino3dm" >nul 2>nul && goto run
  echo 이 PC에 맞게 다시 설치합니다.
  rmdir /s /q .venv
)

echo [처음 한 번] 필요한 프로그램을 설치합니다. 3~5분 걸릴 수 있습니다.
where py >nul 2>nul && (py -3 -m venv .venv) || (python -m venv .venv)
if not exist ".venv\Scripts\python.exe" (
  echo.
  echo 파이썬을 찾지 못했습니다. https://www.python.org/downloads/ 에서 Python 3.12 를 설치하고
  echo 설치 첫 화면의 "Add python.exe to PATH" 를 꼭 체크한 뒤 이 파일을 다시 실행하세요.
  pause
  exit /b 1
)
".venv\Scripts\python.exe" -m pip install --upgrade pip
".venv\Scripts\python.exe" -m pip install -r requirements.txt || (
  echo.
  echo 설치 중 문제가 생겼습니다. 위 메시지를 캡처해 보내 주세요.
  pause
  exit /b 1
)
echo ok> ".venv\installed.txt"

:run
echo 브라우저에 화면이 열립니다. 이 검은 창을 닫으면 프로그램이 꺼집니다.
".venv\Scripts\python.exe" -m parking_ai %*
pause
