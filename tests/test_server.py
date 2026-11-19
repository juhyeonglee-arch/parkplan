"""화면 서버를 실제로 켜서 예시 도면 → 배치 찾기 → 내보내기까지."""
import json
import os
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

import pytest

from conftest import ROOT


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    port = _free_port()
    home = tmp_path_factory.mktemp("home")
    env = dict(os.environ, PYTHONIOENCODING="utf-8", HOME=str(home), USERPROFILE=str(home))
    p = subprocess.Popen([sys.executable, "-m", "parking_ai", "--no-browser", "--port", str(port)], cwd=ROOT, env=env)
    url = f"http://127.0.0.1:{port}"
    for _ in range(60):
        try:
            urllib.request.urlopen(url + "/api/status", timeout=2)
            break
        except Exception:
            time.sleep(0.5)
    yield url
    p.terminate()
    p.wait(10)


def _post(url, data=b"", headers=None):
    req = urllib.request.Request(url, data=data, headers=headers or {}, method="POST")
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.loads(r.read())


def test_page_loads(server):
    with urllib.request.urlopen(server + "/", timeout=10) as r:
        html = r.read().decode("utf-8")
    assert "<html" in html.lower()


def test_sample_run_export(server):
    src = _post(server + "/api/sample")
    mapping = {l["name"]: l["role"] for l in src["layers"]}
    _post(server + "/api/run", json.dumps(dict(id=src["id"], mapping=mapping, rules=dict(effort="fast", floors=1))).encode(),
          {"Content-Type": "application/json"})
    t = time.time()
    while time.time() - t < 900:
        with urllib.request.urlopen(server + "/api/status", timeout=10) as r:
            d = json.loads(r.read())
        if d["state"] in ("done", "error"):
            break
        time.sleep(2)
    assert d["state"] == "done", d.get("error")
    files = _post(server + "/api/export", b'{"alt": 0}', {"Content-Type": "application/json"})["files"]
    assert files


def test_server_mode(tmp_path):
    """웹 배포 모드: 모든 주소에서 받고, 폴더 열기는 막는다."""
    port = _free_port()
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PARKPLAN_SERVER="1", PARKPLAN_HOME=str(tmp_path), PORT=str(port))
    p = subprocess.Popen([sys.executable, "-m", "parking_ai", "--no-browser", "--host", "127.0.0.1"], cwd=ROOT, env=env)
    url = f"http://127.0.0.1:{port}"
    try:
        for _ in range(60):
            try:
                with urllib.request.urlopen(url + "/api/status", timeout=2) as r:
                    st = json.loads(r.read())
                break
            except Exception:
                time.sleep(0.5)
        assert st["server"] is True
        with pytest.raises(urllib.error.HTTPError):
            _post(url + "/api/openpath", json.dumps({"p": str(tmp_path)}).encode(), {"Content-Type": "application/json"})
    finally:
        p.terminate()
        p.wait(10)
