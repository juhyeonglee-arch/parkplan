"""브라우저 화면: 도면을 넣고 레이어 역할을 고른 뒤 '배치 찾기'.

python -m parking_ai   → http://127.0.0.1:8766 이 열린다. 파이썬 표준 http.server 만 쓴다.
"""
import json
import mimetypes
import os
import re
import subprocess
import sys
import threading
import time
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse

from . import VERSION, engine, export, io_input
from .rules import Rules

WEB = os.path.join(os.path.dirname(__file__), "web")
# 웹 서버로 배포할 때(PARKPLAN_SERVER=1): 모든 주소에서 받고, 결과는 PARKPLAN_HOME 에 두고, '폴더 열기'는 끈다.
SERVER_MODE = bool(os.environ.get("PARKPLAN_SERVER"))
HOME = os.environ.get("PARKPLAN_HOME") or os.path.join(os.path.expanduser("~"), "지하주차장")
UPLOADS = os.path.join(HOME, "_입력")

SOURCES = {}      # id → dict(path, src)
JOB = {"state": "idle", "log": [], "progress": None, "results": None, "error": None, "stop": False,
       "out": None, "files": [], "src": None, "started": 0}
LOCK = threading.Lock()


def _cleanup_old(hours=6):
    """서버 모드: 오래된 입력·결과 폴더를 지워 디스크를 비운다."""
    import shutil
    lim = time.time() - hours * 3600
    for root in (HOME, UPLOADS):
        if not os.path.isdir(root):
            continue
        for n in os.listdir(root):
            p = os.path.join(root, n)
            if p == UPLOADS:
                continue
            try:
                if os.path.getmtime(p) < lim:
                    shutil.rmtree(p) if os.path.isdir(p) else os.remove(p)
            except OSError:
                pass


def _log(msg):
    with LOCK:
        JOB["log"].append(msg)


def _geo_preview(src):
    """입력 레이어 미리보기(m 단위, 간략화)."""
    s = 1.0 / src["scale"]
    out = []
    for name, ps in src["layers"].items():
        polys = []
        for p in ps[:400]:
            q = p.simplify(0.05 / s if s else 0.05)
            polys.append([[round(x * s, 3), round(y * s, 3)] for x, y in q.exterior.coords])
        out.append(dict(name=name, polys=polys))
    return out


def open_source(path):
    src = io_input.load(path)
    sid = re.sub(r"\W+", "_", os.path.splitext(os.path.basename(path))[0])[:40] + f"_{int(time.time())}"
    SOURCES[sid] = dict(path=path, src=src)
    return dict(id=sid, file=os.path.basename(path), unit=src["unit"], scale=src["scale"],
                layers=src["info"], geo=_geo_preview(src), warn_scale=src["warn_scale"])


def _run_job(sid, mapping, rules_d, unit):
    try:
        S = SOURCES[sid]
        src = S["src"]
        if unit and unit in ("mm", "cm", "m"):
            src["scale"] = {"mm": 1000.0, "cm": 100.0, "m": 1.0}[unit]
            src["unit"] = unit
        site, cores, ramps, obs = io_input.pick(src, mapping)
        rules = Rules.from_dict(rules_d)
        _log(f"대지 {site.area:,.0f}㎡ · 코어 {len(cores)}개 · 경사로 {len(ramps)}개 · 장애물 {len(obs)}개")
        if rules.ramp_mode == "drawn" and not ramps:
            _log("그린 경사로가 없어 경사로 없이 찾습니다. (경사로 레이어를 고르거나 '자동'을 쓰세요)")
            rules.ramp_mode = "none"

        def prog(stage, i, n):
            with LOCK:
                JOB["progress"] = dict(stage=stage, i=i, n=n)

        with LOCK:
            JOB["inputs"] = (site, cores, ramps, obs, rules)
        finals = engine.search(site, cores, ramps, obs, rules, log=_log, stop=lambda: JOB["stop"], progress=prog)
        if not finals:
            raise RuntimeError("멈췄거나 결과가 없습니다.")
        # 같은 결과는 하나로
        uniq, seen = [], set()
        for r in finals:
            st = r["stats"]
            sig = (st["total_legal"], st["grid_angle"], st["park_angle"], tuple(st["spans"]),
                   tuple(p["legal"] for p in st["per_floor"]), st["columns"])
            if sig not in seen:
                seen.add(sig)
                uniq.append(r)
        # 피드백 → 고칠 수 있는 것을 넣은 대안
        from . import critique
        try:
            alts = critique.improve(site, cores, ramps, obs, rules, uniq, log=_log, stop=lambda: JOB["stop"])
            uniq = uniq + alts
        except Exception:
            traceback.print_exc()
        # 구역별 차로 방향: 같이 쓰는 차로로 나눠 구역마다 방향을 따로
        if rules.zone_search and not JOB["stop"]:
            from . import zones
            try:
                _log("구역별 차로 방향: 대지를 같이 쓰는 차로로 나눠 구역마다 방향을 따로 찾는 중")
                z = zones.search_zones(site, cores, ramps, obs, rules, base_total=uniq[0]["stats"]["total_legal"],
                                       log=_log, stop=lambda: JOB["stop"])
                if z is not None:
                    uniq.append(z)
            except Exception:
                traceback.print_exc()
        try:
            critique.annotate(uniq)
        except Exception:
            traceback.print_exc()
        name = re.sub(r"\W+", "_", os.path.splitext(os.path.basename(S["path"]))[0])[:30]
        out = os.path.join(HOME, f"{name}_{time.strftime('%Y%m%d_%H%M%S')}")
        os.makedirs(out, exist_ok=True)
        with LOCK:
            JOB["results"] = uniq
            JOB["out"] = out
            JOB["unit"] = src["unit"]
            JOB["scale"] = src["scale"]
            JOB["json"] = [export.to_json(r) for r in uniq]
        files = do_export(0)
        with LOCK:
            JOB["files"] = files
            JOB["state"] = "done"
    except Exception as e:
        traceback.print_exc()
        with LOCK:
            JOB["state"] = "error"
            JOB["error"] = str(e)


LIVE = {"gen": 0, "ex": None}


def live_run(alt, ramp_polys):
    """경사로를 옮긴 자리로 빠르게 다시 찾기. 새 요청이 오면 이전 계산은 멈춘다."""
    import dataclasses
    from shapely.geometry import Polygon
    with LOCK:
        LIVE["gen"] += 1
        gen = LIVE["gen"]
        if JOB.get("inputs") is None or not JOB.get("results"):
            raise RuntimeError("먼저 '배치 찾기'를 한 번 해 주세요.")
        site, cores, _, obs, rules = JOB["inputs"]
        base = JOB["results"][alt]
    ramps = [Polygon(p) for p in ramp_polys if len(p) >= 3]
    for rp in ramps:
        if not site.buffer(0.05).contains(rp):
            raise ValueError("경사로가 대지 밖으로 나갔습니다.")
    st = base["stats"]
    if st.get("zones"):
        raise RuntimeError("구역별 방향 안은 경사로를 끌어 다시 찾을 수 없습니다. 다른 안을 고르거나 '배치 찾기'를 다시 하세요.")
    r = dataclasses.replace(rules, effort="live", grid_angles=(st["grid_angle"],), park_angles=(st["park_angle"],),
                            wall_rows=tuple(st.get("walls") or ()),
                            ramp_mode="drawn" if ramps else "none")
    _live_executor()
    t0 = time.time()
    finals = engine.search(site, cores, ramps, obs, r, log=lambda m: None, stop=lambda: LIVE["gen"] != gen,
                           executor=LIVE["ex"])
    if LIVE["gen"] != gen:
        return None
    if not finals:
        raise RuntimeError("이 자리에서는 배치를 찾지 못했습니다.")
    res = finals[0]
    res["rules"] = dataclasses.replace(res["rules"], effort=rules.effort)
    res["label"] = "경사로이동"
    res["stats"]["label"] = "경사로이동"
    res["stats"]["tab"] = "경사로 이동"
    try:
        from . import critique
        res["stats"]["feedback"] = critique.critique(res)
        res["stats"]["wall_text"] = critique.wall_text(res)
    except Exception:
        traceback.print_exc()
    with LOCK:
        results = [x for x in JOB["results"] if x.get("label") != "경사로이동"]
        results.append(res)
        JOB["results"] = results
        JOB["json"] = [x for x in JOB["json"][:len(results) - 1]] + [export.to_json(res)]
        idx = len(results) - 1
    return dict(alt=idx, result=export.to_json(res), seconds=round(time.time() - t0, 1),
                base_total=st["total_legal"])


SCAN = {"state": "idle", "i": 0, "n": 0, "results": [], "error": None, "gen": 0, "ramp": 0}


def _live_executor():
    if LIVE["ex"] is None:
        import os
        LIVE["ex"] = engine._executor(max(1, min(8, (os.cpu_count() or 2))))
    return LIVE["ex"]


def ramp_scan(alt, idx, ramp_polys=None):
    """고른 안의 격자·주차 각도 그대로, idx 번 경사로를 대지 둘레를 따라 옮겨 가며 대수 비교."""
    import dataclasses
    from . import rampscan
    with LOCK:
        if JOB.get("inputs") is None or not JOB.get("results"):
            raise RuntimeError("먼저 '배치 찾기'를 한 번 해 주세요.")
        site, cores, ramps, obs, rules = JOB["inputs"]
        base = JOB["results"][alt]
        if base["stats"].get("zones"):
            raise RuntimeError("구역별 방향 안에서는 경사로 자리 추천을 쓸 수 없습니다. 다른 안을 고르세요.")
        SCAN["gen"] += 1
        gen = SCAN["gen"]
        SCAN.update(state="running", i=0, n=0, results=[], error=None, ramp=idx)
    if ramp_polys:
        from shapely.geometry import Polygon
        ramps = [Polygon(p) for p in ramp_polys if len(p) >= 3]
    if not ramps or idx >= len(ramps):
        with LOCK:
            SCAN.update(state="error", error="도면에 그린 경사로가 있어야 자리를 추천할 수 있습니다.")
        raise RuntimeError(SCAN["error"])
    st = base["stats"]
    if st.get("zones"):
        raise RuntimeError("구역별 방향 안은 경사로를 끌어 다시 찾을 수 없습니다. 다른 안을 고르거나 '배치 찾기'를 다시 하세요.")
    r = dataclasses.replace(rules, effort="live", grid_angles=(st["grid_angle"],), park_angles=(st["park_angle"],),
                            wall_rows=tuple(st.get("walls") or ()), ramp_mode="drawn", circ_weight=0.0)
    # 자리 비교는 대수만(동선 다듬기는 자리를 고른 뒤 실시간 이동에서) — 큰 대지에서 5배 빠름

    def prog(i, n):
        with LOCK:
            if SCAN["gen"] == gen:
                SCAN["i"], SCAN["n"] = i, n

    def work():
        try:
            out = rampscan.scan(site, cores, list(ramps), obs, r, idx, executor=_live_executor(), progress=prog,
                                stop=lambda: SCAN["gen"] != gen)
            with LOCK:
                if SCAN["gen"] == gen:
                    SCAN.update(state="done", results=out)
        except Exception as e:
            traceback.print_exc()
            with LOCK:
                SCAN.update(state="error", error=str(e))
    threading.Thread(target=work, daemon=True).start()


def do_export(alt):
    res = JOB["results"][alt]
    out = JOB["out"]
    base = res.get("label") or f"안{alt + 1}"
    scale = JOB.get("scale", 1.0)
    unit = JOB.get("unit", "m")
    ins = {"mm": 4, "cm": 5, "m": 6}.get(unit, 6)
    files = []
    p = os.path.join(out, f"{base}_B1_평면.png"); export.plan_png(res, p, 0); files.append(p)
    if res["stats"]["floors"] > 1:
        p = os.path.join(out, f"{base}_B2_평면.png"); export.plan_png(res, p, 1, title="B2~ 주차 배치"); files.append(p)
    p = os.path.join(out, f"{base}_배치.dxf"); export.write_dxf(res, p, unit_scale=scale, insunits=ins); files.append(p)
    p = os.path.join(out, f"{base}_모델.3dm"); export.write_3dm(res, p, unit_scale=scale, unit=unit); files.append(p)
    export.write_stats(res, out, base)
    files += [os.path.join(out, f"{base}_층별대수.csv"), os.path.join(out, f"{base}_요약.txt"),
              os.path.join(out, f"{base}_통계.json")]
    return [dict(name=os.path.basename(f), path=f, size=os.path.getsize(f)) for f in files]


def _open_path(p):
    if sys.platform.startswith("win"):
        os.startfile(p)  # noqa
    elif sys.platform == "darwin":
        subprocess.Popen(["open", p])
    else:
        subprocess.Popen(["xdg-open", p])


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        b = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def _body(self):
        n = int(self.headers.get("Content-Length", 0))
        return self.rfile.read(n) if n else b""

    def do_GET(self):
        u = urlparse(self.path)
        if u.path in ("/", "/index.html"):
            return self._static(os.path.join(WEB, "index.html"))
        if u.path.startswith("/vendor/"):
            p = os.path.normpath(os.path.join(WEB, u.path.lstrip("/")))
            if p.startswith(WEB):
                return self._static(p)
        if u.path == "/api/status":
            with LOCK:
                d = dict(state=JOB["state"], log=JOB["log"][-200:], progress=JOB["progress"], error=JOB["error"],
                         out=JOB["out"], files=JOB["files"], version=VERSION, server=SERVER_MODE,
                         elapsed=round(time.time() - JOB["started"], 1) if JOB["started"] else 0)
                q = parse_qs(u.query)
                if JOB["state"] == "done" and q.get("full"):
                    d["results"] = JOB["json"]
            return self._json(d)
        if u.path == "/api/ramp_scan":
            with LOCK:
                d = dict(state=SCAN["state"], i=SCAN["i"], n=SCAN["n"], error=SCAN["error"], ramp=SCAN["ramp"],
                         results=SCAN["results"] if SCAN["state"] == "done" else [])
            return self._json(d)
        if u.path == "/files":
            q = parse_qs(u.query)
            p = os.path.normpath(q.get("p", [""])[0])
            if p.startswith(os.path.normpath(HOME)) and os.path.isfile(p):
                return self._static(p, download=q.get("dl") is not None)
        self.send_error(404)

    def _static(self, p, download=False):
        if not os.path.isfile(p):
            return self.send_error(404)
        ctype = mimetypes.guess_type(p)[0] or "application/octet-stream"
        if p.endswith(".js"):
            ctype = "text/javascript"
        with open(p, "rb") as f:
            b = f.read()
        self.send_response(200)
        self.send_header("Content-Type", ctype + ("; charset=utf-8" if ctype.startswith("text") else ""))
        self.send_header("Content-Length", str(len(b)))
        if download:
            from urllib.parse import quote
            self.send_header("Content-Disposition", "attachment; filename*=UTF-8''" + quote(os.path.basename(p)))
        self.end_headers()
        self.wfile.write(b)

    def do_POST(self):
        u = urlparse(self.path)
        try:
            if u.path == "/api/open":
                name = unquote(self.headers.get("X-Name", "drawing.dxf"))
                name = os.path.basename(name)
                os.makedirs(UPLOADS, exist_ok=True)
                p = os.path.join(UPLOADS, f"{int(time.time())}_{name}")
                with open(p, "wb") as f:
                    f.write(self._body())
                return self._json(open_source(p))
            if u.path == "/api/sample":
                os.makedirs(UPLOADS, exist_ok=True)
                p = os.path.join(UPLOADS, "예시_대지.dxf")
                io_input.write_sample_dxf(p)
                return self._json(open_source(p))
            if u.path == "/api/run":
                d = json.loads(self._body() or b"{}")
                if SERVER_MODE:
                    _cleanup_old()
                with LOCK:
                    if JOB["state"] == "running":
                        return self._json(dict(error="이미 찾는 중입니다."), 409)
                    JOB.update(state="running", log=[], progress=None, results=None, error=None, stop=False,
                               out=None, files=[], started=time.time())
                threading.Thread(target=_run_job, args=(d["id"], d.get("mapping", {}), d.get("rules", {}),
                                                        d.get("unit")), daemon=True).start()
                return self._json(dict(ok=True))
            if u.path == "/api/stop":
                JOB["stop"] = True
                return self._json(dict(ok=True))
            if u.path == "/api/live":
                d = json.loads(self._body() or b"{}")
                r = live_run(int(d.get("alt", 0)), d.get("ramps", []))
                if r is None:
                    return self._json(dict(stale=True))
                return self._json(r)
            if u.path == "/api/ramp_scan":
                d = json.loads(self._body() or b"{}")
                ramp_scan(int(d.get("alt", 0)), int(d.get("ramp", 0)), d.get("ramps"))
                return self._json(dict(ok=True))
            if u.path == "/api/live_export":
                d = json.loads(self._body() or b"{}")
                files = do_export(int(d.get("alt", 0)))
                with LOCK:
                    JOB["files"] = files
                return self._json(dict(files=files))
            if u.path == "/api/export":
                d = json.loads(self._body() or b"{}")
                files = do_export(int(d.get("alt", 0)))
                with LOCK:
                    JOB["files"] = files
                return self._json(dict(files=files))
            if u.path == "/api/openpath":
                if SERVER_MODE:
                    return self._json(dict(error="웹 버전에서는 '받기'로 내려받아 주세요."), 400)
                d = json.loads(self._body() or b"{}")
                p = os.path.normpath(d.get("p", ""))
                if p.startswith(os.path.normpath(HOME)) and os.path.exists(p):
                    _open_path(p)
                    return self._json(dict(ok=True))
                return self._json(dict(error="열 수 없는 경로"), 400)
        except Exception as e:
            traceback.print_exc()
            return self._json(dict(error=str(e)), 400)
        self.send_error(404)


def serve(port=8766, open_browser=True, host="127.0.0.1"):
    os.makedirs(HOME, exist_ok=True)
    for p in range(port, port + 20):
        try:
            srv = ThreadingHTTPServer((host, p), H)
            break
        except OSError:
            continue
    url = f"http://127.0.0.1:{srv.server_address[1]}"
    print(f"지하주차장 배치 v{VERSION} — {url}  (이 창을 닫으면 꺼집니다)")
    if open_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    srv.serve_forever()
