"""브라우저로 화면을 직접 눌러 보는 시험(Playwright가 있을 때만). 화면 캡처를 남긴다."""
import os

import pytest

from test_server import server  # noqa: F401  (같은 서버 고정물 사용)

sync_api = pytest.importorskip("playwright.sync_api")


def test_gui_sample_flow(server, art):  # noqa: F811
    errors = []
    with sync_api.sync_playwright() as p:
        exe = os.environ.get("PW_CHROMIUM")
        b = p.chromium.launch(executable_path=exe) if exe else p.chromium.launch()
        pg = b.new_page(viewport={"width": 1440, "height": 900}, device_scale_factor=1)
        pg.on("pageerror", lambda e: errors.append(str(e)))
        pg.goto(server + "/")
        pg.click("#sample")
        pg.wait_for_selector("#fileInfo:not([hidden])")
        pg.click("#effort button[data-v=fast]")
        pg.screenshot(path=os.path.join(art, "화면_1_입력.png"))
        pg.click("#go")
        pg.wait_for_selector("#stats:not([hidden])", timeout=900_000)
        pg.wait_for_timeout(500)
        pg.screenshot(path=os.path.join(art, "화면_2_결과.png"))
        b.close()
    assert not errors, errors
