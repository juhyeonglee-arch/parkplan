"""도면 읽기: DXF(캐드) 또는 3dm(라이노)의 닫힌 선을 레이어별 다각형으로.

- 레이어 이름으로 역할을 추정한다(대지경계 / 코어 / 경사로 / 장애물). 화면에서 바꿀 수 있다.
- 단위: 도면 단위 설정을 읽고, 없으면 크기로 추정(대지 폭이 1000 넘으면 mm).
- 내부 계산은 m. scale = 도면 단위 / m (mm 도면이면 1000).
- DWG 는 직접 읽지 못한다. 캐드에서 DXF 로 저장해서 넣는다.
"""
import math
import os
import re

from shapely.geometry import LineString, Polygon
from shapely.ops import polygonize, unary_union
from shapely.validation import make_valid

ROLE_WORDS = {
    "site": ["대지", "경계", "site", "boundary", "외벽", "주차영역", "outline", "옹벽"],
    "core": ["코어", "core", "ev", "elev", "계단", "stair", "승강", "엘리베이터", "샤프트", "shaft"],
    "ramp": ["램프", "경사로", "ramp", "rmp"],
    "obstacle": ["장애물", "obst", "제외", "기계실", "전기실", "설비", "pit", "피트", "dry", "da", "물탱크", "저수조"],
}
ROLE_ORDER = ["ramp", "core", "obstacle", "site"]


def guess_role(name):
    n = name.lower()
    for role in ROLE_ORDER:
        for w in ROLE_WORDS[role]:
            if len(w) <= 3 and w.isascii():
                if re.search(r"(^|[^a-z])" + re.escape(w) + r"([^a-z]|$)", n):
                    return role
            elif w in n:
                return role
    return "none"


def _poly_from_pts(pts):
    if len(pts) < 3:
        return None
    try:
        p = Polygon(pts)
        if not p.is_valid:
            p = make_valid(p)
            if p.geom_type != "Polygon":
                parts = [g for g in getattr(p, "geoms", []) if g.geom_type == "Polygon"]
                if not parts:
                    return None
                p = max(parts, key=lambda g: g.area)
        return p if p.area > 1e-6 else None
    except Exception:
        return None


# ------------------------------------------------------------------ DXF
INSUNITS = {1: ("in", 39.3701), 2: ("ft", 3.28084), 4: ("mm", 1000.0), 5: ("cm", 100.0), 6: ("m", 1.0)}


def read_dxf(path):
    import ezdxf
    from ezdxf import path as ezpath
    from ezdxf import recover
    try:
        doc = ezdxf.readfile(path)
    except Exception:
        doc, _ = recover.readfile(path)
    msp = doc.modelspace()
    layers = {}
    open_lines = {}

    def handle(e, layer_override=None, depth=0):
        t = e.dxftype()
        layer = layer_override if (layer_override and e.dxf.get("layer", "0") == "0") else e.dxf.get("layer", "0")
        if t == "INSERT" and depth < 4:
            try:
                for v in e.virtual_entities():
                    handle(v, layer, depth + 1)
            except Exception:
                pass
            return
        if t not in ("LWPOLYLINE", "POLYLINE", "CIRCLE", "ELLIPSE", "SPLINE", "LINE", "ARC"):
            return
        try:
            pth = ezpath.make_path(e)
            pts = [(v.x, v.y) for v in pth.flattening(0.02 * _unit_hint[0], segments=8)]
        except Exception:
            return
        if len(pts) < 2:
            return
        closed = t in ("CIRCLE",) or getattr(e, "closed", False) or \
            (t == "POLYLINE" and e.is_closed) or math.dist(pts[0], pts[-1]) < 1e-6 * _unit_hint[0] + 1e-9
        if t == "ELLIPSE" or (t == "SPLINE" and math.dist(pts[0], pts[-1]) < 1e-3 * _unit_hint[0]):
            closed = True
        if closed and len(pts) >= 3:
            p = _poly_from_pts(pts)
            if p is not None:
                layers.setdefault(layer, []).append(p)
        else:
            open_lines.setdefault(layer, []).append(LineString(pts))

    ins = doc.header.get("$INSUNITS", 0)
    _unit_hint = [1000.0 if ins == 4 else (100.0 if ins == 5 else 1.0)]
    for e in msp:
        handle(e)
    # 닫히지 않은 선분들로 이루어진 경계는 이어 붙여 면으로
    for layer, lines in open_lines.items():
        try:
            merged = unary_union(lines)
            for p in polygonize(merged):
                if p.area > 1e-6:
                    layers.setdefault(layer, []).append(p)
        except Exception:
            pass
    unit, scale = INSUNITS.get(ins, (None, None))
    return layers, unit, scale, ins


# ------------------------------------------------------------------ 3dm
def read_3dm(path):
    import rhino3dm as rh
    model = rh.File3dm.Read(path)
    if model is None:
        raise ValueError("3dm 파일을 읽지 못했습니다.")
    names = {}
    for i in range(len(model.Layers)):
        L = model.Layers[i]
        names[L.Index] = L.FullPath or L.Name
    layers = {}
    for obj in model.Objects:
        g = obj.Geometry
        lay = names.get(obj.Attributes.LayerIndex, "기본값")
        pts = None
        if isinstance(g, rh.Curve):
            if not g.IsClosed:
                continue
            pts = _curve_pts(rh, g)
        elif isinstance(g, (rh.Hatch,)) if hasattr(rh, "Hatch") else False:
            continue
        if pts:
            p = _poly_from_pts([(x, y) for x, y, z in pts])
            if p is not None:
                layers.setdefault(lay, []).append(p)
    us = model.Settings.ModelUnitSystem
    m = {rh.UnitSystem.Millimeters: ("mm", 1000.0), rh.UnitSystem.Centimeters: ("cm", 100.0),
         rh.UnitSystem.Meters: ("m", 1.0)}
    unit, scale = m.get(us, (None, None))
    return layers, unit, scale, None


def _curve_pts(rh, g):
    if isinstance(g, rh.PolylineCurve):
        return [(g.Point(i).X, g.Point(i).Y, g.Point(i).Z) for i in range(g.PointCount)]
    if isinstance(g, rh.NurbsCurve) and g.Degree == 1:
        return [(g.Points[i].X, g.Points[i].Y, g.Points[i].Z) for i in range(len(g.Points))]
    if isinstance(g, rh.PolyCurve):
        out = []
        for i in range(g.SegmentCount):
            seg = g.SegmentCurve(i)
            sp = _curve_pts(rh, seg)
            if sp:
                out += sp if not out else sp[1:]
        return out
    t0, t1 = g.Domain.T0, g.Domain.T1
    n = 96
    out = []
    for i in range(n + 1):
        p = g.PointAt(t0 + (t1 - t0) * i / n)
        out.append((p.X, p.Y, p.Z))
    return out


# ------------------------------------------------------------------ 공통
def load(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".dxf":
        layers, unit, scale, ins = read_dxf(path)
    elif ext == ".3dm":
        layers, unit, scale, ins = read_3dm(path)
    elif ext == ".dwg":
        raise ValueError("DWG 는 바로 읽을 수 없습니다. 캐드에서 '다른 이름으로 저장 → DXF'로 저장해서 넣어 주세요.")
    else:
        raise ValueError("DXF 또는 3dm 파일을 넣어 주세요.")
    if not layers:
        raise ValueError("닫힌 선(폴리라인·원)이 하나도 없습니다. 대지 경계와 코어를 닫힌 폴리라인으로 그려 주세요.")
    # 단위 추정
    allp = [p for ps in layers.values() for p in ps]
    big = max(allp, key=lambda p: p.area)
    b = big.bounds
    span = max(b[2] - b[0], b[3] - b[1])
    if scale is None:
        if span > 3000:
            unit, scale = "mm", 1000.0
        elif span > 600:
            unit, scale = "cm", 100.0
        else:
            unit, scale = "m", 1.0
    guessed = (unit is not None and span / scale > 3000)
    info = []
    for name, ps in sorted(layers.items(), key=lambda kv: -max(p.area for p in kv[1])):
        info.append(dict(name=name, count=len(ps), area=round(sum(p.area for p in ps) / scale ** 2, 1),
                         role=guess_role(name)))
    # 대지 역할이 없으면 가장 큰 면이 있는 레이어
    if not any(i["role"] == "site" for i in info):
        info[0]["role"] = "site"
    return dict(layers=layers, info=info, unit=unit, scale=scale, warn_scale=guessed)


def pick(src, mapping):
    """mapping: {레이어: 역할} → (site, cores, ramps, obstacles) m 단위."""
    from shapely import affinity
    s = 1.0 / src["scale"]
    sc = lambda g: affinity.scale(g, s, s, origin=(0, 0))
    site_ps, cores, ramps, obs = [], [], [], []
    for name, ps in src["layers"].items():
        role = mapping.get(name, "none")
        if role == "site":
            site_ps += ps
        elif role == "core":
            cores += ps
        elif role == "ramp":
            ramps += ps
        elif role == "obstacle":
            obs += ps
    if not site_ps:
        raise ValueError("대지경계 레이어를 하나 골라 주세요.")
    site = max(site_ps, key=lambda p: p.area)
    # 대지 안의 다른 닫힌 면이 대지 레이어에 같이 있으면(안쪽 선) 무시
    site = sc(Polygon(site.exterior.coords))
    cores = [sc(p) for p in cores]
    ramps = [sc(p) for p in ramps]
    obs = [sc(p) for p in obs]
    inside = lambda p: p.intersects(site)
    return site, [p for p in cores if inside(p)], [p for p in ramps if inside(p)], [p for p in obs if inside(p)]


def write_sample_dxf(path):
    """예시 도면(mm): 꺾인 대지 + 코어 2개 + 그린 경사로."""
    import ezdxf
    doc = ezdxf.new("R2010")
    doc.header["$INSUNITS"] = 4
    for n, c in (("A-대지경계", 7), ("A-코어", 8), ("A-경사로", 5)):
        doc.layers.add(n, color=c)
    msp = doc.modelspace()
    k = 1000
    site = [(0, 0), (70, 0), (76, 30), (68, 50), (0, 50)]
    msp.add_lwpolyline([(x * k, y * k) for x, y in site], close=True, dxfattribs={"layer": "A-대지경계"})
    for x0, y0, x1, y1 in ((20, 20, 28, 30), (48, 8, 55, 16)):
        msp.add_lwpolyline([(x0 * k, y0 * k), (x1 * k, y0 * k), (x1 * k, y1 * k), (x0 * k, y1 * k)], close=True,
                           dxfattribs={"layer": "A-코어"})
    msp.add_lwpolyline([(0, 27 * k), (6.4 * k, 27 * k), (6.4 * k, 50 * k), (0, 50 * k)], close=True,
                       dxfattribs={"layer": "A-경사로"})
    doc.saveas(path)
