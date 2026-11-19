"""주차장법 기준값과 탐색 설정. 화면에서 모두 바꿀 수 있다.

근거 (주차장법 시행규칙, 2026-07 시행본 기준으로 확인한 것과 확인 못 한 것을 나눠 적음)
- 제3조 주차단위구획: 경형 2.0×3.6, 일반형 2.5×5.0, 확장형 2.6×5.2, 장애인전용 3.3×5.0 (평행주차 외)
- 제6조①14 + 제11조④: 주차대수 50대 이상 부설주차장은 확장형을 구획 총수의 30% 이상
- 제6조①5: 경사로 직선 17%·곡선 14% 이하, 폭 직선 3.3m(2차로 6.0m)·곡선 3.6m(6.5m),
           곡선 내변반경 6m(50대 이하 5m), 차로 높이 2.3m
- 제6조①7: 주차 부분 높이 2.1m
- 시행령 별표1 비고: 경형 전용구획은 전체 구획 수의 10%까지만 법정 대수로 인정
- 장애인전용: 장애인등편의법 시행령 — 총주차대수의 2~4% 범위 지자체 조례 (서울 3%)
- 차로 너비(제6조①3 나목 표): 직각 6.0, 60°대향 4.5(출입구 1개면 5.5), 45°대향 3.5(1개면 5.0)
  → 이 표는 법령 원문과 대조하지 못했으므로 화면에서 확인·수정할 것
"""
from dataclasses import dataclass, field, asdict


@dataclass
class Rules:
    # 주차단위구획 (너비, 길이) m
    stall_w: float = 2.5
    stall_l: float = 5.0
    ext_w: float = 2.6
    ext_l: float = 5.2
    compact_w: float = 2.0
    compact_l: float = 3.6
    acc_w: float = 3.3
    acc_l: float = 5.0

    # 차로 너비 m
    aisle_90: float = 6.0
    aisle_60_one: float = 4.5     # 출입구 2개 이상(일방통행 가능)
    aisle_60_two: float = 5.5     # 출입구 1개
    aisle_45_one: float = 3.5
    aisle_45_two: float = 5.0
    two_entrances: bool = False   # 출입구(경사로)가 2개 이상이면 True
    link_aisle: float = 6.0       # 주차면 없는 연결 차로(회차로) 너비

    # 비율
    ext_ratio: float = 0.30
    ext_threshold: int = 50       # 총 대수가 이 이상이면 확장형 비율 적용
    acc_ratio: float = 0.03
    acc_min_total: int = 10       # 총 대수가 이 이상일 때 장애인 구획을 둠
    compact_legal_ratio: float = 0.10
    compact_fill: bool = True     # 일반 구획이 안 들어가는 자리를 경형으로 채움
    conn_snap: bool = True        # 코어·경사로 옆에 붙인 연결 차로 후보도 보기
    band_dp: bool = True          # 줄·차로 쌓기를 동적계획법으로도 찾기(코어 깊이·대지 끝 남는 폭 활용)
    wall_rows: tuple = ()         # 내부용: 차로 끝 벽을 따라 한 줄("lo"/"hi") — 피드백 대안에서 씀
    wall_search: bool = True      # 차로 끝 벽을 따라 한 줄 두는 조합(없음/한쪽/양쪽)도 함께 찾기
    zone_search: bool = True      # 구역별 차로 방향(같이 쓰는 차로로 두 구역) 대안도 찾기
    edge_fill: bool = True        # 연결 차로 옆·코어·경사로 옆 자투리에 차로에 수직으로 구획 더 넣기

    # 구조 (2단계: 주차 배치를 정한 뒤 기둥을 배열)
    column_w: float = 0.7         # 기둥 크기 — 주차줄 방향(구획 사이에 들어가는 쪽)
    column_d: float = 0.5         # 기둥 크기 — 구획 깊이 방향
    column_inset: float = 0.0     # 차로 쪽 주차선에서 기둥을 안으로 들이는 거리
    span_min: float = 5.0         # 기둥 간격(중심 아님, 기둥 왼쪽 면 사이) 최소
    span_max: float = 9.0         # 기둥 간격 최대(구조 검토 값으로 바꾸기)
    span_step: float = 0.1        # 기둥 위치를 찾는 간격
    stalls_per_bay: tuple = (3,)  # 정형 기둥 간격(대수 × 확장형 너비 + 기둥) — 같은 대수면 이 간격에 가깝게
    bay_pitches: tuple = ()       # (예전 방식 호환용, 쓰지 않음)

    # 동선 (1단계 주차 배치에서 함께 평가)
    circ_weight: float = 1.0      # 0 = 대수만, 1 = 보통, 2 = 동선 중시
    dead_end_ok: float = 6.0      # 이 길이 이하의 막다른 차로 끝은 회차 여유로 봄(구획 1개 깊이 정도)
    flow_radius: float = 3.0      # 도면의 차량 동선 모서리 둥글리기 반지름

    # 전기차 전용구획 (친환경자동차법 시행령: 신축 5%)
    ev_ratio: float = 0.05

    # 대지·층
    wall_offset: float = 0.0      # 입력 경계선에서 안쪽으로 띄울 거리(외벽 두께 등)
    core_clear: float = 0.0       # 코어 둘레 띄움
    floors: int = 2
    floor_h: float = 3.6          # 층고(경사로 길이 계산)
    target_total: int = 0         # 법정 주차대수(넣으면 필요 층수 계산)

    # 경사로
    ramp_mode: str = "drawn"      # drawn(기본, 직접 그린 것) | auto | none
    ramp_slope: float = 0.17
    ramp_lane: float = 3.3
    ramp_two_lane: float = 6.0
    ramp_wall: float = 0.2
    ramp_extra: float = 2.0       # 앞뒤 완화 구간 여유 길이

    # 탐색
    park_angles: tuple = (90,)    # 90, 60, 45
    grid_angles: tuple = ()       # 비우면 대지 변 방향에서 자동
    effort: str = "normal"        # fast | normal | thorough

    def aisle_for(self, ang):
        if ang == 90:
            return self.aisle_90
        if ang == 60:
            return self.aisle_60_one if self.two_entrances else self.aisle_60_two
        if ang == 45:
            return self.aisle_45_one if self.two_entrances else self.aisle_45_two
        raise ValueError(ang)

    def to_dict(self):
        d = asdict(self)
        for k, v in d.items():
            if isinstance(v, tuple):
                d[k] = list(v)
        return d

    @classmethod
    def from_dict(cls, d):
        r = cls()
        for k, v in (d or {}).items():
            if not hasattr(r, k) or v is None or v == "":
                continue
            cur = getattr(r, k)
            if k == "wall_rows":
                setattr(r, k, tuple(str(x) for x in (v if not isinstance(v, str) else v.split(",")) if x))
                continue
            if isinstance(cur, bool):
                v = v if isinstance(v, bool) else str(v).lower() in ("1", "true", "yes", "on")
            elif isinstance(cur, int) and not isinstance(cur, bool):
                v = int(float(v))
            elif isinstance(cur, float):
                v = float(v)
            elif isinstance(cur, tuple):
                if isinstance(v, str):
                    v = [x for x in v.replace(" ", "").split(",") if x]
                v = tuple(float(x) if "." in str(x) else int(x) for x in v)
            setattr(r, k, v)
        return r
