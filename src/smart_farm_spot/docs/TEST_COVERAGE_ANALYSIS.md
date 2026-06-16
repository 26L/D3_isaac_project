# 테스트 커버리지 분석 및 개선 제안 (Test Coverage Analysis)

_작성일: 2026-06-11 · 대상: `smart_farm_spot` (꼬마 로봇 두리 — 스마트 축사 자율순찰 Spot 패키지)_

## 1. 요약 (Executive Summary)

현재 이 패키지의 **자동화된 테스트 커버리지는 사실상 0%** 이다.

- `setup.py` 에 `tests_require=["pytest"]` 가 선언되어 있으나 **실제 테스트 파일은 단 한 개도 존재하지 않는다.**
- 이름에 `test` 가 들어간 유일한 파일 `isaac/ros_bridge_test.py` 는 단위 테스트가 아니라 **수동 진단/스모크 스크립트**(ROS 토픽이 흐르는지 눈으로 확인)이다.
- `pytest.ini` / `tox.ini` / `conftest.py` 등 테스트 설정 파일이 없고, **CI 워크플로(`.github/workflows`)도 없다.**
- 코드 ~7,000 줄(28개 Python 모듈)이 검증 장치 없이 운영되고 있다.

대부분의 로직이 ROS2 노드(`rclpy.Node`) 안에 묶여 있어 통합 테스트는 시뮬레이터(Isaac Sim)와 Nav2 스택을 요구하지만, **순수 함수로 분리 가능한(혹은 이미 분리된) 핵심 수학·파싱 로직이 상당량** 있어서 빠르게 의미 있는 커버리지를 확보할 수 있다.

## 2. 현황 진단

| 항목 | 상태 |
|---|---|
| 단위 테스트 | ❌ 없음 |
| 통합 테스트 | ❌ 없음 (`ros_bridge_test.py` 는 수동 스크립트) |
| 테스트 러너 설정 | ❌ 없음 (`pytest.ini`/`conftest.py` 부재) |
| CI 파이프라인 | ❌ 없음 |
| `ament` 표준 테스트(`test_flake8`, `test_pep257`) | ❌ 미구성 |
| 코드 내 단언/계약 | ⚠️ 로깅 위주, 검증 부재 |

### 구조적 위험 요소
1. **중복된 핵심 함수**: `make_pose(x, y, yaw)`(yaw→쿼터니언 변환)가 `patrol.py`, `h_drive.py`, `waypoint_patrol.py`, `scenario_nav.py` 에 **거의 동일하게 4벌 복붙**되어 있다. 한 곳을 고치면 나머지가 드리프트한다 — 회귀 테스트가 없으면 발견 불가.
2. **검증되지 않은 좌표 수학**: 카메라 역투영, TF 변환, 쿼터니언 곱, depth 샘플링 등 **부호 하나만 틀려도 로봇이 엉뚱한 곳으로 가는** 로직이 전부 무테스트.
3. **외부 파일 의존**: YAML/JSON 웨이포인트 파싱이 실패 경로(파일 없음/키 누락) 처리 분기를 갖지만 한 번도 검증된 적 없음.

## 3. 우선순위별 테스트 개선 제안

### 🔴 P0 — 순수 기하/수학 함수 (가장 높은 ROI, 의존성 0)

시뮬레이터 없이 즉시 테스트 가능. 버그 시 영향도 최고.

- **`make_pose` / yaw→쿼터니언 변환** (`patrol.py:35`, `h_drive.py:34`, `waypoint_patrol.py:53`, `scenario_nav.py:39`)
  - 검증: yaw=0 → `(z=0, w=1)`; yaw=π → `(z≈1, w≈0)`; yaw=π/2 → `(z=w≈0.707)`; `z²+w²==1`.
  - **권장: 4벌 중복을 `smart_farm_spot/geometry.py` 같은 공용 모듈로 통합한 뒤 한 번만 테스트.**
- **쿼터니언 헬퍼** (`isaac/nav_bridge.py`): `_quat_mul`, `_yaw_quat_wxyz`, `_visual_quat_wxyz`
  - 검증: 항등원(`1,0,0,0`) 곱, 결합법칙, 단위 노름 보존, `FACE_YAW_OFFSET` 적용.
- **`_yaw_from_quat`** (`isaac/nav_policy_bridge.py:59`, `nav_policy_slam_bridge.py:84`)
  - 검증: `_yaw_quat_wxyz` 와의 **왕복(round-trip)** 일관성 — yaw→quat→yaw 가 입력과 일치.
- **`_lerp`** (`isaac/nav_bridge.py:373`): 경계값(t=0, t=1)과 중간값.

### 🔴 P0 — 항법 목표 기하 (Navigation Target Geometry)

- **소 접근점 계산** (`nav_to_cow.py:56-59`): 소→로봇 방향 단위벡터 × `APPROACH` 오프셋, 소를 바라보는 yaw. 거리 0 분기(`d < 1e-3`) 포함.
- **꼬리 후방점 계산** (`cow_tail_seek.py:_maybe_send`): `BEHIND` 만큼 물러난 점 + 꼬리 바라보는 yaw, `RESEND_DIST` 게이팅.
- **"다음 웨이포인트 바라보기" yaw** (`patrol.py:79`, `scenario_nav.py:127`): `atan2(ny-y, nx-x)` 및 순환 인덱스 `(i+1) % n`.
- **카메라 역투영** (`cow_tail_seek.py:123-125`): `(u,v,d) → (x,y,z)` 핀홀 모델. 내부파라미터 `fx,fy,cx,cy` 로 광학중심·임의 픽셀 검증.

### 🟠 P1 — 파싱 / 설정 로딩 (실패 경로 포함)

- **YAML 웨이포인트 로더** (`waypoint_patrol.py:66 load_waypoints_from_yaml`, `h_drive.py:45 load_waypoints`)
  - 정상 파싱, `yaw` 키 누락 시 기본값 0.0, `waypoints` 키 부재 시 빈 리스트.
  - `waypoint_patrol.py` 의 **로드 실패 → `DEFAULT_WAYPOINTS` 폴백** 분기(현재 무검증).
- **JSON 웨이포인트 로더** (`patrol.py:52`, `scenario_nav.py:64`): `/tmp/*.json` 구조 파싱, 빈 리스트 시 `RuntimeError`.
- **저장된 config YAML 정합성**: `config/*.yaml`, `maps/*.yaml` 이 실제 로더 기대 스키마와 일치하는지 데이터 기반 테스트.

### 🟠 P1 — 비전/열화상 순수 함수 (numpy/cv2, 시뮬레이터 불필요)

`wip/thermal_processor.py` 는 ROS 와 분리된 순수 영상 함수가 있어 **가장 테스트하기 쉬운 모듈** 중 하나:
- `apply_fake_thermal(rgb)` (`:30`): 핫스팟/극값 마스크가 의도한 색으로 칠해지는지(합성 입력).
- `get_hotspot_bbox(thermal, min_area)` (`:59`): `min_area` 미만 노이즈 제거, bbox 좌표 정확도.
- `draw_hotspot_overlay` (`:110`): 오버레이가 프레임 크기를 보존하는지.
- **`_sample_depth`** (`cow_tail_seek.py:139`, `yolo_view.py:102`): RGB↔depth 해상도 스케일링, 경계 클램프, 비유한/범위밖 값(`0.1 < d < 30.0`) 필터링.

### 🟡 P2 — 노드 상태 머신 로직 (경량 모킹)

`rclpy` 콜백을 모킹하면 순찰 진행 로직을 테스트 가능:
- **인덱스 진행 + 루프 카운팅** (`h_drive.py:_send_next`, `patrol.py:_send_next`): 마지막 도달 후 `loop` 종료 vs 무한 반복, `SF_PATROL_LOOPS` 경계.
- **목표 거부/실패 시 다음 점 진행** (`_on_goal_response`, `_on_result`, status==4 분기).
- **탐색 회전 게이팅** (`cow_tail_seek.py:_search`): `_goal_sent`/`_last_detect` 상태에 따른 회전 발행 여부.
- **중앙값 안정화** (`cow_tail_seek.py:_maybe_send`): `N_STABLE` 개 누적 후 median 산출, 슬라이딩 윈도(`pop(0)`).

### 🟡 P2 — 환경변수 기반 설정

`SF_BEHIND_TAIL`, `SF_SEARCH_WZ`, `SF_PATROL_LOOPS`, `SF_COW_YAW`, `SF_BASE_FLIP` 등 다수의 동작이 환경변수로 결정된다. 기본값/파싱/형변환을 테스트하면 배포 설정 실수를 방지한다.

### 🟢 P3 — 통합/스모크 (인프라 필요)

- `launch/*.launch.py` 가 import·구성 오류 없이 로드되는지 `launch_testing` 스모크.
- `isaac/ros_bridge_test.py` 를 수동 스크립트에서 **pytest 통합 테스트로 승격**(토픽 발행/구독 카운트 단언).

## 4. 권장 실행 계획 (Roadmap)

1. **테스트 하네스 부트스트랩**
   - `src/smart_farm_spot/test/` 디렉터리 + `conftest.py` 생성.
   - `setup.cfg` 또는 `pytest.ini` 에 `pytest` 설정 추가, `ament_python` 표준 테스트(`test_flake8`, `test_pep257`) 등록.
   - 무거운 의존성(`rclpy`, `cv2`, `ultralytics`, `torch`, `omni.*`)은 **import 보호 또는 모킹**하여 CI 머신에서 시뮬레이터 없이 P0/P1 테스트가 돌도록.
2. **중복 제거 후 테스트**: 4벌 `make_pose` 를 공용 `geometry.py` 로 통합 → 단일 테스트로 4개 노드 동시 보호.
3. **P0 → P1 → P2 순서로 테스트 작성.** 우선 순수 함수만으로도 핵심 안전 로직(좌표/항법) 커버리지 확보.
4. **CI 도입**: `.github/workflows/` 에 `pytest` + flake8 워크플로 추가, PR 게이트화.

## 5. 빠른 효과 정리 (Quick Wins)

| 작업 | 의존성 | 난이도 | 효과 |
|---|---|---|---|
| `make_pose`/쿼터니언 테스트 | 없음 | 낮음 | 4개 노드의 항법 정확도 보호 |
| `_yaw_from_quat` 왕복 테스트 | 없음 | 낮음 | 정책 브릿지 odom yaw 검증 |
| YAML/JSON 로더 + 폴백 테스트 | `pyyaml` | 낮음 | 설정 실수 조기 발견 |
| `thermal_processor` 함수 테스트 | `numpy`,`cv2` | 낮음 | 비전 후처리 회귀 방지 |
| `_sample_depth` 경계 테스트 | `numpy` | 낮음 | depth 좌표 버그 방지 |

> 위 5개만 작성해도 시뮬레이터 없이 **로봇이 "어디로 갈지"를 결정하는 핵심 수학** 대부분을 커버할 수 있다.
