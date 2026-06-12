# Spot Cow Udder Inspection — Full Code Pack

이 ZIP은 발표 PPT에 들어간 실제 파이프라인 코드 4개와, 초기 개발 과정에서 사용한 marker/GT 기반 B안 보조 스크립트들을 함께 묶은 GitHub 업로드용 코드팩입니다.

## 1. 실제 파이프라인 코드

`actual_yolo_depth_pipeline/`

```text
cow_capture_depth_script.py
cow_yolo_3d_nav.py
spot_cow_inspection_script.py
run_full_pipeline_3d.sh
```

실행 순서:

```text
[Isaac Sim Script Editor]
cow_capture_depth_script.py
  → /tmp/cow_capture.png
  → /tmp/cow_left.png
  → /tmp/cow_right.png
  → /tmp/camera_info.json

[Terminal]
bash run_full_pipeline_3d.sh
  → cow_yolo_3d_nav.py
  → /tmp/nav_target.json
  → /tmp/cow_3d_result.png

[Isaac Sim Script Editor]
spot_cow_inspection_script.py
  → nav_target.json 읽기
  → Spot teleport
  → ARM_READY → ARM_INSERT
  → udder capture
  → ARM_STOW
```

## 2. 초기 marker/GT 기반 B안 보조 코드

`legacy_marker_bplan_scripts/`

```text
00_create_simple_cow_proxy_and_udder_markers.py
01_create_cow_pose_markers_red.py
02_print_marker_world_positions.py
03_b_plan_marker_rear_goal_demo.py
04_spot_root_xform_diagnostics.py
05_remove_yaw_delta_op.py
06_notes_on_yaw_troubleshooting.md
```

이 코드들은 다음 개발 과정에 사용했습니다.

- 실제 cow USD가 없을 때 simple cow proxy와 udder/teat marker 생성
- 실제 cow USD가 들어온 뒤 head/tail/hind-leg marker 배치
- marker 기반으로 소 뒤쪽 goal 계산
- `/World/spot_with_arm/base`가 goal marker에 오도록 root translate 보정
- Spot root transform op가 꼬였을 때 진단 및 yaw_delta 제거

## 3. 주요 path

```text
Spot root:
  /World/spot_with_arm

Spot body 기준:
  /World/spot_with_arm/base

Cow pose markers:
  /World/cow_head_marker
  /World/cow_tail_marker
  /World/cow_left_hind_marker
  /World/cow_right_hind_marker

B-plan goal marker:
  /World/b_plan_goal_marker
```

## 4. 핵심 수식

### Marker 기반 B안

```python
rear_dir = normalize(tail_marker - head_marker)
hind_center = (left_hind_marker + right_hind_marker) / 2
goal = hind_center + rear_dir * rear_distance
```

### YOLO + Depth 기반 3D 좌표

```python
Xc =  (u - cx) / fx * depth
Yc = -(v - cy) / fy * depth
Zc = -depth

world = [Xc, Yc, Zc, 1.0] @ world_matrix
```

### 후방 접근 target

```python
tail_dir = tail_3d - head_3d
tail_dir[2] = 0
tail_dir = tail_dir / norm(tail_dir)

target = tail_3d + tail_dir * REAR_DIST
target[2] = SPOT_Z

face_dir = -tail_dir
yaw_deg = atan2(face_dir[1], face_dir[0])
```

## 5. 주의사항

- `cow_capture_depth_script.py`와 `spot_cow_inspection_script.py`는 Isaac Sim Script Editor에서 실행합니다.
- `cow_yolo_3d_nav.py`와 `run_full_pipeline_3d.sh`는 터미널에서 실행합니다.
- USD/Omniverse는 row-vector 방식이므로 `local @ M`을 사용합니다.
- Translation은 일반적인 `M[0:3,3]`이 아니라 USD 기준 `M[3,0:3]`입니다.
- Spot root에 `ClearXformOpOrder()`를 쓰면 원래 자세가 깨질 수 있습니다.
