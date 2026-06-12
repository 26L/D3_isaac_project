#!/bin/bash
# run_full_pipeline_3d.sh
# =======================
# YOLO + Depth 기반 완전 3D 네비게이션 파이프라인
#
# 실행 순서:
#   [Isaac Sim] cow_capture_depth_script.py  ← 먼저 실행
#   [Terminal]  bash run_full_pipeline_3d.sh
#   [Isaac Sim] spot_cow_inspection_script.py

PYTHON="/home/rokey/dev_ws/venv/isaaclab/bin/python3"
DIR="/home/rokey/Downloads/yolo"

echo "======================================="
echo "  YOLO 3D Navigation Pipeline"
echo "======================================="

# 입력 확인
for f in /tmp/cow_capture.png /tmp/camera_info.json; do
    if [ ! -f "$f" ]; then
        echo "[ERROR] 없음: $f"
        echo "  Isaac Sim Script Editor에서 cow_capture_depth_script.py 먼저 실행"
        exit 1
    fi
done

# stereo 이미지 확인
if [ -f /tmp/cow_left.png ] && [ -f /tmp/cow_right.png ]; then
    echo "[INFO] 스테레오 이미지 발견 → stereo depth 계산 예정"
elif [ -f /tmp/cow_depth.npy ]; then
    echo "[INFO] depth .npy 발견"
else
    echo "[WARN] depth/stereo 없음 → ground-plane fallback 사용"
fi

# YOLO + 3D 역투영 + nav target 계산
echo ""
echo "[1/1] YOLO + 3D 역투영..."
$PYTHON "$DIR/cow_yolo_3d_nav.py"
if [ $? -ne 0 ]; then
    echo "[ERROR] 3D nav 계산 실패"
    exit 1
fi

echo ""
echo "======================================="
echo "  완료!"
echo "  /tmp/nav_target.json  — Spot 목표 좌표"
echo "  /tmp/cow_3d_result.png — 시각화"
echo ""
echo "  다음: Isaac Sim Script Editor에서"
echo "        spot_cow_inspection_script.py 실행"
echo "======================================="
