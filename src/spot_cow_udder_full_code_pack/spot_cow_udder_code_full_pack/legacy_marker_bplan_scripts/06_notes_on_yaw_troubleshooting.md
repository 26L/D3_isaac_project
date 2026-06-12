# Yaw / Transform Troubleshooting Notes

## 문제

`/World/spot_with_arm` root transform을 직접 수정하면서 Spot 모델이 뒤집히는 문제가 발생했습니다.

## 원인 추정

- `/World/spot_with_arm`의 root transform op에는 `translate`, `rotateY`, `rotateX`, `rotateZ`, `scale` 등이 존재했습니다.
- `ClearXformOpOrder()`로 transform op를 지우고 새 orient/rotateZ를 넣으면 원래 USD가 가지고 있던 upright orientation이 깨집니다.
- `rotateXYZ`, `rotateZYX`의 성분을 직접 수정하면 USD rotation order 때문에 yaw가 아니라 roll/pitch가 바뀔 수 있습니다.

## 해결 원칙

1. Spot root에는 `ClearXformOpOrder()`를 쓰지 않는다.
2. 이동은 translate op만 수정한다.
3. `/World/spot_with_arm/base`를 실제 몸체 기준으로 사용한다.
4. root origin이 아니라 base가 goal에 오도록 root translate를 offset 보정한다.
5. yaw 자동정렬은 안정화 전까지 수동 Rotate Z 조정 또는 별도 controller로 분리한다.
