# abnormal_behavior_detector

## 개요

원형 교차로에서 이상 거동(역주행, 과속, 저속, 비정상 정차 등)을 하는 차량을 검출하는 노드입니다.

## 기능

### 검출 가능한 이상 거동

1. **역주행 (Wrong-way Driving)**
   - 차선의 정상 주행 방향과 반대로 주행하는 차량 검출
   - 연속 프레임 확인으로 오검출 방지
   - 신뢰도: 95%

2. **과속 (Over-speeding)**
   - 차선의 제한 속도를 초과하는 차량 검출
   - 기본: 제한 속도의 120% 초과 시
   - 신뢰도: 70%

3. **저속 (Under-speeding)**
   - 교통 흐름에 비해 현저히 느린 차량 검출
   - 기본: 제한 속도의 30% 미만
   - 신뢰도: 60%

4. **비정상 정차 (Abnormal Stop)**
   - 신호등이나 정지선이 아닌 곳에서 정차한 차량 검출
   - 신뢰도: 80%

## 입출력

### 입력

| 토픽 | 타입 | 설명 |
|------|------|------|
| `~/input/objects` | `autoware_auto_perception_msgs/PredictedObjects` | Prediction 모듈의 출력 |
| `~/input/vector_map` | `autoware_auto_mapping_msgs/HADMapBin` | Lanelet2 벡터 맵 |

### 출력

| 토픽 | 타입 | 설명 |
|------|------|------|
| `~/output/abnormal_objects` | `autoware_auto_perception_msgs/PredictedObjects` | 이상 거동 객체 |
| `~/debug/markers` | `visualization_msgs/MarkerArray` | 디버그 마커 (RViz 시각화용) |

## 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `dist_threshold_for_searching_lanelet` | 5.0 | 차선 검색 최대 거리 (m) |
| `delta_yaw_threshold_for_searching_lanelet` | 0.785 | 차선 검색 각도 임계값 (45도) |
| `wrong_way_angle_threshold` | 2.356 | 역주행 판단 각도 (135도) |
| `consecutive_count_threshold` | 3 | 역주행 확정 연속 프레임 수 |
| `speed_threshold_ratio` | 1.2 | 과속 판단 비율 (120%) |
| `min_speed_threshold` | 0.5 | 정차 판단 속도 (m/s) |
| `history_buffer_size` | 10 | 이력 버퍼 크기 |
| `history_timeout` | 3.0 | 이력 타임아웃 (초) |

## 사용 방법

### Planning Simulator에서 실행

```bash
# 1. Planning Simulator 실행
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/path/to/your/map \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit

# 2. 이상 거동 검출 노드 실행
ros2 launch abnormal_behavior_detector abnormal_behavior_detector.launch.xml
```

### 토픽 확인

```bash
# 이상 거동 객체 확인
ros2 topic echo /abnormal_behavior/objects

# 로그 확인
ros2 topic echo /rosout | grep ABNORMAL
```

### RViz2 시각화

RViz2에서 다음 디스플레이 추가:
- **MarkerArray**: `/abnormal_behavior/debug/markers`
  - 이상 거동 객체 위에 빨간 텍스트 및 원으로 표시

## 알고리즘

### 역주행 검출 로직

```
1. 객체 위치에서 가장 가까운 차선 찾기
2. 객체의 주행 방향 벡터 계산 (속도 + 방향)
3. 차선의 정상 주행 방향 벡터 계산
4. 두 벡터의 내적으로 각도 계산
5. 각도 > 135도 → 역주행 의심
6. 연속 3프레임 이상 역주행 → 확정
```

### 오검출 방지

- **연속 프레임 확인**: 일시적인 U턴이나 센서 노이즈 필터링
- **속도 임계값**: 정차 상태에서는 방향 판단 안 함
- **이력 관리**: 객체별 이력을 저장하여 일관성 확인

## 빌드 방법

```bash
cd ~/autoware
colcon build --packages-select abnormal_behavior_detector
source install/setup.bash
```

## 개발 로드맵

### Phase 1 (현재)
- [x] 기본 프레임워크
- [x] 역주행 검출
- [x] 과속/저속 검출
- [x] 비정상 정차 검출

### Phase 2 (예정)
- [ ] 신호등 정보 연동 (정상 정차 판단)
- [ ] 칼만 필터 기반 상태 추정
- [ ] 확률 기반 판단 (Covariance 활용)

### Phase 3 (예정)
- [ ] LSTM 기반 의도 예측
- [ ] 충돌 위험도 계산
- [ ] 자전거/보행자 이상 거동 검출

## 라이선스

Apache License 2.0

## 문의

프로젝트: 원형 교차로 이상 거동 검출 시스템
