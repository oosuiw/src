# Adaptive NDT Controller - 파라미터 튜닝 가이드

## 목차
1. [개요](#개요)
2. [파라미터 이해하기](#파라미터-이해하기)
3. [튜닝 단계별 가이드](#튜닝-단계별-가이드)
4. [실험 시나리오별 추천 설정](#실험-시나리오별-추천-설정)
5. [문제 해결 및 최적화](#문제-해결-및-최적화)
6. [성능 평가 메트릭](#성능-평가-메트릭)

---

## 개요

이 가이드는 `adaptive_ndt_controller`의 파라미터를 체계적으로 튜닝하는 방법을 제공합니다.
적응형 NDT 제어의 성능은 파라미터 설정에 크게 의존하므로, 실험 환경과 목적에 맞는 최적화가 필수적입니다.

### 튜닝 목표
- **정확도 향상**: 불확실성이 높은 상황에서 위치추정 오차 감소
- **강건성 확보**: 다양한 환경에서 안정적인 수렴
- **효율성 유지**: 불필요한 계산 부하 최소화

---

## 파라미터 이해하기

### 1. Control Gains (제어 게인)

#### `gain_step_size` (기본값: 0.5)
**역할**: 위치 불확실성에 따른 step_size 증가율

```
새로운 step_size = base_step_size + gain_step_size × sqrt(std_x² + std_y²)
```

- **값이 클 때**:
  - 장점: 불확실성 높을 때 탐색 범위 크게 확장 → 전역 최적해 찾기 유리
  - 단점: 과도한 step_size로 인한 발산 가능성, 계산 불안정

- **값이 작을 때**:
  - 장점: 안정적인 수렴, 미세 조정에 유리
  - 단점: 불확실성 높을 때 local minimum에 빠질 위험

**추천 범위**: 0.1 ~ 2.0

**튜닝 시작점**:
```yaml
# 보수적 (안정성 중시)
gain_step_size: 0.2

# 균형형 (기본)
gain_step_size: 0.5

# 공격적 (탐색 범위 중시)
gain_step_size: 1.0
```

#### `gain_max_iterations` (기본값: 10.0)
**역할**: yaw 불확실성에 따른 최대 반복 횟수 증가율

```
새로운 max_iterations = base_max_iterations + gain_max_iterations × std_yaw
```

- **값이 클 때**:
  - 장점: 회전 방향 불확실성 클 때 더 정밀한 정렬
  - 단점: 계산 시간 증가, 실시간성 저하

- **값이 작을 때**:
  - 장점: 빠른 처리 속도
  - 단점: 회전 정렬 부족으로 인한 오차 증가

**추천 범위**: 5.0 ~ 30.0

**튜닝 시작점**:
```yaml
# 속도 중시
gain_max_iterations: 5.0

# 균형형 (기본)
gain_max_iterations: 10.0

# 정확도 중시
gain_max_iterations: 20.0
```

#### `gain_resolution` (기본값: 0.2, 기본 비활성화)
**역할**: 위치 불확실성에 따른 복셀 해상도 조정

```
새로운 resolution = base_resolution - gain_resolution × sqrt(std_x² + std_y²)
```

⚠️ **주의**: Resolution 변경은 큰 계산 부하를 유발하므로 신중히 사용!

**추천**: 초기 실험에서는 비활성화 유지 (`enable_resolution_control: false`)

---

### 2. Base Parameters (기준 파라미터)

#### `base_step_size` (기본값: 0.1)
NDT의 기본 step size. 불확실성이 0일 때 사용되는 값.

**설정 원칙**:
- `ndt_scan_matcher.param.yaml`의 `step_size` 값과 동일하게 설정
- NDT 기본 성능을 유지하는 기준점

```yaml
base_step_size: 0.1  # ndt_scan_matcher의 기본값과 일치시킴
```

#### `base_max_iterations` (기본값: 30)
NDT의 기본 최대 반복 횟수.

**설정 원칙**:
- `ndt_scan_matcher.param.yaml`의 `max_iterations` 값과 동일하게 설정
- 일반적인 상황에서 충분한 수렴을 보장하는 값

```yaml
base_max_iterations: 30  # ndt_scan_matcher의 기본값과 일치시킴
```

#### `base_resolution` (기본값: 2.0)
NDT 복셀 그리드의 기본 해상도 [미터].

**설정 원칙**:
- `ndt_scan_matcher.param.yaml`의 `resolution` 값과 동일하게 설정

```yaml
base_resolution: 2.0  # ndt_scan_matcher의 기본값과 일치시킴
```

---

### 3. Uncertainty Thresholds (불확실성 임계값)

#### `uncertainty_threshold_high` (기본값: 0.5)
**역할**: 높은 불확실성 영역 정의 [미터 또는 라디안]

**활용**:
- 이 값을 초과하면 "고불확실성" 상황으로 간주
- 로깅, 알람, 추가 안전 로직 트리거에 사용 가능

**설정 방법**:
1. 정상 주행 데이터에서 `std_x`, `std_y`, `std_yaw` 분포 분석
2. 99 백분위수(99th percentile) 값을 기준으로 설정
3. GPS 차단, 동적 장애물 등 극한 상황 고려

```yaml
# 예시: 정상 주행 시 std_x, std_y < 0.3m, 극한 상황 < 0.8m
uncertainty_threshold_high: 0.5  # 중간값 선택
```

#### `uncertainty_threshold_low` (기본값: 0.1)
**역할**: 낮은 불확실성 영역 정의

**활용**:
- 이 값 미만이면 "저불확실성" 상황 → 최소 적응 또는 적응 스킵
- 계산 자원 절약

```yaml
# 예시: 정상 주행 시 평균 std < 0.15m
uncertainty_threshold_low: 0.1
```

---

### 4. Enable Flags (기능 활성화 플래그)

#### `enable_step_size_control` (기본값: true)
step_size 적응 제어 활성화

**추천**: 항상 활성화 (핵심 기능)

#### `enable_max_iterations_control` (기본값: true)
max_iterations 적응 제어 활성화

**추천**: 항상 활성화 (회전 정렬 개선)

#### `enable_resolution_control` (기본값: false)
resolution 적응 제어 활성화

**추천**:
- 초기 실험: 비활성화
- 고급 최적화: 충분한 계산 자원 확보 후 활성화 테스트

---

### 5. Rate Limiting (업데이트 속도 제한)

#### `min_param_update_interval_sec` (기본값: 0.5)
**역할**: 파라미터 업데이트 최소 간격 [초]

**설정 기준**:
- **LiDAR 주파수**: 10Hz LiDAR → 0.1초 간격도 가능
- **계산 부하**: 파라미터 변경 시 NDT 내부 재초기화 고려
- **안정성**: 너무 빈번한 변경은 수렴 방해

```yaml
# LiDAR 10Hz, 매 프레임마다 업데이트
min_param_update_interval_sec: 0.1

# 보수적 (기본)
min_param_update_interval_sec: 0.5

# 매우 보수적 (느린 변화)
min_param_update_interval_sec: 1.0
```

---

## 튜닝 단계별 가이드

### Step 1: 기준선 (Baseline) 설정

먼저 적응형 제어를 **비활성화**한 고정 파라미터로 성능 측정.

```yaml
# baseline_fixed.yaml
/adaptive_ndt_controller:
  ros__parameters:
    # 모든 적응 제어 비활성화
    enable_step_size_control: false
    enable_max_iterations_control: false
    enable_resolution_control: false

    # 기본값만 사용
    base_step_size: 0.1
    base_max_iterations: 30
    base_resolution: 2.0
```

**실험**:
1. 다양한 시나리오(정상 주행, GPS 차단, 터널 등)에서 주행
2. RMSE, 수렴 실패율, 계산 시간 기록
3. 이 값이 비교 기준(baseline)이 됨

---

### Step 2: 불확실성 데이터 수집 및 분석

적응형 제어를 활성화하기 전, 불확실성 분포를 이해해야 합니다.

```bash
# 불확실성 벡터 로깅
ros2 topic echo /localization/diagnostics/uncertainty_vector > uncertainty_log.txt

# 불확실성 스코어 로깅
ros2 topic echo /localization/diagnostics/uncertainty_score > score_log.txt
```

**분석**:
1. `std_x`, `std_y`, `std_z`, `std_yaw`의 평균, 표준편차, 최대/최소값 계산
2. 히스토그램 생성 → 분포 확인
3. 시나리오별 차이 파악 (정상 vs 극한 상황)

**Python 분석 예시**:
```python
import numpy as np
import matplotlib.pyplot as plt

# 로그 파일에서 std_x, std_y 추출 (형식에 맞게 파싱)
std_x_data = [...]  # 실제 데이터
std_y_data = [...]

position_uncertainty = np.sqrt(np.array(std_x_data)**2 + np.array(std_y_data)**2)

print(f"Position Uncertainty Stats:")
print(f"  Mean: {np.mean(position_uncertainty):.4f}")
print(f"  Std: {np.std(position_uncertainty):.4f}")
print(f"  50th percentile: {np.percentile(position_uncertainty, 50):.4f}")
print(f"  95th percentile: {np.percentile(position_uncertainty, 95):.4f}")
print(f"  99th percentile: {np.percentile(position_uncertainty, 99):.4f}")

plt.hist(position_uncertainty, bins=50)
plt.xlabel('Position Uncertainty (m)')
plt.ylabel('Frequency')
plt.title('Position Uncertainty Distribution')
plt.show()
```

---

### Step 3: 보수적 설정으로 시작

데이터 분석 결과를 바탕으로 **보수적인 gain** 값으로 시작.

```yaml
# conservative_adaptive.yaml
/adaptive_ndt_controller:
  ros__parameters:
    ndt_node_name: "ndt_scan_matcher"

    # 보수적 게인 (작은 값)
    gain_step_size: 0.2          # 천천히 증가
    gain_max_iterations: 5.0      # 약간만 증가
    gain_resolution: 0.0          # 비활성화

    # 기본값 (ndt_scan_matcher와 일치)
    base_step_size: 0.1
    base_max_iterations: 30
    base_resolution: 2.0

    # 임계값 (데이터 분석 기반)
    uncertainty_threshold_high: 0.5  # 99th percentile
    uncertainty_threshold_low: 0.1   # 평균

    # 활성화 플래그
    enable_step_size_control: true
    enable_max_iterations_control: true
    enable_resolution_control: false

    # 업데이트 간격
    min_param_update_interval_sec: 0.5
```

**실험**:
1. 동일한 시나리오 재실행
2. baseline과 비교
3. 개선이 없거나 악화되면 → gain을 더 낮춤
4. 개선이 있지만 미미하면 → Step 4로 진행

---

### Step 4: 점진적 게인 증가

성능 개선이 확인되면 gain을 점진적으로 증가시키며 최적값을 찾습니다.

**Grid Search 방식**:

```yaml
# 실험 1: gain_step_size 탐색
gain_step_size: [0.2, 0.4, 0.6, 0.8, 1.0]
gain_max_iterations: 5.0  # 고정

# 실험 2: gain_max_iterations 탐색
gain_step_size: 0.6  # 실험 1의 최적값 사용
gain_max_iterations: [5.0, 10.0, 15.0, 20.0]
```

**기록 사항**:
| gain_step_size | gain_max_iterations | RMSE (m) | 수렴 실패율 (%) | 평균 계산 시간 (ms) |
|----------------|---------------------|----------|-----------------|---------------------|
| 0.2            | 5.0                 | ...      | ...             | ...                 |
| 0.4            | 5.0                 | ...      | ...             | ...                 |
| ...            | ...                 | ...      | ...             | ...                 |

**최적값 선택**:
- RMSE가 가장 낮은 조합 선택
- 단, 계산 시간이 실시간 제약(예: 100ms) 이내여야 함

---

### Step 5: 미세 조정 (Fine-tuning)

최적 근처에서 미세 조정 수행.

```yaml
# 예시: Step 4에서 gain_step_size=0.6, gain_max_iterations=10.0이 최적
# 주변 값으로 미세 탐색
gain_step_size: [0.5, 0.55, 0.6, 0.65, 0.7]
gain_max_iterations: [8.0, 9.0, 10.0, 11.0, 12.0]
```

---

### Step 6: 업데이트 간격 최적화

파라미터 게인이 확정되면 업데이트 주기를 조정.

```yaml
min_param_update_interval_sec: [0.1, 0.2, 0.5, 1.0]
```

**실험**:
- 간격이 짧을수록 반응성 좋지만 계산 부하 증가
- 간격이 길면 느린 적응

**최적 간격**: 불확실성 변화율 분석
- 불확실성이 천천히 변하면 → 긴 간격 (0.5~1.0초)
- 불확실성이 급격히 변하면 → 짧은 간격 (0.1~0.2초)

---

### Step 7: 고급 - Resolution 제어 (선택)

충분한 계산 자원이 있고 추가 성능 향상이 필요한 경우.

```yaml
enable_resolution_control: true
gain_resolution: [0.1, 0.2, 0.3, 0.4]
```

⚠️ **주의**:
- Resolution 변경은 NDT 내부 맵 재구성 유발
- 실시간성 보장 확인 필수

---

## 실험 시나리오별 추천 설정

### 시나리오 1: 도심 주행 (정상 GPS 수신)

**특징**:
- 낮은~중간 불확실성
- 빠른 계산 속도 요구

```yaml
/adaptive_ndt_controller:
  ros__parameters:
    gain_step_size: 0.3
    gain_max_iterations: 8.0

    base_step_size: 0.1
    base_max_iterations: 30

    uncertainty_threshold_high: 0.4
    uncertainty_threshold_low: 0.08

    enable_step_size_control: true
    enable_max_iterations_control: true
    enable_resolution_control: false

    min_param_update_interval_sec: 0.5
```

---

### 시나리오 2: 터널/지하 주차장 (GPS 차단)

**특징**:
- 높은 불확실성
- 정확도 최우선

```yaml
/adaptive_ndt_controller:
  ros__parameters:
    gain_step_size: 0.8
    gain_max_iterations: 15.0

    base_step_size: 0.1
    base_max_iterations: 35

    uncertainty_threshold_high: 0.8
    uncertainty_threshold_low: 0.15

    enable_step_size_control: true
    enable_max_iterations_control: true
    enable_resolution_control: false

    min_param_update_interval_sec: 0.3
```

---

### 시나리오 3: 고속도로 주행

**특징**:
- 매우 낮은 불확실성
- 고속 처리 요구

```yaml
/adaptive_ndt_controller:
  ros__parameters:
    gain_step_size: 0.2
    gain_max_iterations: 5.0

    base_step_size: 0.1
    base_max_iterations: 25

    uncertainty_threshold_high: 0.3
    uncertainty_threshold_low: 0.05

    enable_step_size_control: true
    enable_max_iterations_control: true
    enable_resolution_control: false

    min_param_update_interval_sec: 1.0  # 불확실성 변화 느림
```

---

### 시나리오 4: 동적 장애물 많은 환경

**특징**:
- 중간~높은 불확실성
- 급격한 불확실성 변화

```yaml
/adaptive_ndt_controller:
  ros__parameters:
    gain_step_size: 0.6
    gain_max_iterations: 12.0

    base_step_size: 0.1
    base_max_iterations: 30

    uncertainty_threshold_high: 0.6
    uncertainty_threshold_low: 0.12

    enable_step_size_control: true
    enable_max_iterations_control: true
    enable_resolution_control: false

    min_param_update_interval_sec: 0.2  # 빠른 적응
```

---

## 문제 해결 및 최적화

### 문제 1: 적응형 제어가 baseline보다 성능이 나쁨

**원인**:
- Gain 값이 너무 높아 과도한 파라미터 변화
- 업데이트 간격이 너무 짧아 수렴 방해

**해결**:
```yaml
# Gain을 절반으로 줄임
gain_step_size: 0.25  # 0.5 → 0.25
gain_max_iterations: 5.0  # 10.0 → 5.0

# 업데이트 간격 늘림
min_param_update_interval_sec: 1.0  # 0.5 → 1.0
```

---

### 문제 2: 고불확실성 상황에서 여전히 수렴 실패

**원인**:
- Gain 값이 너무 낮아 충분한 적응 안 됨
- base 값 자체가 부족

**해결**:
```yaml
# Gain 증가
gain_step_size: 1.0
gain_max_iterations: 20.0

# Base 값도 상향 조정
base_max_iterations: 40  # 30 → 40
```

---

### 문제 3: 계산 시간 초과 (real-time 위반)

**원인**:
- max_iterations가 너무 많이 증가
- 불필요하게 빈번한 업데이트

**해결**:
```yaml
# max_iterations 증가를 제한
gain_max_iterations: 5.0  # 줄임

# 또는 코드 수정: 클램핑 범위 축소
# adaptive_ndt_controller_node.cpp:169
# new_max_iterations = std::clamp(new_max_iterations, 10, 50);  // 100 → 50

# 업데이트 간격 늘림
min_param_update_interval_sec: 0.8
```

---

### 문제 4: 불확실성 변화에 적응이 느림

**원인**:
- 업데이트 간격이 너무 김
- Gain이 너무 작음

**해결**:
```yaml
# 업데이트 간격 단축
min_param_update_interval_sec: 0.2

# Gain 증가
gain_step_size: 0.8
```

---

## 성능 평가 메트릭

### 1. 위치추정 정확도

**RMSE (Root Mean Square Error)**:
```
RMSE = sqrt(mean((x_est - x_true)² + (y_est - y_true)²))
```

**측정 방법**:
- Ground truth: RTK-GPS 또는 사전 구축된 정밀 경로
- 비교: 각 타임스탬프에서 추정 위치와 ground truth 차이 계산

**목표**: Adaptive < Baseline RMSE

---

### 2. 수렴 성공률

**정의**:
```
수렴 성공률 = (수렴 성공한 프레임 수) / (전체 프레임 수) × 100%
```

**수렴 판정 기준**:
- NDT의 `transform_probability` > threshold
- 또는 `nearest_voxel_transformation_likelihood` > threshold

**목표**: Adaptive ≥ Baseline (특히 고불확실성 상황)

---

### 3. 계산 시간

**측정**:
```bash
ros2 topic echo /diagnostics | grep exe_time
```

**메트릭**:
- 평균 계산 시간
- 95th percentile 계산 시간
- 최대 계산 시간

**목표**: Real-time 제약 만족 (예: < 100ms)

---

### 4. 불확실성 감소율

**정의**:
```
감소율 = (EKF 불확실성 - NDT 후 불확실성) / EKF 불확실성 × 100%
```

**측정**:
- EKF a priori 불확실성 vs a posteriori 불확실성 비교

**목표**: Adaptive가 더 높은 감소율

---

### 5. 강건성 테스트

**시나리오 기반 평가**:

| 시나리오            | Baseline 수렴률 | Adaptive 수렴률 | 개선율 |
|---------------------|-----------------|-----------------|--------|
| 정상 주행           | 98%             | 99%             | +1%    |
| GPS 차단 (터널)     | 85%             | 93%             | +8%    |
| 동적 장애물         | 88%             | 94%             | +6%    |
| 다층 주차장         | 80%             | 90%             | +10%   |

**목표**: 모든 시나리오에서 Adaptive ≥ Baseline

---

## 체크리스트: 튜닝 완료 확인

- [ ] Baseline 성능 측정 완료
- [ ] 불확실성 데이터 수집 및 분석 완료
- [ ] 불확실성 임계값 설정 (threshold_high, threshold_low)
- [ ] Base 파라미터를 ndt_scan_matcher와 일치시킴
- [ ] 보수적 gain으로 시작하여 점진적 증가 실험 완료
- [ ] Grid search 또는 유사 방법으로 최적 gain 탐색
- [ ] 업데이트 간격 최적화 완료
- [ ] 다양한 시나리오에서 성능 검증
- [ ] RMSE, 수렴률, 계산 시간 모두 baseline 대비 개선 확인
- [ ] Real-time 제약 만족 확인
- [ ] 최종 설정을 `adaptive_ndt_controller.param.yaml`에 반영
- [ ] 논문용 실험 데이터 및 그래프 준비 완료

---

## 추가 팁

### 로그 분석 도구 활용

ROS 2 로그에서 적응 파라미터 변화를 추적:

```bash
# 실시간 모니터링
ros2 topic echo /rosout | grep "Adaptive"

# 파일로 저장
ros2 topic echo /rosout | grep "Adaptive" > adaptive_params_log.txt
```

로그 예시:
```
[adaptive_ndt_controller]: Adaptive step_size: 0.1234 (pos_uncertainty=0.0567)
[adaptive_ndt_controller]: Adaptive max_iterations: 35 (std_yaw=0.0890)
```

### Python 스크립트로 자동 실험

여러 gain 조합을 자동으로 테스트:

```python
import subprocess
import yaml

gain_step_sizes = [0.2, 0.4, 0.6, 0.8, 1.0]
gain_max_iterations = [5.0, 10.0, 15.0, 20.0]

for gs in gain_step_sizes:
    for gm in gain_max_iterations:
        # 파라미터 파일 수정
        config = {
            '/adaptive_ndt_controller': {
                'ros__parameters': {
                    'gain_step_size': gs,
                    'gain_max_iterations': gm,
                    # ... 기타 파라미터
                }
            }
        }

        with open('config/experiment.yaml', 'w') as f:
            yaml.dump(config, f)

        # ROS 2 런치 실행
        subprocess.run(['ros2', 'launch', 'adaptive_ndt_controller',
                       'adaptive_ndt_controller.launch.xml',
                       'config_file:=config/experiment.yaml'])

        # 결과 수집 및 저장
        # ...
```

---

**작성일**: 2025-10-18
**버전**: 1.0
**문의**: adaptive_ndt_controller 개발팀
