# `pose_uncertainty_monitor` 패키지 설명서

이 패키지는 Autoware의 EKF(Extended Kalman Filter)가 출력하는 공분산(Covariance) 값을 기반으로, 현재 위치 추정(Localization) 상태의 불확실성을 정량적인 '스코어'로 계산하고 발행하는 역할을 합니다.

이 스코어를 통해 측위 시스템의 안정성을 실시간으로 모니터링하고, 특정 상황(예: 터널)에서 측위가 불안정해질 가능성을 미리 예측할 수 있습니다.

---

## 1. 측위 성능 지표 비교

Autoware에서 측위 성능을 나타내는 주요 지표들은 각기 다른 관점의 정보를 제공합니다.

| 지표 (Metric) | 토픽 (Topic) | 무엇을 나타내는가? | 관점 (Perspective) |
| :--- | :--- | :--- | :--- |
| **자세 불확실성 스코어** | `/localization/pose_uncertainty` | EKF가 예측(Predict) 단계 직후 계산한, **미래 위치에 대한 불확실성의 총량** | 측위가 **불안정해질 가능성**을 미리 알려주는 **예측(Predictive) 지표** |
| **NDT 변환 우도** | `/.../nearest_voxel_transformation_likelihood` | 현재 Lidar 스캔이 지도와 얼마나 일관성 있게 정합되었는가에 대한 **매칭 점수** | 센서 관측이 끝난 후, **현재 상태가 얼마나 잘 맞았는지**를 알려주는 **사후(Reactive) 지표** |
| **공분산 타원 분석** | `/localization_accuracy` | EKF 최종 출력 공분산으로부터 계산된 **2D 불확실성 타원의 기하학적 크기** | 현재 불확실성을 **타원의 장/단반경** 형태로 시각화하는 **사후(Reactive) 지표** |

### 'pose_uncertainty_monitor'와 'localization_error_monitor'의 핵심 차이점

두 패키지 모두 EKF의 공분산을 사용하지만, 계산 시점과 방식, 주된 용도에서 다음과 같은 핵심적인 차이가 있습니다.

| 구분 | `pose_uncertainty_monitor` (본 패키지) | `localization_error_monitor` (`/localization_accuracy`) |
| :--- | :--- | :--- |
| **계산 시점** | **사전(a priori) 공분산** 사용 (EKF의 **예측** 단계 직후) | **사후(a posteriori) 공분산** 사용 (EKF의 **최종 결과** 출력) |
| **계산 방식** | **Trace(대각합)** 계산 (6-DOF 전체 불확실성의 총량) | **고유값(Eigenvalue)** 계산 (2D 위치 불확실성의 기하학적 크기) |
| **주된 용도** | 측위가 불안정해질 **가능성**을 사전에 수치로 감지 | 현재 결과의 불확실성을 **타원의 크기**로 표현 및 시각화 |
| **스코어 특성**| **예측적(Predictive)**: 문제가 발생하기 전 값이 **지속적으로 상승** | **결과적(Reactive)**: 문제가 발생한 후의 최종 불확실성 크기를 표시 |

---

## 2. `ekf_localizer` 패키지 수정 내역

'자세 불확실성 스코어'를 계산하기 위해서는 EKF의 내부 변수인 **'사전(a priori) 공분산'** 값이 필요합니다. 표준 `ekf_localizer`는 이 값을 외부로 발행하지 않으므로, 다음과 같이 소스 코드를 직접 수정했습니다.

1.  **`ekf_localizer.hpp` 수정**:
    *   `pub_a_priori_pose_cov_` 라는 새로운 `rclcpp::Publisher`를 `EKFLocalizer` 클래스에 멤버 변수로 선언했습니다.

2.  **`ekf_localizer.cpp` (생성자) 수정**:
    *   선언된 Publisher를 `~/output/a_priori_pose_with_covariance` 라는 새 토픽 이름으로 초기화했습니다.

3.  **`ekf_module.hpp` 수정**:
    *   `EKFModule` 클래스에 `getAPrioriPoseWithCovariance` 라는 새로운 public 함수를 선언하여, EKF의 내부 상태를 외부에서 접근할 수 있는 통로를 만들었습니다.

4.  **`ekf_module.cpp` 수정**:
    *   `getAPrioriPoseWithCovariance` 함수의 내용을 구현했습니다. 이 함수는 칼만 필터의 예측 단계 직후 상태(`X`)와 사전 공분산(`P`)을 가져와 `PoseWithCovarianceStamped` 메시지로 조합하여 반환합니다.

5.  **`ekf_localizer.cpp` (`timerCallback`) 수정**:
    *   EKF의 `predictWithDelay` 함수가 호출된 직후, 위에서 만든 `getAPrioriPoseWithCovariance` 함수를 호출하고 그 결과를 `pub_a_priori_pose_cov_` Publisher를 통해 발행하는 코드를 추가했습니다.

---

## 3. 스코어 계산 및 분석 방법

### 계산 공식

스코어는 EKF의 **사전 공분산 행렬(a priori covariance matrix `P_pri`)**의 **Trace(대각합)** 값으로 계산됩니다.

`Score = Trace(P_pri) = σ²_x + σ²_y + σ²_z + σ²_roll + σ²_pitch + σ²_yaw`

*   `σ²`: 각 축의 분산(Variance) 값으로, 불확실성의 제곱을 의미합니다.
*   **의미**: 스코어는 6-DOF(자유도) Pose의 모든 축에 대한 **'총체적인 불확실성의 양'**을 나타내는 단일 값입니다.

### 스코어 분석

이 스코어는 **낮을수록 좋습니다.**

*   **낮은 값 (Good)**: 총 불확실성이 낮다는 의미입니다. EKF가 자신의 예측 위치를 강하게 신뢰하고 있으며, 측위가 매우 안정적인 상태임을 나타냅니다.

*   **높은 값 (Bad)**: 총 불확실성이 높다는 의미입니다. EKF의 예측 위치에 대한 신뢰도가 낮으며, NDT 정보가 없거나 Odometry 드리프트가 누적되어 측위가 불안정해질 가능성이 높은 위험 상태임을 나타냅니다.

*   **핵심 분석법**: 터널 진입과 같이 외부 특징점 정보가 단절되는 구간에서 이 스코어 값이 **지속적으로 상승**하는지 확인하는 것이 중요합니다. 이는 시스템이 점차 불안정해지고 있다는 명확한 예측 신호입니다.

---

## 4. `pose_uncertainty_monitor` 패키지 워크플로우

이 패키지의 전체적인 데이터 흐름은 다음과 같습니다.

1.  **구독 (Subscription)**
    *   `pose_uncertainty_monitor` 노드는 `ekf_localizer`가 발행하는 **사전 공분산 토픽**을 구독합니다.
    *   **토픽 이름**: `/localization/ekf_localizer/output/a_priori_pose_with_covariance`
    *   **메시지 타입**: `geometry_msgs::msg::PoseWithCovarianceStamped`

2.  **연산 (Calculation)**
    *   노드는 메시지를 수신할 때마다, 메시지에 포함된 6x6 `covariance` 행렬의 **Trace(대각합)**를 계산합니다.
    *   `Trace = covariance[0] + covariance[7] + covariance[14] + covariance[21] + covariance[28] + covariance[35]`

3.  **발행 (Publication)**
    *   계산된 Trace 값을 `tier4_debug_msgs::msg::Float64Stamped` 메시지에 담아 발행합니다.
    *   **토픽 이름**: `/localization/pose_uncertainty`
