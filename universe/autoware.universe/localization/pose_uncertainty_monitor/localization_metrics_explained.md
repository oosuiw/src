# Autoware 측위 안정성 지표 분석 (Localization Stability Metrics Explained)

## 1. 세 가지 핵심 지표 (The Three Key Metrics)

Autoware의 측위 성능을 평가하고 모니터링할 때 주로 사용되는 세 가지 핵심 지표는 다음과 같습니다.

1.  **공분산 (Covariance)**: EKF의 최종 출력 (`/localization/pose_with_covariance_stamped`)
2.  **변환 우도 (Transformation Likelihood)**: NDT의 매칭 점수 (`/.../nearest_voxel_transformation_likelihood`)
3.  **정확도 (Accuracy)**: 평가 노드의 결과 (`/localization_accuracy`)

각 지표는 서로 다른 관점에서 측위 상태를 표현하므로, 그 의미와 계산 시점을 정확히 이해하는 것이 중요합니다.

---

## 2. 지표별 의미와 시점 분석

| 구분 (Metric) | 핵심 의미 (Core Meaning) | 관점 (Perspective) | 계산 시점 (Timing) |
| :--- | :--- | :--- | :--- |
| **공분산 (Covariance)** | 현재 상태의 **종합적인 불확실성** | 🔮 **예측적 신뢰도** (Predictive Reliability) | 관측 보정 **후** (After Measurement Update) |
| **변환 우도 (Likelihood)** | 현재 스캔과 지도의 **기하학적 일치도** | ⚙️ **관측 기반 품질** (Observed Quality) | NDT 매칭 **직후** (After NDT Alignment) |
| **정확도 (Accuracy)** | 추정치와 **실제 정답 간의 오차** | ✅ **결과 기반 품질** (Result-based Quality) | 후처리/평가 시 (During Post-processing) |

#### 공분산 (Covariance)에 대한 부연 설명
EKF가 발행하는 공분산은 NDT, Odometry 등 모든 센서 정보를 종합하여 **관측 보정을 마친 후의 최종 불확실성**입니다. 기술적으로는 '사후(a posteriori)' 값이지만, 이 값은 다음 예측 단계의 기반이 되므로 **"곧 측위가 불안정해질 수 있다"**는 것을 알려주는 강력한 **'예측적' 지표**로 해석할 수 있습니다.

---

## 3. 각 지표의 역할과 관계

| 지표 (Metric) | 역할 (Role) | 시점 (Phase) |
| :--- | :--- | :--- |
| **공분산 (Covariance)** | 시스템이 자신의 불확실성을 **예측** (System **predicts** its own uncertainty) | 🔮 **예측 단계** (Predictive) |
| **변환 우도 (Likelihood)** | 실제 센서 관측이 얼마나 잘 맞았는지 **판단** (System **judges** how well the observation fits) | ⚙️ **관측 반영 단계** (Reactive) |
| **정확도 (Accuracy)** | 최종 결과가 정답과 얼마나 일치하는지 **평가** (System **evaluates** the final result) | ✅ **결과 확인 단계** (Evaluative) |

---

## 4. 실용적 활용 방안

이 세 지표의 특성을 이해하면 다음과 같이 실용적으로 활용할 수 있습니다.

1.  **공분산 (예측 지표) 활용**
    *   실시간으로 공분산의 Trace(대각합) 또는 특정 축(예: X축)의 분산 값을 모니터링합니다.
    *   이 값이 설정된 임계치를 초과하면 **"측위 신뢰도 저하 경고 (Localization Unreliable Warning)"** 상태를 발생시킵니다. 이는 문제가 발생하기 전, 불안정해질 '조짐'을 미리 파악하는 것입니다.

2.  **Likelihood (결과 지표)와 조합**
    *   공분산이 커져 '경고' 상태가 된 상황에서, NDT의 `Likelihood` 값까지 낮아진다면 이는 **"실제로 센서 매칭에 실패하여 측위가 불안정해졌다"**는 것을 확인(Confirm)하는 근거가 됩니다.

3.  **종합적 판단**
    *   **공분산**: "앞으로 흔들릴 가능성"을 알려주는 예측적 경고
    *   **Likelihood**: "이미 흔들리고 있다"는 것을 보여주는 현재 상태의 증거
    *   **Accuracy**: "그래서 결국 정답과 이만큼 차이가 났다"는 최종 성적표

이처럼 여러 지표를 조합하면, 단순한 결과 평가를 넘어 시스템의 안정성을 예측하고 사전 대응하는 정교한 모니터링 시스템을 구축할 수 있습니다.

---

## 5. 요약

> 📍 **공분산**은 현재 추정 상태가 얼마나 불확실한지를 예측적으로 나타내는 가장 종합적인 지표입니다.
>
> 📍 **Likelihood** 및 **Accuracy**와 함께 모니터링하면, 전체 측위 시스템의 신뢰도를 정량적으로 평가하고 안정성이 저하될 조짐을 미리 감지하여 대응할 수 있습니다.
