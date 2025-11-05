# NDT 동적 파라미터 제어 가이드

## 목차
1. [개요](#개요)
2. [파라미터 관계 이해하기](#파라미터-관계-이해하기)
3. [ROS 2 동적 파라미터 시스템](#ros-2-동적-파라미터-시스템)
4. [Adaptive NDT Controller의 동작 원리](#adaptive-ndt-controller의-동작-원리)
5. [수동 파라미터 제어 방법](#수동-파라미터-제어-방법)
6. [파라미터 동기화 및 검증](#파라미터-동기화-및-검증)
7. [트러블슈팅](#트러블슈팅)
8. [고급: 코드 레벨 이해](#고급-코드-레벨-이해)

---

## 개요

이 문서는 `adaptive_ndt_controller`가 **어떻게** NDT 스캔 매처의 파라미터를 동적으로 제어하는지,
그리고 사용자가 수동으로 파라미터를 제어하는 방법을 설명합니다.

### 핵심 개념
- **동적 파라미터**: 노드 실행 중에 변경 가능한 파라미터
- **Parameter Client**: 다른 노드의 파라미터를 변경하는 ROS 2 메커니즘
- **Base vs Adaptive**: 기준값(base)과 적응적으로 계산된 값의 관계

---

## 파라미터 관계 이해하기

### 1. 세 가지 파라미터 설정 위치

#### A. NDT Scan Matcher 초기 설정
**위치**: `/home/sws/autoware/src/universe/autoware.universe/localization/ndt_scan_matcher/config/ndt_scan_matcher.param.yaml`

```yaml
/**:
  ros__parameters:
    step_size: 0.1
    max_iterations: 30
    resolution: 2.0
```

**역할**: NDT 노드 시작 시 **초기값**으로 로드됨

#### B. Adaptive Controller Base 설정
**위치**: `/home/sws/autoware/src/universe/autoware.universe/localization/adaptive_ndt_controller/config/adaptive_ndt_controller.param.yaml`

```yaml
/adaptive_ndt_controller:
  ros__parameters:
    base_step_size: 0.1
    base_max_iterations: 30
    base_resolution: 2.0
```

**역할**: 적응 제어의 **기준값** (불확실성이 0일 때 사용)

#### C. 실행 중 동적 변경
**방법**: ROS 2 parameter service 또는 adaptive controller를 통한 자동 변경

**역할**: 실시간으로 NDT 노드의 파라미터를 업데이트

---

### 2. 자동 파라미터 동기화 ✨ (NEW!)

**v1.1부터 추가된 기능**: `auto_sync_base_parameters`

이제 수동으로 A와 B를 일치시킬 필요가 없습니다!

#### 자동 동기화 활성화 (기본값: true)

```yaml
# adaptive_ndt_controller.param.yaml
/adaptive_ndt_controller:
  ros__parameters:
    # 자동 동기화 활성화 (권장)
    auto_sync_base_parameters: true

    # 아래 값들은 fallback으로만 사용됨
    # (NDT에서 읽어올 수 없을 때만 사용)
    base_step_size: 0.1
    base_max_iterations: 30
    base_resolution: 2.0
```

#### 동작 원리

```
┌──────────────────────────────────┐
│  1. NDT 노드 시작                │
│     step_size: 0.1               │
│     max_iterations: 30           │
│     resolution: 2.0              │
└──────────────┬───────────────────┘
               │
               ▼
┌──────────────────────────────────┐
│  2. Adaptive Controller 시작     │
│                                  │
│  auto_sync_base_parameters: true │
└──────────────┬───────────────────┘
               │
               ├─► get_parameters("step_size")
               ├─► get_parameters("max_iterations")
               ├─► get_parameters("resolution")
               │
               ▼
┌──────────────────────────────────┐
│  3. Base 파라미터 자동 설정      │
│     base_step_size: 0.1 (from NDT)│
│     base_max_iterations: 30      │
│     base_resolution: 2.0         │
└──────────────────────────────────┘
    ✅ 완벽한 동기화!
```

#### 시작 로그 확인

자동 동기화가 성공하면 다음과 같은 로그가 출력됩니다:

```
[adaptive_ndt_controller]: Waiting for NDT parameter service...
[adaptive_ndt_controller]: NDT parameter service is available.
[adaptive_ndt_controller]: Auto-syncing base parameters from NDT node...
[adaptive_ndt_controller]: Base parameters synchronized successfully!
[adaptive_ndt_controller]:   Synced step_size: 0.100
[adaptive_ndt_controller]:   Synced max_iterations: 30
[adaptive_ndt_controller]:   Synced resolution: 2.000
```

#### Fallback: 수동 설정 (auto_sync_base_parameters: false)

자동 동기화를 비활성화하면 기존 방식대로 동작합니다:

```yaml
/adaptive_ndt_controller:
  ros__parameters:
    # 자동 동기화 비활성화
    auto_sync_base_parameters: false

    # 수동으로 NDT와 일치시켜야 함!
    base_step_size: 0.1        # ndt_scan_matcher.param.yaml과 일치!
    base_max_iterations: 30
    base_resolution: 2.0
```

**주의**: 수동 설정 시 A와 B를 반드시 일치시켜야 합니다!

**문제 발생 예시**:
```yaml
# ndt_scan_matcher.param.yaml (A)
step_size: 0.1        # NDT 초기값

# adaptive_ndt_controller.param.yaml (B)
auto_sync_base_parameters: false  # 수동 설정
base_step_size: 0.2   # Controller 기준값 (다름!)
```

→ NDT 노드 시작: `step_size = 0.1`
→ Adaptive controller 시작: 불확실성 0일 때 `0.2`로 변경 시도
→ **의도하지 않은 초기 파라미터 변경** 발생!

---

### 3. 파라미터 흐름도

```
┌──────────────────────────────────┐
│  ndt_scan_matcher.param.yaml     │
│  step_size: 0.1                  │
│  max_iterations: 30              │
└─────────────┬────────────────────┘
              │ 1. 노드 시작 시 로드
              ▼
┌──────────────────────────────────┐
│  NDT Scan Matcher Node           │
│  현재 파라미터:                   │
│  - step_size: 0.1                │
│  - max_iterations: 30            │
└─────────────┬────────────────────┘
              │
              │ 2. 실행 중 업데이트
              │
┌─────────────▼────────────────────┐
│  Adaptive NDT Controller         │
│                                  │
│  base_step_size: 0.1             │◄──── 기준값 (A와 일치!)
│  gain_step_size: 0.5             │◄──── 게인
│                                  │
│  불확실성 수신:                   │
│  std_x = 0.2, std_y = 0.15       │
│                                  │
│  계산:                           │
│  pos_unc = sqrt(0.2² + 0.15²)   │
│          = 0.25                  │
│                                  │
│  new_step_size =                 │
│    0.1 + 0.5 × 0.25 = 0.225      │
└─────────────┬────────────────────┘
              │ 3. Parameter Client
              │    (set_parameters)
              ▼
┌──────────────────────────────────┐
│  NDT Scan Matcher Node           │
│  업데이트된 파라미터:              │
│  - step_size: 0.225              │◄──── 동적 변경!
│  - max_iterations: 35            │◄──── 동적 변경!
└──────────────────────────────────┘
```

---

## ROS 2 동적 파라미터 시스템

### 1. ROS 2 Parameter Service

모든 ROS 2 노드는 자동으로 다음 서비스를 제공합니다:

```bash
# 노드의 파라미터 서비스 확인
ros2 service list | grep ndt_scan_matcher

# 출력 예시:
/ndt_scan_matcher/describe_parameters
/ndt_scan_matcher/get_parameter_types
/ndt_scan_matcher/get_parameters
/ndt_scan_matcher/list_parameters
/ndt_scan_matcher/set_parameters           # ← 이것을 사용!
/ndt_scan_matcher/set_parameters_atomically
```

### 2. Parameter Client API

`adaptive_ndt_controller`는 C++ API를 사용합니다:

```cpp
// adaptive_ndt_controller_node.cpp 참조

// 1. Parameter Client 생성
ndt_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
  this, ndt_node_name_);  // ndt_node_name_ = "ndt_scan_matcher"

// 2. 서비스 대기
ndt_param_client_->wait_for_service(std::chrono::seconds(5));

// 3. 파라미터 설정
std::vector<rclcpp::Parameter> new_params;
new_params.push_back(rclcpp::Parameter("step_size", 0.225));
new_params.push_back(rclcpp::Parameter("max_iterations", 35));

auto future_result = ndt_param_client_->set_parameters(new_params);

// 4. 결과 확인
auto results = future_result.get();
for (auto & result : results) {
  if (!result.successful) {
    RCLCPP_WARN(this->get_logger(), "Failed: %s", result.reason.c_str());
  }
}
```

---

## Adaptive NDT Controller의 동작 원리

### 1. 초기화 단계 (자동 동기화 포함)

```cpp
// adaptive_ndt_controller_node.cpp:23-100

AdaptiveNdtControllerNode::AdaptiveNdtControllerNode(...)
{
  // 1. Base 파라미터 로드 (fallback 값)
  base_step_size_ = declare_parameter<double>("base_step_size", 0.1);
  base_max_iterations_ = declare_parameter<int>("base_max_iterations", 30);
  base_resolution_ = declare_parameter<double>("base_resolution", 2.0);

  // 2. Gain 값 로드
  gain_step_size_ = declare_parameter<double>("gain_step_size", 0.5);
  gain_max_iterations_ = declare_parameter<double>("gain_max_iterations", 10.0);

  // 3. 자동 동기화 플래그 로드
  auto_sync_base_parameters_ = declare_parameter<bool>("auto_sync_base_parameters", true);

  // 4. Parameter Client 생성
  ndt_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    this, ndt_node_name_);  // "ndt_scan_matcher"

  // 5. NDT 서비스 대기
  if (!ndt_param_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(this->get_logger(), "NDT service not available");
  } else {
    // 6. 자동 동기화 실행 (활성화된 경우)
    if (auto_sync_base_parameters_) {
      RCLCPP_INFO(this->get_logger(), "Auto-syncing base parameters from NDT node...");
      if (sync_base_parameters_from_ndt()) {
        RCLCPP_INFO(this->get_logger(), "Base parameters synchronized successfully!");
        // 동기화된 값이 base_* 변수에 저장됨
      } else {
        RCLCPP_WARN(this->get_logger(),
                   "Failed to sync. Using configured fallback values.");
      }
    }
  }

  // 7. 불확실성 토픽 구독
  sub_uncertainty_vector_ = create_subscription<...>(
    "/localization/diagnostics/uncertainty_vector", ...);
}
```

### 1-1. 자동 동기화 함수 (NEW!)

```cpp
// adaptive_ndt_controller_node.cpp:262-334

bool AdaptiveNdtControllerNode::sync_base_parameters_from_ndt()
{
  // 1. 서비스 가용성 확인
  if (!ndt_param_client_->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "NDT parameter service is not ready.");
    return false;
  }

  // 2. NDT에서 파라미터 가져오기 (get_parameters)
  std::vector<std::string> param_names = {"step_size", "max_iterations", "resolution"};
  auto future_result = ndt_param_client_->get_parameters(param_names);

  // 3. 타임아웃 대기 (2초)
  if (future_result.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(), "Timeout waiting for NDT parameters.");
    return false;
  }

  try {
    auto params = future_result.get();

    // 4. 파라미터 추출 및 저장
    for (const auto & param : params) {
      if (param.get_name() == "step_size") {
        base_step_size_ = param.as_double();
      }
      else if (param.get_name() == "max_iterations") {
        base_max_iterations_ = param.as_int();
      }
      else if (param.get_name() == "resolution") {
        base_resolution_ = param.as_double();
      }
    }

    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    return false;
  }
}
```

### 2. 실행 중 적응 단계

```cpp
// adaptive_ndt_controller_node.cpp:100-191

void AdaptiveNdtControllerNode::on_uncertainty_vector(...)
{
  // 1. 불확실성 수신
  const double std_x = msg->std_x;
  const double std_y = msg->std_y;
  const double std_yaw = msg->std_yaw;

  // 2. 적응형 파라미터 계산
  const double position_uncertainty = sqrt(std_x² + std_y²);

  double new_step_size = base_step_size_ + gain_step_size_ * position_uncertainty;
  new_step_size = std::clamp(new_step_size, 0.05, 1.0);  // 범위 제한

  int new_max_iterations = base_max_iterations_ +
                           static_cast<int>(gain_max_iterations_ * std_yaw);
  new_max_iterations = std::clamp(new_max_iterations, 10, 100);

  // 3. NDT 파라미터 업데이트
  std::vector<rclcpp::Parameter> new_params;
  new_params.push_back(rclcpp::Parameter("step_size", new_step_size));
  new_params.push_back(rclcpp::Parameter("max_iterations", new_max_iterations));

  set_ndt_parameters(new_params);  // Parameter Client로 전송
}
```

### 3. 파라미터 전송 단계

```cpp
// adaptive_ndt_controller_node.cpp:193-224

bool AdaptiveNdtControllerNode::set_ndt_parameters(...)
{
  // 1. 서비스 가용성 확인
  if (!ndt_param_client_->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(), "NDT service not ready");
    return false;
  }

  // 2. 비동기 전송
  auto future_result = ndt_param_client_->set_parameters(parameters);

  // 3. 타임아웃 대기 (100ms)
  if (future_result.wait_for(std::chrono::milliseconds(100)) ==
      std::future_status::ready) {
    auto results = future_result.get();

    // 4. 결과 확인
    for (size_t i = 0; i < results.size(); ++i) {
      if (!results[i].successful) {
        RCLCPP_WARN(this->get_logger(),
                   "Failed to set %s: %s",
                   parameters[i].get_name().c_str(),
                   results[i].reason.c_str());
        return false;
      }
    }
    return true;
  } else {
    RCLCPP_WARN(this->get_logger(), "Timeout");
    return false;
  }
}
```

---

## 수동 파라미터 제어 방법

### 1. 현재 파라미터 확인

```bash
# NDT의 모든 파라미터 보기
ros2 param list /ndt_scan_matcher

# 특정 파라미터 값 확인
ros2 param get /ndt_scan_matcher step_size
ros2 param get /ndt_scan_matcher max_iterations
ros2 param get /ndt_scan_matcher resolution
```

**출력 예시**:
```
Double value is: 0.1
Integer value is: 30
Double value is: 2.0
```

### 2. 실행 중 파라미터 변경

#### A. 단일 파라미터 변경

```bash
# step_size 변경
ros2 param set /ndt_scan_matcher step_size 0.2

# max_iterations 변경
ros2 param set /ndt_scan_matcher max_iterations 40

# resolution 변경
ros2 param set /ndt_scan_matcher resolution 1.5
```

**출력**:
```
Set parameter successful
```

#### B. 여러 파라미터 한번에 변경

ROS 2 CLI로는 불가능, Python 스크립트 사용:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

rclpy.init()
node = Node('param_setter')

# Parameter Client 생성
param_client = node.create_client(
    '/ndt_scan_matcher',
    'set_parameters'
)

# 파라미터 설정
params = [
    Parameter(
        name='step_size',
        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.2)
    ),
    Parameter(
        name='max_iterations',
        value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=40)
    )
]

# 전송
future = param_client.call_async(params)
rclpy.spin_until_future_complete(node, future)

print(future.result())
```

### 3. 변경 사항 실시간 모니터링

```bash
# 파라미터 변경 이벤트 구독
ros2 param event /ndt_scan_matcher
```

**출력 예시**:
```
parameter changed:
  name: step_size
  value: 0.225
parameter changed:
  name: max_iterations
  value: 35
```

### 4. Adaptive Controller 로그 확인

```bash
# Controller의 파라미터 변경 로그 보기
ros2 topic echo /rosout | grep "Adaptive"
```

**출력 예시**:
```
[adaptive_ndt_controller]: Adaptive step_size: 0.2250 (pos_uncertainty=0.2500)
[adaptive_ndt_controller]: Adaptive max_iterations: 35 (std_yaw=0.0500)
```

---

## 파라미터 동기화 및 검증

### 1. 파라미터 일치성 검증 스크립트

```bash
#!/bin/bash
# check_param_sync.sh

echo "=== NDT Scan Matcher 현재 파라미터 ==="
ros2 param get /ndt_scan_matcher step_size
ros2 param get /ndt_scan_matcher max_iterations
ros2 param get /ndt_scan_matcher resolution

echo ""
echo "=== Adaptive Controller Base 파라미터 ==="
ros2 param get /adaptive_ndt_controller base_step_size
ros2 param get /adaptive_ndt_controller base_max_iterations
ros2 param get /adaptive_ndt_controller base_resolution

echo ""
echo "=== Adaptive Controller Gain 파라미터 ==="
ros2 param get /adaptive_ndt_controller gain_step_size
ros2 param get /adaptive_ndt_controller gain_max_iterations
```

### 2. 자동 동기화 스크립트

```bash
#!/bin/bash
# sync_base_params.sh

# NDT에서 현재 값 읽기
NDT_STEP=$(ros2 param get /ndt_scan_matcher step_size | awk '{print $NF}')
NDT_ITER=$(ros2 param get /ndt_scan_matcher max_iterations | awk '{print $NF}')
NDT_RES=$(ros2 param get /ndt_scan_matcher resolution | awk '{print $NF}')

# Adaptive Controller에 동일한 base 값 설정
ros2 param set /adaptive_ndt_controller base_step_size $NDT_STEP
ros2 param set /adaptive_ndt_controller base_max_iterations $NDT_ITER
ros2 param set /adaptive_ndt_controller base_resolution $NDT_RES

echo "동기화 완료!"
echo "base_step_size: $NDT_STEP"
echo "base_max_iterations: $NDT_ITER"
echo "base_resolution: $NDT_RES"
```

### 3. 파라미터 변경 히스토리 기록

```bash
# 모든 파라미터 변경을 파일로 기록
ros2 param event /ndt_scan_matcher > ndt_param_history.log &
ros2 param event /adaptive_ndt_controller > controller_param_history.log &
```

---

## 트러블슈팅

### 문제 1: "NDT parameter service not available"

**원인**: NDT 노드가 실행되지 않았거나, 노드 이름이 다름

**해결**:
```bash
# NDT 노드 실행 확인
ros2 node list | grep ndt

# 노드 이름 확인
ros2 node list

# adaptive_ndt_controller의 ndt_node_name 파라미터 확인
ros2 param get /adaptive_ndt_controller ndt_node_name

# 필요시 수정
ros2 param set /adaptive_ndt_controller ndt_node_name "실제_노드_이름"
```

---

### 문제 2: "Failed to set parameter"

**원인**: NDT 노드가 해당 파라미터를 거부 (읽기 전용 등)

**해결**:
```bash
# 파라미터가 변경 가능한지 확인
ros2 param describe /ndt_scan_matcher step_size
```

**출력**:
```
Parameter name: step_size
  Type: double
  Description: ...
  Constraints:
    Read only: false  # ← false여야 함!
```

**코드 수정 필요**: NDT 노드에서 파라미터를 `declare_parameter`할 때 읽기 전용으로 설정했다면, 소스 코드 수정 필요

---

### 문제 3: 파라미터 변경이 적용되지 않음

**원인**: NDT 내부에서 파라미터를 캐싱하고 콜백 미구현

**확인**:
```cpp
// ndt_scan_matcher 코드에서 확인 필요
// 파라미터 변경 콜백이 있는지?

// 예시: 올바른 구현
this->add_on_set_parameters_callback(
  std::bind(&NDTScanMatcher::paramCallback, this, std::placeholders::_1)
);

rcl_interfaces::msg::SetParametersResult paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    if (param.get_name() == "step_size") {
      // 내부 변수 업데이트
      step_size_ = param.as_double();
    }
  }
  // ...
}
```

**해결**: NDT 노드가 동적 파라미터를 지원하지 않으면, 소스 코드에 콜백 추가 필요

---

### 문제 4: Base 파라미터와 NDT 초기값 불일치

**확인**:
```bash
# 두 값 비교
ros2 param get /ndt_scan_matcher step_size
ros2 param get /adaptive_ndt_controller base_step_size
```

**해결**:
```bash
# 수동 동기화
ros2 param set /adaptive_ndt_controller base_step_size 0.1

# 또는 설정 파일 수정 후 재시작
nano config/adaptive_ndt_controller.param.yaml
```

---

## 고급: 코드 레벨 이해

### 1. NDT가 동적 파라미터를 지원하는지 확인

```bash
# NDT 소스 코드 확인
grep -r "add_on_set_parameters_callback" \
  /home/sws/autoware/src/universe/autoware.universe/localization/ndt_scan_matcher/
```

**출력이 있으면**: 동적 파라미터 지원 ✅
**출력이 없으면**: 동적 파라미터 미지원 ❌ → 코드 수정 필요

---

### 2. NDT에 동적 파라미터 지원 추가하기 (고급)

만약 NDT가 동적 파라미터를 지원하지 않는다면, 다음 코드를 추가:

#### `ndt_scan_matcher_core.hpp`에 추가:
```cpp
private:
  // 파라미터 콜백 선언
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  // 콜백 핸들러 저장
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
```

#### `ndt_scan_matcher_core.cpp` 생성자에 추가:
```cpp
NDTScanMatcher::NDTScanMatcher() : Node("ndt_scan_matcher")
{
  // 기존 코드...

  // 파라미터 콜백 등록
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&NDTScanMatcher::onSetParameters, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Dynamic parameter callback registered");
}
```

#### 콜백 함수 구현:
```cpp
rcl_interfaces::msg::SetParametersResult NDTScanMatcher::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    if (param.get_name() == "step_size") {
      double new_value = param.as_double();

      // 범위 검증
      if (new_value < 0.01 || new_value > 2.0) {
        result.successful = false;
        result.reason = "step_size out of range [0.01, 2.0]";
        return result;
      }

      // NDT 객체에 적용
      std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);
      if (ndt_ptr_) {
        ndt_ptr_->setStepSize(new_value);
        RCLCPP_INFO(this->get_logger(), "step_size updated to %.3f", new_value);
      }
    }
    else if (param.get_name() == "max_iterations") {
      int new_value = param.as_int();

      if (new_value < 1 || new_value > 200) {
        result.successful = false;
        result.reason = "max_iterations out of range [1, 200]";
        return result;
      }

      std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);
      if (ndt_ptr_) {
        ndt_ptr_->setMaximumIterations(new_value);
        RCLCPP_INFO(this->get_logger(), "max_iterations updated to %d", new_value);
      }
    }
    else if (param.get_name() == "resolution") {
      double new_value = param.as_double();

      if (new_value < 0.5 || new_value > 10.0) {
        result.successful = false;
        result.reason = "resolution out of range [0.5, 10.0]";
        return result;
      }

      // Resolution 변경은 맵 재구성 필요 → 주의!
      std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);
      if (ndt_ptr_) {
        ndt_ptr_->setResolution(new_value);
        RCLCPP_WARN(this->get_logger(),
                   "resolution updated to %.2f (map reconstruction triggered!)",
                   new_value);
      }
    }
  }

  return result;
}
```

---

### 3. 파라미터 변경 효과 즉시 확인

```bash
# 터미널 1: NDT 로그 모니터링
ros2 topic echo /rosout | grep ndt_scan_matcher

# 터미널 2: 파라미터 변경
ros2 param set /ndt_scan_matcher step_size 0.3

# 터미널 3: 적용 확인
ros2 param get /ndt_scan_matcher step_size
```

---

## 요약: 파라미터 제어 체크리스트

### 시작 전 확인
- [ ] NDT 노드가 실행 중인지 확인
- [ ] `adaptive_ndt_controller.param.yaml`에서 `auto_sync_base_parameters: true` 설정 (권장)
- [ ] 자동 동기화 비활성화 시에만: `adaptive_ndt_controller.param.yaml`의 base 값이 NDT와 일치하는지 확인
- [ ] NDT가 동적 파라미터를 지원하는지 확인 (코드 검사)

### 운영 중 모니터링
- [ ] `ros2 param get`으로 현재 파라미터 주기적 확인
- [ ] `/rosout` 토픽에서 adaptive controller 로그 모니터링
- [ ] 파라미터 변경 히스토리 기록

### 문제 발생 시
- [ ] Parameter service 가용성 확인 (`ros2 service list`)
- [ ] 노드 이름 일치 확인
- [ ] 파라미터 읽기/쓰기 권한 확인 (`ros2 param describe`)
- [ ] NDT 소스 코드에서 콜백 구현 확인

---

**작성일**: 2025-10-18
**버전**: 1.1 (자동 파라미터 동기화 기능 추가)
**관련 파일**:
- `adaptive_ndt_controller_node.hpp:86` (sync_base_parameters_from_ndt 선언)
- `adaptive_ndt_controller_node.cpp:72-85` (자동 동기화 실행)
- `adaptive_ndt_controller_node.cpp:262-334` (sync_base_parameters_from_ndt 구현)
- `adaptive_ndt_controller_node.cpp:194-230` (파라미터 전송 로직)
- `adaptive_ndt_controller.param.yaml:67` (auto_sync_base_parameters 설정)
