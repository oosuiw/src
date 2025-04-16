// Copyright 2015-2019 Autoware Foundation. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Ryohsuke Mitsudome

#ifndef LANELET2_EXTENSION__REGULATORY_ELEMENTS__AUTOWARE_TRAFFIC_MIRROR_HPP_ //KMS_250313
#define LANELET2_EXTENSION__REGULATORY_ELEMENTS__AUTOWARE_TRAFFIC_MIRROR_HPP_ //KMS_250313

// NOLINTBEGIN(readability-identifier-naming)

#include <lanelet2_extension/regulatory_elements/Forward.hpp>  // Forward.hpp 헤더 포함

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>

namespace lanelet::autoware
{
// AutowareTrafficLight 클래스 정의 (lanelet::TrafficLight 상속) | AutowareTrafficLight -> AutowareTrafficMirror
class AutowareTrafficMirror : public lanelet::RegulatoryElement  // TrafficLight -> TrafficMirror
{
public:
  using Ptr = std::shared_ptr<AutowareTrafficMirror>;  // AutowareTrafficLight의 스마트 포인터 타입 정의 | chg. AutowareTrafficLight -> AutowareTrafficMirror
  static constexpr char RuleName[] = "traffic_mirror";  // 규칙 이름을 "traffic_light"로 설정 | chg. traffic_light -> traffic_mirror
  explicit AutowareTrafficMirror(const lanelet::RegulatoryElementDataPtr & data); // chg AutowareTrafficLight -> AutowareTrafficMirror

  struct AutowareRoleNameString
  {
    static constexpr const char LightBulbs[] = "light_bulbs";  // 도로반사경 전구의 역할 이름 정의
  };

  //! 필수 규칙 파라미터를 이용하여 정지선을 직접 생성하는 정적 함수
  //! 입력 데이터를 수정하여 올바른 태그를 적용할 수도 있음
  static Ptr make(
    Id id, const AttributeMap & attributes, const LineStringsOrPolygons3d & trafficMirrors,
    const Optional<LineString3d> & stopLine = {}, const LineStrings3d & lightBulbs = {})
  {
    return Ptr{new AutowareTrafficMirror(id, attributes, trafficMirrors, stopLine, lightBulbs)};
  }

  /**
   * @brief 관련된 신호등 전구(램프) 가져오기
   * @return 신호등 전구(램프) 목록 반환
   *
   * 여러 개의 신호등 전구가 존재할 수 있지만 동일한 신호를 표시해야 함.
   */
  [[nodiscard]] ConstLineStrings3d lightBulbs() const;

  /**
   * @brief 새로운 신호등 전구(램프) 추가
   * @param primitive 추가할 신호등 전구(램프)
   *
   * 신호등 전구는 선(line) 형태로 표현되며, 각 점(point)은 개별 전구의 위치를 나타냄.
   */
  void addLightBulbs(const LineStringOrPolygon3d & primitive);

  /**
   * @brief 특정 신호등 전구(램프) 제거
   * @param primitive 제거할 전구(램프)
   * @return true(전구가 존재했고 삭제됨), false(전구가 존재하지 않음)
   */
  bool removeLightBulbs(const LineStringOrPolygon3d & primitive);

private:
  // Lanelet2에서 이 객체를 생성할 수 있도록 하기 위해 필요한 부분
  friend class lanelet::RegisterRegulatoryElement<AutowareTrafficLight>;
  AutowareTrafficMirror( // chg AutowareTrafficLight -> AutowareTrafficMirror
    Id id, const AttributeMap & attributes, const LineStringsOrPolygons3d & trafficMirrors,
    const Optional<LineString3d> & stopLine, const LineStrings3d & lightBulbs);
};

// Lanelet2에서 AutowareTrafficLight 클래스를 등록
static lanelet::RegisterRegulatoryElement<AutowareTrafficMirror> regAutowareTrafficMirror; // chg regAutowareTraffic -> regAutowareTrafficMirror

}  // namespace lanelet::autoware

// NOLINTEND(readability-identifier-naming)

#endif  // LANELET2_EXTENSION__REGULATORY_ELEMENTS__AUTOWARE_TRAFFIC_MIRROR_HPP_ //KMS_250313
