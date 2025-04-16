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

// NOLINTBEGIN(readability-identifier-naming)

#include "lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp" // AutowareTrafficLight 클래스의 정의 포함

#include <boost/variant.hpp>

#include <lanelet2_core/primitives/RegulatoryElement.h>

#include <algorithm>  // std::find 사용을 위해 포함
#include <memory>     // 스마트 포인터 사용을 위해 포함
#include <utility>    // std::make_pair 사용을 위해 포함
#include <vector>     // std::vector 사용을 위해 포함

namespace lanelet::autoware
{
namespace
{
/**
 * @brief 특정 규제 요소 멤버에서 주어진 primitive를 찾아 제거하는 함수
 * @tparam T primitive 타입
 * @param primitive 제거할 primitive 객체
 * @param member 제거할 요소가 포함된 규칙 파라미터 멤버 (RuleParameters)
 * @return true(제거 성공), false(primitive가 존재하지 않음)
 */
template <typename T>
bool findAndErase(const T & primitive, RuleParameters * member)
{
  if (member == nullptr) {  // 멤버가 nullptr이면 오류 메시지를 출력하고 false 반환
    std::cerr << __FUNCTION__ << ": member is null pointer";
    return false;
  }
  auto it = std::find(member->begin(), member->end(), RuleParameter(primitive)); // primitive가 존재하는지 탐색
  if (it == member->end()) {  // 찾지 못한 경우 false 반환
    return false;
  }
  member->erase(it);  // primitive를 리스트에서 제거
  return true;
}

/**
 * @brief 주어진 primitive 벡터를 RuleParameters 형식으로 변환
 * @tparam T 변환할 primitive 타입
 * @param primitives 변환할 primitive 벡터
 * @return RuleParameters로 변환된 값
 */
template <typename T>
RuleParameters toRuleParameters(const std::vector<T> & primitives)
{
  auto cast_func = [](const auto & elem) { return static_cast<RuleParameter>(elem); };
  return utils::transform(primitives, cast_func);  // 벡터의 각 요소를 RuleParameter로 변환
}

/**
 * @brief LineStringOrPolygon3d 벡터를 RuleParameters로 변환하는 특수화된 함수
 * @param primitives 변환할 primitive 벡터
 * @return RuleParameters로 변환된 값
 */
template <>
RuleParameters toRuleParameters(const std::vector<LineStringOrPolygon3d> & primitives)
{
  auto cast_func = [](const auto & elem) { return elem.asRuleParameter(); };  // RuleParameter로 변환
  return utils::transform(primitives, cast_func);
}

/**
 * @brief 규제 요소 맵에서 특정 역할(RoleName)에 해당하는 LineString 또는 Polygon을 가져옴
 * @param paramsMap 규제 요소 맵
 * @param role 검색할 역할(RoleName)
 * @return 해당 역할에 속하는 LineStringsOrPolygons3d 객체
 */
LineStringsOrPolygons3d getLsOrPoly(const RuleParameterMap & paramsMap, RoleName role)
{
  auto params = paramsMap.find(role);  // 역할(role)에 해당하는 항목 찾기
  if (params == paramsMap.end()) {  // 찾지 못하면 빈 리스트 반환
    return {};
  }
  LineStringsOrPolygons3d result;
  for (auto & param : params->second) {  // 해당 역할의 모든 요소를 순회
    auto l = boost::get<LineString3d>(&param);
    if (l != nullptr) {  // LineString3d 타입이면 결과에 추가
      result.push_back(*l);
    }
    auto p = boost::get<Polygon3d>(&param);
    if (p != nullptr) {  // Polygon3d 타입이면 결과에 추가
      result.push_back(*p);
    }
  }
  return result;
}

/**
 * @brief getLsOrPoly 함수를 ConstLineStringsOrPolygons3d 타입으로 변환하는 함수
 * @param params 규제 요소 맵
 * @param role 검색할 역할(RoleName)
 * @return 해당 역할에 속하는 ConstLineStringsOrPolygons3d 객체
 */
[[maybe_unused]] ConstLineStringsOrPolygons3d getConstLsOrPoly(
  const RuleParameterMap & params, RoleName role)
{
  auto cast_func = [](auto & lsOrPoly) {
    return static_cast<ConstLineStringOrPolygon3d>(lsOrPoly);
  };
  return utils::transform(getLsOrPoly(params, role), cast_func);
}

/**
 * @brief Autoware 신호등 데이터를 생성하는 함수
 * @param id 규제 요소 ID
 * @param attributes 속성 맵
 * @param trafficLights 신호등 정보
 * @param stopLine 정지선 정보 (선택 사항)
 * @param lightBulbs 신호등 전구 정보 (선택 사항)
 * @return 생성된 RegulatoryElementData 스마트 포인터
 */
[[maybe_unused]] RegulatoryElementDataPtr constructAutowareTrafficLightData(
  Id id, const AttributeMap & attributes, const LineStringsOrPolygons3d & trafficLights,
  const Optional<LineString3d> & stopLine, const LineStrings3d & lightBulbs)
{
  RuleParameterMap rpm = {{RoleNameString::Refers, toRuleParameters(trafficLights)}};

  if (!!stopLine) {  // stopLine이 존재하면 추가
    RuleParameters rule_parameters = {*stopLine};
    rpm.insert(std::make_pair(RoleNameString::RefLine, rule_parameters));
  }
  if (!lightBulbs.empty()) {  // lightBulbs가 존재하면 추가
    rpm.insert(std::make_pair(
      AutowareTrafficLight::AutowareRoleNameString::LightBulbs, toRuleParameters(lightBulbs)));
  }

  auto data = std::make_shared<RegulatoryElementData>(id, rpm, attributes);
  data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
  data->attributes[AttributeName::Subtype] = AttributeValueString::TrafficLight;
  return data;
}
}  // namespace

/**
 * @brief AutowareTrafficLight 클래스 생성자 (RegulatoryElementDataPtr 사용)
 * @param data 규제 요소 데이터 포인터
 */
AutowareTrafficLight::AutowareTrafficLight(const RegulatoryElementDataPtr & data)
: TrafficLight(data)
{
}

/**
 * @brief AutowareTrafficLight 클래스 생성자
 * @param id 규제 요소 ID
 * @param attributes 속성 맵
 * @param trafficLights 신호등 정보
 * @param stopLine 정지선 정보 (선택 사항)
 * @param lightBulbs 신호등 전구 정보 (선택 사항)
 */
AutowareTrafficLight::AutowareTrafficLight(
  Id id, const AttributeMap & attributes, const LineStringsOrPolygons3d & trafficLights,
  const Optional<LineString3d> & stopLine, const LineStrings3d & lightBulbs)
: TrafficLight(id, attributes, trafficLights, stopLine)  // 부모 클래스 생성자 호출
{
  for (const auto & lightBulb : lightBulbs) {  // lightBulbs가 존재하면 추가
    addLightBulbs(lightBulb);
  }
}

/**
 * @brief 신호등 전구 정보를 반환하는 함수
 * @return 신호등 전구 리스트
 */
ConstLineStrings3d AutowareTrafficLight::lightBulbs() const
{
  return getParameters<ConstLineString3d>(AutowareRoleNameString::LightBulbs);
}

/**
 * @brief 신호등 전구를 추가하는 함수
 * @param primitive 추가할 신호등 전구 객체
 */
void AutowareTrafficLight::addLightBulbs(const LineStringOrPolygon3d & primitive)
{
  parameters()[AutowareRoleNameString::LightBulbs].emplace_back(primitive.asRuleParameter());
}

/**
 * @brief 특정 신호등 전구를 제거하는 함수
 * @param primitive 제거할 신호등 전구 객체
 * @return true(제거 성공), false(제거할 전구가 없음)
 */
bool AutowareTrafficLight::removeLightBulbs(const LineStringOrPolygon3d & primitive)
{
  return findAndErase(
    primitive.asRuleParameter(), &parameters().find(AutowareRoleNameString::LightBulbs)->second);
}

}  // namespace lanelet::autoware

// NOLINTEND(readability-identifier-naming)
