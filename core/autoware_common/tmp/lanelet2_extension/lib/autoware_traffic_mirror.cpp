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

#include "lanelet2_extension/regulatory_elements/autoware_traffic_mirror.hpp"

#include <boost/variant.hpp>
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace lanelet::autoware
{
namespace
{
template <typename T>
bool findAndErase(const T & primitive, RuleParameters * member)
{
  if (member == nullptr) {
    std::cerr << __FUNCTION__ << ": member is null pointer";
    return false;
  }
  auto it = std::find(member->begin(), member->end(), RuleParameter(primitive));
  if (it == member->end()) {
    return false;
  }
  member->erase(it);
  return true;
}

template <typename T>
RuleParameters toRuleParameters(const std::vector<T> & primitives)
{
  auto cast_func = [](const auto & elem) { return static_cast<RuleParameter>(elem); };
  return utils::transform(primitives, cast_func);
}

template <>
RuleParameters toRuleParameters(const std::vector<LineStringOrPolygon3d> & primitives)
{
  auto cast_func = [](const auto & elem) { return elem.asRuleParameter(); };
  return utils::transform(primitives, cast_func);
}

LineStringsOrPolygons3d getLsOrPoly(const RuleParameterMap & paramsMap, RoleName role)
{
  auto params = paramsMap.find(role);
  if (params == paramsMap.end()) {
    return {};
  }
  LineStringsOrPolygons3d result;
  for (auto & param : params->second) {
    try {
      auto l = boost::get<LineString3d>(param);
      result.push_back(l);
    } catch (const boost::bad_get &) {
      // param이 LineString3d가 아닌 경우 무시
    }
    try {
      auto p = boost::get<Polygon3d>(param);
      result.push_back(p);
    } catch (const boost::bad_get &) {
      // param이 Polygon3d가 아닌 경우 무시
    }
  }
  return result;
}

[[maybe_unused]] ConstLineStringsOrPolygons3d getConstLsOrPoly(
  const RuleParameterMap & params, RoleName role)
{
  auto cast_func = [](auto & lsOrPoly) {
    return static_cast<ConstLineStringOrPolygon3d>(lsOrPoly);
  };
  return utils::transform(getLsOrPoly(params, role), cast_func);
}

RegulatoryElementDataPtr constructAutowareTrafficMirrorData(
  Id id, const AttributeMap & attributes, const LineStringsOrPolygons3d & trafficMirrors,
  const Optional<LineString3d> & stopLine, const LineStrings3d & lightBulbs)
{
  RuleParameterMap rpm = {{RoleNameString::Refers, toRuleParameters(trafficMirrors)}};

  if (stopLine) {
    RuleParameters rule_parameters = {*stopLine};
    rpm.insert(std::make_pair(RoleNameString::RefLine, rule_parameters));
  }
  if (!lightBulbs.empty()) {
    rpm.insert(std::make_pair(
      AutowareTrafficMirror::AutowareRoleNameString::LightBulbs, toRuleParameters(lightBulbs)));
  }

  auto data = std::make_shared<RegulatoryElementData>(id, rpm, attributes);
  data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
  data->attributes[AttributeName::Subtype] = AttributeValueString::TrafficMirror;
  return data;
}
}  // namespace

// RegulatoryElementDataPtr을 사용하는 생성자
AutowareTrafficMirror::AutowareTrafficMirror(const lanelet::RegulatoryElementDataPtr & data)
  : RegulatoryElement(data)
{
}

// 단일 생성자 정의
AutowareTrafficMirror::AutowareTrafficMirror(
  Id id, const AttributeMap & attributes, const LineStringsOrPolygons3d & trafficMirrors,
  const Optional<LineString3d> & stopLine, const LineStrings3d & lightBulbs)
  : RegulatoryElement(constructAutowareTrafficMirrorData(id, attributes, trafficMirrors, stopLine, lightBulbs))
{
}

ConstLineStrings3d AutowareTrafficMirror::lightBulbs() const
{
  return getParameters<ConstLineString3d>(AutowareRoleNameString::LightBulbs);
}

void AutowareTrafficMirror::addLightBulbs(const LineStringOrPolygon3d & primitive)
{
  parameters()[AutowareRoleNameString::LightBulbs].emplace_back(primitive.asRuleParameter());
}

bool AutowareTrafficMirror::removeLightBulbs(const LineStringOrPolygon3d & primitive)
{
  auto it = parameters().find(AutowareRoleNameString::LightBulbs);
  if (it != parameters().end()) {
    return findAndErase(primitive.asRuleParameter(), &it->second);
  }
  return false;
}

}  // namespace lanelet::autoware

// NOLINTEND(readability-identifier-naming)