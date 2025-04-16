// Copyright 2020 Tier IV, Inc.
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

// 수정: 필요 없어진 헤더 포함 가능성 (ClassificationResult 등 관련된 헤더 제거 필요)
#include "traffic_mirror_visualization/traffic_mirror_roi_visualizer/nodelet.hpp"  // NOLINT(whitespace/line_length)

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
// 수정: message_filters 관련 헤더만 남김 (TrafficSignalArray 제거)
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_mirror_roi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <memory>
#include <string>
#include <utility>
#include <vector> // vector 헤더 추가

namespace traffic_mirror
{

// 수정: TrafficSignalArray 제거된 새 Sync Policy 정의
using ImageRoiSyncPolicy = message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::Image, tier4_perception_msgs::msg::TrafficMirrorRoiArray>;
using ImageRoiRoughRoiSyncPolicy = message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::Image, tier4_perception_msgs::msg::TrafficMirrorRoiArray, // Fine ROI
  tier4_perception_msgs::msg::TrafficMirrorRoiArray  // Rough ROI
  >;

using Sync = message_filters::Synchronizer<ImageRoiSyncPolicy>;
using SyncWithRoughRoi = message_filters::Synchronizer<ImageRoiRoughRoiSyncPolicy>;

TrafficMirrorRoiVisualizerNodelet::TrafficMirrorRoiVisualizerNodelet(
  const rclcpp::NodeOptions & options)
: Node("traffic_mirror_roi_visualizer_node", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  // 수정: 플레이스홀더 _4 제거
  // using std::placeholders::_4;
  enable_fine_detection_ = this->declare_parameter("enable_fine_detection", false);

  // 수정: Synchronizer 초기화 시 traffic_signals_sub_ 제거
  if (enable_fine_detection_) {
    // 수정: 새로운 Sync Policy 사용 및 traffic_signals_sub_ 제거
    sync_with_rough_roi_.reset(new SyncWithRoughRoi(
      ImageRoiRoughRoiSyncPolicy(10), image_sub_, roi_sub_, rough_roi_sub_));
    // 수정: imageRoughRoiCallback의 바인딩 파라미터 수정 (_4 제거)
    sync_with_rough_roi_->registerCallback(
      std::bind(&TrafficMirrorRoiVisualizerNodelet::imageRoughRoiCallback, this, _1, _2, _3));
  } else {
    // 수정: 새로운 Sync Policy 사용 및 traffic_signals_sub_ 제거
    sync_.reset(new Sync(ImageRoiSyncPolicy(10), image_sub_, roi_sub_));
    // 수정: imageRoiCallback의 바인딩 파라미터 수정 (_3 제거)
    sync_->registerCallback(
      std::bind(&TrafficMirrorRoiVisualizerNodelet::imageRoiCallback, this, _1, _2));
  }

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&TrafficMirrorRoiVisualizerNodelet::connectCb, this));

  image_pub_ =
    image_transport::create_publisher(this, "~/output/image", rclcpp::QoS{1}.get_rmw_qos_profile());
}

void TrafficMirrorRoiVisualizerNodelet::connectCb()
{
  if (image_pub_.getNumSubscribers() == 0) {
    image_sub_.unsubscribe();
    // 수정: traffic_signals_sub_ 구독 해제 제거
    // traffic_signals_sub_.unsubscribe();
    roi_sub_.unsubscribe();
    if (enable_fine_detection_) {
      rough_roi_sub_.unsubscribe();
    }
  } else if (!image_sub_.getSubscriber()) {
    image_sub_.subscribe(this, "~/input/image", "raw", rmw_qos_profile_sensor_data);
    roi_sub_.subscribe(this, "~/input/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
    // 수정: traffic_signals_sub_ 구독 제거
    // traffic_signals_sub_.subscribe(
    //   this, "~/input/traffic_signals", rclcpp::QoS{1}.get_rmw_qos_profile());
    if (enable_fine_detection_) {
      rough_roi_sub_.subscribe(this, "~/input/rough/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
    }
  }
}

// 이 함수는 변경 없음 (기본 색상으로 그리는 버전)
bool TrafficMirrorRoiVisualizerNodelet::createRect(
  cv::Mat & image, const tier4_perception_msgs::msg::TrafficMirrorRoi & tm_roi,
  const cv::Scalar & color)
{
  cv::rectangle(
    image, cv::Point(tm_roi.roi.x_offset, tm_roi.roi.y_offset),
    cv::Point(tm_roi.roi.x_offset + tm_roi.roi.width, tm_roi.roi.y_offset + tm_roi.roi.height),
    color, 3);
  cv::putText(
    image, std::to_string(tm_roi.traffic_mirror_id),
    cv::Point(tm_roi.roi.x_offset, tm_roi.roi.y_offset + 30), // ID 텍스트 위치 조정 (겹침 방지)
    cv::FONT_HERSHEY_COMPLEX, 1.0, color, 1, CV_AA);
  return true;
}

// 수정: ClassificationResult를 사용하는 createRect 함수 제거
/*
bool TrafficMirrorRoiVisualizerNodelet::createRect(
  cv::Mat & image, const tier4_perception_msgs::msg::TrafficMirrorRoi & tm_roi,
  const ClassificationResult & result)
{
  // ... (내용 제거) ...
}
*/

// 수정: Callback 시그니처에서 TrafficSignalArray 제거 및 로직 변경
void TrafficMirrorRoiVisualizerNodelet::imageRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const tier4_perception_msgs::msg::TrafficMirrorRoiArray::ConstSharedPtr & input_tm_roi_msg)
//  [[maybe_unused]] const tier4_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr &
//    input_traffic_signals_msg) // 제거됨
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::RGB8);
    for (const auto& tm_roi : input_tm_roi_msg->rois) { // const auto& 사용
      // 수정: ClassificationResult 관련 로직 제거, 항상 기본 색상 사용
      // ClassificationResult result;
      // bool has_correspond_traffic_signal =
      //   getClassificationResult(tm_roi.traffic_mirror_id, *input_traffic_signals_msg, result);

      // if (!has_correspond_traffic_signal) {
        // does not have classification result
      createRect(cv_ptr->image, tm_roi, cv::Scalar(255, 255, 255)); // 흰색으로 ROI 그리기
      // } else {
      //   // has classification result
      //   createRect(cv_ptr->image, tm_roi, result);
      // }
    }
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Could not convert from '%s' to 'rgb8'.", input_image_msg->encoding.c_str());
    return; // 에러 발생 시 함수 종료
  }
  if (cv_ptr) { // cv_ptr 유효성 검사 추가
      image_pub_.publish(cv_ptr->toImageMsg());
  }
}

// 수정: getClassificationResult 함수 전체 제거
/*
bool TrafficMirrorRoiVisualizerNodelet::getClassificationResult(
  int id, const tier4_perception_msgs::msg::TrafficSignalArray & traffic_signals,
  ClassificationResult & result)
{
 // ... (내용 제거) ...
}
*/

// 이 함수는 변경 없음 (ID로 ROI 찾는 함수)
bool TrafficMirrorRoiVisualizerNodelet::getRoiFromId(
  int id, const tier4_perception_msgs::msg::TrafficMirrorRoiArray::ConstSharedPtr & rois,
  tier4_perception_msgs::msg::TrafficMirrorRoi & correspond_roi)
{
  for (const auto& roi : rois->rois) { // const auto& 사용
    if (roi.traffic_mirror_id == id) {
      correspond_roi = roi;
      return true;
    }
  }
  return false;
}

// 수정: Callback 시그니처에서 TrafficSignalArray 제거 및 로직 변경
void TrafficMirrorRoiVisualizerNodelet::imageRoughRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const tier4_perception_msgs::msg::TrafficMirrorRoiArray::ConstSharedPtr & input_tm_roi_msg,      // Fine ROI
  const tier4_perception_msgs::msg::TrafficMirrorRoiArray::ConstSharedPtr & input_tm_rough_roi_msg // Rough ROI
  // const tier4_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr & input_traffic_signals_msg // 제거됨
  )
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::RGB8);
    // Rough ROI를 먼저 그림 (예: 녹색)
    for (const auto& tm_rough_roi : input_tm_rough_roi_msg->rois) { // const auto& 사용
      createRect(cv_ptr->image, tm_rough_roi, cv::Scalar(0, 255, 0)); // 녹색으로 Rough ROI 그리기
    }

    // Fine ROI를 그림 (예: 흰색)
    for (const auto& tm_roi : input_tm_roi_msg->rois) { // const auto& 사용
       createRect(cv_ptr->image, tm_roi, cv::Scalar(255, 255, 255)); // 흰색으로 Fine ROI 그리기
    }

    // 아래 로직은 제거됨 (ClassificationResult 및 복잡한 조건 분기 불필요)
    /*
    for (auto tm_rough_roi : input_tm_rough_roi_msg->rois) {
      // visualize rough roi
      createRect(cv_ptr->image, tm_rough_roi, cv::Scalar(0, 255, 0));

      ClassificationResult result;
      // 수정: getClassificationResult 호출 제거
      // bool has_correspond_traffic_signal =
      //   getClassificationResult(tm_rough_roi.traffic_mirror_id, *input_traffic_signals_msg, result);
      tier4_perception_msgs::msg::TrafficMirrorRoi tm_roi;
      bool has_correspond_roi =
        getRoiFromId(tm_rough_roi.traffic_mirror_id, input_tm_roi_msg, tm_roi);

      // 수정: traffic_signals 의존성 제거 및 로직 단순화
      if (has_correspond_roi) { // Fine ROI가 있으면 흰색으로 그림
        createRect(cv_ptr->image, tm_roi, cv::Scalar(255, 255, 255));
      }
      // else {
      //   // Fine ROI가 없는 경우 (Rough ROI만 녹색으로 그려진 상태 유지)
      // }

      // 아래 원본 로직 제거
      // if (has_correspond_roi && has_correspond_traffic_signal) {
      //   // has fine detection and classification results
      //   createRect(cv_ptr->image, tm_roi, result);
      // } else if (has_correspond_roi && !has_correspond_traffic_signal) {
      //   // has fine detection result and does not have classification result
      //   createRect(cv_ptr->image, tm_roi, cv::Scalar(255, 255, 255));
      // } else if (!has_correspond_roi && has_correspond_traffic_signal) {
      //   // does not have fine detection result and has classification result
      //   createRect(cv_ptr->image, tm_rough_roi, result);
      // } else {
      // }
    }
    */
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Could not convert from '%s' to 'rgb8'.", input_image_msg->encoding.c_str());
     return; // 에러 발생 시 함수 종료
  }
   if (cv_ptr) { // cv_ptr 유효성 검사 추가
      image_pub_.publish(cv_ptr->toImageMsg());
  }
}

}  // namespace traffic_mirror

RCLCPP_COMPONENTS_REGISTER_NODE(traffic_mirror::TrafficMirrorRoiVisualizerNodelet)