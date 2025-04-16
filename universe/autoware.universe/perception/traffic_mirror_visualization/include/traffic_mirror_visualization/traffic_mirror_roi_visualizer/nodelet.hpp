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
#ifndef TRAFFIC_MIRROR_VISUALIZATION__TRAFFIC_MIRROR_ROI_VISUALIZER__NODELET_HPP_
#define TRAFFIC_MIRROR_VISUALIZATION__TRAFFIC_MIRROR_ROI_VISUALIZER__NODELET_HPP_

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/highgui/highgui.hpp> // OpenCV 관련 헤더 유지
#include <opencv2/imgproc/imgproc_c.h> // OpenCV 관련 헤더 유지
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_mirror_roi_array.hpp>
// 수정: TrafficSignalArray 메시지 헤더 제거
// #include <tier4_perception_msgs/msg/traffic_signal_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// 수정: map 헤더 제거 (state2label_ 제거로 불필요)
// #include <map>
#include <memory>
// 수정: mutex 헤더 제거 (사용되지 않으면 제거 가능, 필요시 유지)
// #include <mutex>
#include <string> // string 헤더 유지
#include <vector> // vector 헤더 추가 (cpp 파일에서 사용 가능성 고려)


namespace traffic_mirror
{
// 수정: ClassificationResult 구조체 제거
/*
struct ClassificationResult
{
  float prob = 0.0;
  std::string label;
};
*/

class TrafficMirrorRoiVisualizerNodelet : public rclcpp::Node
{
public:
  explicit TrafficMirrorRoiVisualizerNodelet(const rclcpp::NodeOptions & options);
  void connectCb();

  // 수정: Callback 선언에서 TrafficSignalArray 파라미터 제거
  void imageRoiCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
    const tier4_perception_msgs::msg::TrafficMirrorRoiArray::ConstSharedPtr & input_tm_roi_msg);
    // const tier4_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr &
    //   input_traffic_signals_msg); // 제거됨

  // 수정: Callback 선언에서 TrafficSignalArray 파라미터 제거
  void imageRoughRoiCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
    const tier4_perception_msgs::msg::TrafficMirrorRoiArray::ConstSharedPtr & input_tm_roi_msg, // Fine ROI
    const tier4_perception_msgs::msg::TrafficMirrorRoiArray::ConstSharedPtr & input_tm_rough_roi_msg // Rough ROI
    // const tier4_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr &
    //   input_traffic_signals_msg); // 제거됨
    );

private:
  // 수정: state2label_ 맵 제거
  /*
  std::map<int, std::string> state2label_{
    // ... (내용 제거) ...
  };
  */

  // 이 함수 선언은 유지 (기본 색상으로 그리는 버전)
  bool createRect(
    cv::Mat & image, const tier4_perception_msgs::msg::TrafficMirrorRoi & tm_roi,
    const cv::Scalar & color);

  // 수정: ClassificationResult 사용하는 createRect 함수 선언 제거
  /*
  bool createRect(
    cv::Mat & image, const tier4_perception_msgs::msg::TrafficMirrorRoi & tm_roi,
    const ClassificationResult & result);
  */

  // 수정: getClassificationResult 함수 선언 제거
  /*
  bool getClassificationResult(
    int id, const tier4_perception_msgs::msg::TrafficSignalArray & traffic_signals,
    ClassificationResult & result);
  */

  // 이 함수 선언은 유지 (ID로 ROI 찾는 함수)
  bool getRoiFromId(
    int id, const tier4_perception_msgs::msg::TrafficMirrorRoiArray::ConstSharedPtr & rois,
    tier4_perception_msgs::msg::TrafficMirrorRoi & correspond_roi);

  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<tier4_perception_msgs::msg::TrafficMirrorRoiArray> roi_sub_;
  message_filters::Subscriber<tier4_perception_msgs::msg::TrafficMirrorRoiArray> rough_roi_sub_;
  // 수정: traffic_signals_sub_ 멤버 변수 제거
  // message_filters::Subscriber<tier4_perception_msgs::msg::TrafficSignalArray> traffic_signals_sub_;
  image_transport::Publisher image_pub_;

  // 수정: Sync Policy typedef 에서 TrafficSignalArray 제거
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, tier4_perception_msgs::msg::TrafficMirrorRoiArray
    // tier4_perception_msgs::msg::TrafficSignalArray // 제거됨
    >
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  // 수정: Sync Policy typedef 에서 TrafficSignalArray 제거
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, tier4_perception_msgs::msg::TrafficMirrorRoiArray, // Fine ROI
    tier4_perception_msgs::msg::TrafficMirrorRoiArray // Rough ROI
    // tier4_perception_msgs::msg::TrafficSignalArray // 제거됨
    >
    SyncPolicyWithRoughRoi;
  typedef message_filters::Synchronizer<SyncPolicyWithRoughRoi> SyncWithRoughRoi;
  std::shared_ptr<SyncWithRoughRoi> sync_with_rough_roi_;

  bool enable_fine_detection_;
};

}  // namespace traffic_mirror

#endif  // TRAFFIC_MIRROR_VISUALIZATION__TRAFFIC_MIRROR_ROI_VISUALIZER__NODELET_HPP_