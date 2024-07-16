// Copyright 2024 Kotaro Uetake.
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

#ifndef AWVIZ__RERUN_ROS_INTERFACE_HPP_
#define AWVIZ__RERUN_ROS_INTERFACE_HPP_

#include "rerun/recording_stream.hpp"

#include <rerun.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

namespace awviz
{
/**
 * @brief Log TransformStamped msg to rerun stream.
 * @param stream Rerun recording stream.
 * @param entity Entity path of the record.
 * @param msg TransformStamped msg pointer.
 */
void logTransform(
  const rerun::RecordingStream & stream, const std::string & entity,
  const geometry_msgs::msg::TransformStamped & msg);

/**
 * @brief Log PointCloud2 msg to rerun stream.
 * @param stream Rerun recording stream.
 * @param entity Entity path of the record.
 * @param msg PointCloud2 msg pointer.
 */
void logPointCloud(
  const rerun::RecordingStream & stream, const std::string & entity,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

/**
 * @brief Log Image msg to rerun stream.
 * @param stream Rerun recording stream.
 * @param entity Entity path of the record.
 * @param msg Image msg pointer.
 */
void logImage(
  const rerun::RecordingStream & stream, const std::string & entity,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg);

/**
 * @brief Log CompressedImage msg to rerun stream.
 * @param stream Rerun recording stream.
 * @param entity Entity path of the record.
 * @param msg CompressedImage msg pointer.
 */
void logCompressedImage(
  const rerun::RecordingStream & stream, const std::string & entity,
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg);

/**
 * @brief Log DetectedObjects msg to rerun stream.
 * @param stream Rerun recodring stream.
 * @param entity Entity path of the record.
 * @param msg DetectedObjects msg pointer.
 */
void logDetectedObjects(
  const rerun::RecordingStream & stream, const std::string & entity,
  const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr & msg);

/**
 * @brief Log TrackedObjects msg to rerun stream.
 * @param stream Rerun recodring stream.
 * @param entity Entity path of the record.
 * @param msg TrackedObjects msg pointer.
 */
void logTrackedObjects(
  const rerun::RecordingStream & stream, const std::string & entity,
  const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr & msg);
}  // namespace awviz

#endif  // AWVIZ__RERUN_ROS_INTERFACE_HPP_
