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

#include "awviz/rerun_ros_interface.hpp"

#include "awviz/uuid.hpp"
#include "collection_adapters.hpp"
#include "color.hpp"
#include "rerun/archetypes/pinhole.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace awviz
{
void logTransform(
  const rerun::RecordingStream & stream, const std::string & entity,
  const geometry_msgs::msg::TransformStamped & msg)
{
  stream.set_time_seconds(
    "timestamp", rclcpp::Time(msg.header.stamp.sec, msg.header.stamp.nanosec).seconds());

  stream.log(
    entity,
    rerun::Transform3D(
      rerun::Vector3D(
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z),
      rerun::Quaternion::from_wxyz(
        msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y,
        msg.transform.rotation.z)));
}

void logPointCloud(
  const rerun::RecordingStream & stream, const std::string & entity,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  stream.set_time_seconds(
    "timestamp", rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  bool has_x{false}, has_y{false}, has_z{false};
  for (const auto & field : msg->fields) {
    if (field.name == "x") {
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
        stream.log(entity, rerun::TextLog("Only FLOAT32 x field supported"));
        return;
      }
      has_x = true;
    } else if (field.name == "y") {
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
        stream.log(entity, rerun::TextLog("Only FLOAT32 y field supported"));
        return;
      }
      has_y = true;
    } else if (field.name == "z") {
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
        stream.log(entity, rerun::TextLog("Only FLOAT32 z field supported"));
        return;
      }
      has_z = true;
    }
  }

  if (!has_x || !has_y || !has_z) {
    stream.log(entity, rerun::TextLog("Currently only PointCloud2 with x/y/z are supported"));
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> itr_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> itr_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> itr_z(*msg, "z");

  // TODO(ktro2828): update method retrieving colormap values
  std::vector<rerun::Position3D> points(msg->width * msg->height);
  std::vector<float> values(msg->width * msg->height);
  for (; itr_x != itr_x.end(); ++itr_x, ++itr_y, ++itr_z) {
    points.emplace_back(*itr_x, *itr_y, *itr_z);
    values.emplace_back(*itr_z);
  }

  auto colors = colormap(values);
  stream.log(entity, rerun::Points3D(points).with_colors(colors));
}

void logCameraInfo(
  const rerun::RecordingStream & stream, const std::string & entity,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  const std::array<float, 9> camera_intrinsic = {
    static_cast<float>(msg->k[0]), static_cast<float>(msg->k[3]), static_cast<float>(msg->k[6]),
    static_cast<float>(msg->k[1]), static_cast<float>(msg->k[4]), static_cast<float>(msg->k[7]),
    static_cast<float>(msg->k[2]), static_cast<float>(msg->k[5]), static_cast<float>(msg->k[8])};

  stream.log(
    entity, rerun::Pinhole(camera_intrinsic)
              .with_resolution(static_cast<int>(msg->width), static_cast<int>(msg->height)));
}

void logImage(
  const rerun::RecordingStream & stream, const std::string & entity,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  stream.set_time_seconds(
    "timestamp", rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  if (msg->encoding == "16UC1") {
    auto img = cv_bridge::toCvCopy(msg)->image;
    stream.log(
      entity, rerun::DepthImage(
                {static_cast<size_t>(img.rows), static_cast<size_t>(img.cols)},
                rerun::TensorBuffer::u16(img))
                .with_meter(1000));
  } else if (msg->encoding == "32FC1") {
    auto img = cv_bridge::toCvCopy(msg)->image;

    stream.log(
      entity, rerun::DepthImage(
                {static_cast<size_t>(img.rows), static_cast<size_t>(img.cols)},
                rerun::TensorBuffer::f32(img))
                .with_meter(1.0));
  } else {
    auto img = cv_bridge::toCvCopy(msg, "rgb8")->image;
    stream.log(entity, rerun::Image(tensorShape(img), rerun::TensorBuffer::u8(img)));
  }
}

void logCompressedImage(
  const rerun::RecordingStream & stream, const std::string & entity,
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg)
{
  stream.set_time_seconds(
    "timestamp", rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  if (msg->format.find("jpeg")) {
    auto img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    stream.log(entity, rerun::Image(tensorShape(img), rerun::TensorBuffer::u8(img)));
  } else {
    auto img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    cv::Mat depth;
    img.convertTo(depth, CV_32FC1);

    stream.log(
      entity, rerun::DepthImage(
                {static_cast<size_t>(depth.rows), static_cast<size_t>(depth.cols)},
                rerun::TensorBuffer::f32(depth))
                .with_meter(1.0));
  }
}

void logDetectedObjects(
  const rerun::RecordingStream & stream, const std::string & entity,
  const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr & msg)
{
  stream.set_time_seconds(
    "timestamp", rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  std::vector<rerun::Position3D> centers;
  std::vector<rerun::HalfSize3D> sizes;
  std::vector<rerun::Rotation3D> rotations;
  std::vector<rerun::components::ClassId> class_ids;
  for (const auto & object : msg->objects) {
    const auto & pose = object.kinematics.pose_with_covariance.pose;
    const auto & dimensions = object.shape.dimensions;
    centers.emplace_back(pose.position.x, pose.position.y, pose.position.z);
    sizes.emplace_back(dimensions.x, dimensions.y, dimensions.z);
    rotations.emplace_back(rerun::Quaternion::from_wxyz(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));
    class_ids.emplace_back(static_cast<uint16_t>(object.classification.front().label));
  }

  stream.log(
    entity, rerun::Boxes3D::from_centers_and_half_sizes(centers, sizes)
              .with_rotations(rotations)
              .with_class_ids(class_ids));
}

void logTrackedObjects(
  const rerun::RecordingStream & stream, const std::string & entity,
  const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr & msg)
{
  stream.set_time_seconds(
    "timestamp", rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  std::vector<rerun::Position3D> centers;
  std::vector<rerun::HalfSize3D> sizes;
  std::vector<rerun::Rotation3D> rotations;
  std::vector<rerun::components::ClassId> class_ids;
  std::vector<rerun::Text> uuids;
  for (const auto & object : msg->objects) {
    const auto & pose = object.kinematics.pose_with_covariance.pose;
    const auto & dimensions = object.shape.dimensions;
    centers.emplace_back(pose.position.x, pose.position.y, pose.position.z);
    sizes.emplace_back(dimensions.x, dimensions.y, dimensions.z);
    rotations.emplace_back(rerun::Quaternion::from_wxyz(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));
    class_ids.emplace_back(static_cast<uint16_t>(object.classification.front().label));
    uuids.emplace_back(uuid<std::string>(object.object_id));
  }

  // TODO(ktro2828): use a shortened uuid of approximately length 8.
  stream.log(
    entity, rerun::Boxes3D::from_centers_and_half_sizes(centers, sizes)
              .with_rotations(rotations)
              .with_class_ids(class_ids));
}

}  // namespace awviz
