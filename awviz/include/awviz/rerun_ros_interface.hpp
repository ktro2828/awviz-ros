#ifndef AWVIZ__RERUN_ROS_INTERFACE_HPP_
#define AWVIZ__RERUN_ROS_INTERFACE_HPP_

#include <rerun.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

namespace awviz
{
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
}  // namespace awviz

#endif  // AWVIZ__RERUN_ROS_INTERFACE_HPP_
