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

#ifndef AWVIZ_PLUGIN__COLLECTION_ADAPTER_HPP_
#define AWVIZ_PLUGIN__COLLECTION_ADAPTER_HPP_

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <rerun/collection.hpp>

#include <cstring>
#include <utility>
#include <vector>

namespace rerun
{
/**
 * @brief An adaptor to be able to borrow an OpenCV matrix into Rerun images without copying.
 */
template <typename TElement>
struct CollectionAdapter<TElement, cv::Mat>
{
  /**
   * @brief Borrow for non-temporary.
   *
   * @param img OpenCV matrix.
   * @return Collection<TElement>
   */
  Collection<TElement> operator()(const cv::Mat & img)
  {
    return Collection<TElement>::borrow(
      reinterpret_cast<TElement *>(img.data), img.total() * img.channels());
  }

  /**
   * @brief Do a full copy for temporaries (otherwise the data might be deleted when the temporary
   * is destroyed).
   *
   * @param img OpenCV matrix.
   * @return Collection<TElement>
   */
  Collection<TElement> operator()(cv::Mat && img)
  {
    std::vector<TElement> img_vec(img.total() * img.channels());
    img_vec.assign(img.data, img.data + img.total() * img.channels());
    return Collection<TElement>::take_ownership(std::move(img_vec));
  }
};

/**
 * @brief An adaptor to be able to borrow an Eigen matrix into Rerun 3D position without copying.
 */
template <>
struct CollectionAdapter<rerun::Position3D, std::vector<Eigen::Vector3f>>
{
  // Sanity check that this is binary compatible.
  static_assert(
    sizeof(rerun::Position3D) ==
    sizeof(Eigen::Matrix3Xf::Scalar) * Eigen::Matrix3Xf::RowsAtCompileTime);

  /**
   * @brief Borrow for non-temporary.
   *
   * @param container Eigen 3D float vector.
   * @return Collection<rerun::Position3D>
   */
  Collection<rerun::Position3D> operator()(const std::vector<Eigen::Vector3f> & container)
  {
    return Collection<rerun::Position3D>::borrow(container.data(), container.size());
  }

  /**
   * @brief Do a full copy for temporaries (otherwise the data might be deleted when the temporary
   * is destroyed).
   *
   * @param container Eigen 3D float vector.
   * @return Collection<rerun::Position3D>
   */
  Collection<rerun::Position3D> operator()(std::vector<Eigen::Vector3f> && container)
  {
    std::vector<rerun::Position3D> positions(container.size());
    memcpy(positions.data(), container.data(), container.size() * sizeof(Eigen::Vector3f));
    return Collection<rerun::Position3D>::take_ownership(std::move(positions));
  }
};
}  // namespace rerun

namespace awviz_plugin
{
/**
 * @brief Return image tensor shape as `rerun::Collection`.
 *
 * @param img Input image.
 * @return Tensor shape as collection.
 */
inline rerun::Collection<rerun::TensorDimension> tensor_shape(const cv::Mat & img)
{
  return {
    static_cast<size_t>(img.rows), static_cast<size_t>(img.cols),
    static_cast<size_t>(img.channels())};
}
}  // namespace awviz_plugin

#endif
