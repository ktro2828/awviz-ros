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

#ifndef AWVIZ_PLUGIN__IMAGE__COLLECTION_ADAPTER_HPP_
#define AWVIZ_PLUGIN__IMAGE__COLLECTION_ADAPTER_HPP_

#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <rerun/collection.hpp>

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
  /// Borrow for non-temporary.
  Collection<TElement> operator()(const cv::Mat & img)
  {
    return Collection<TElement>::borrow(
      reinterpret_cast<TElement *>(img.data), img.total() * img.channels());
  }

  // Do a full copy for temporaries (otherwise the data might be deleted when the temporary is
  // destroyed).
  Collection<TElement> operator()(cv::Mat && img)
  {
    std::vector<TElement> img_vec(img.total() * img.channels());
    img_vec.assign(img.data, img.data + img.total() * img.channels());
    return Collection<TElement>::take_ownership(std::move(img_vec));
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

#endif  // AWVIZ_PLUGIN__IMAGE__IMAGE_COMPRESSED_IMAGE_DISPLAY_HPP_
