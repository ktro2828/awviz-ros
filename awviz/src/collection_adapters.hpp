#ifndef AWVIZ__COLLECTION_ADAPTERS_HPP_
#define AWVIZ__COLLECTION_ADAPTERS_HPP_

#include <opencv2/opencv.hpp>
#include <rerun.hpp>

#include <vector>

namespace awviz
{
/**
 * @brief An adaptor to be able to borrow an OpenvCV matrix into Rerun images without copying.
 */
template <typename TElement>
struct rerun::CollectionAdapter<TElement, cv::Mat>
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

inline rerun::Collection<rerun::TensorDimension> tensorShape(const cv::Mat & img)
{
  return {
    static_cast<size_t>(img.rows), static_cast<size_t>(img.cols),
    static_cast<size_t>(img.channels())};
}
}  // namespace awviz
#endif  // AWVIZ__COLLECTION_ADAPTERS_HPP_
