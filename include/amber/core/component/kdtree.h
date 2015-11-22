// Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <algorithm>
#include <limits>
#include <mutex>
#include <numeric>
#include <vector>

#include "core/aabb.h"
#include "core/axis.h"
#include "core/vector3.h"

namespace amber {
namespace core {
namespace component {

template <typename T, typename RealType>
class KDTree
{
private:
  using aabb_type    = AABB<RealType>;
  using vector3_type = Vector3<RealType>;

  struct Node : public T
  {
    Axis axis;

    Node() noexcept : T() {}

    explicit Node(T const& value) noexcept : T(value), axis() {}
  };

  struct Comparator
  {
    vector3_type point;

    explicit Comparator(vector3_type const& point) noexcept : point(point) {}

    bool operator()(T const& a, T const& b) const noexcept
    {
      return
        SquaredLength(static_cast<vector3_type>(a) - point) <
        SquaredLength(static_cast<vector3_type>(b) - point);
    }
  };

  std::vector<Node> nodes_;

  std::mutex     mutable search_buffer_mtx_;
  std::vector<T> mutable search_buffer_;

public:
  template <typename RandomAccessIterator>
  KDTree(
    RandomAccessIterator first,
    RandomAccessIterator last
  )
  : nodes_(std::distance(first, last)),
    search_buffer_mtx_(),
    search_buffer_()
  {
    BuildKDTree(first, last, 0);
  }

  KDTree(KDTree&& kdtree)
  : nodes_(std::move(kdtree.nodes_)),
    search_buffer_mtx_(),
    search_buffer_()
  {}

  std::vector<T>
  SearchRNeighbours(
    vector3_type const& search_point,
    RealType radius
  ) const
  {
    return SearchNeighbours(search_point, radius, nodes_.size());
  }

  std::vector<T>
  SearchKNeighbours(
    vector3_type const& search_point,
    std::size_t k
  ) const
  {
    return
      SearchNeighbours(search_point, std::numeric_limits<RealType>::max(), k);
  }

  std::vector<T>
  SearchNeighbours(
    vector3_type const& search_point,
    RealType radius,
    std::size_t k
  ) const
  {
    std::lock_guard<std::mutex> lock(search_buffer_mtx_);
    search_buffer_.clear();

    auto squared_radius = radius * radius;
    Search(search_point, k, 0, squared_radius);

    std::sort_heap(
      search_buffer_.begin(),
      search_buffer_.end(),
      Comparator(search_point)
    );

    return search_buffer_;
  }

private:
  template <typename RandomAccessIterator>
  void
  BuildKDTree(
    RandomAccessIterator first,
    RandomAccessIterator last,
    std::size_t pos
  )
  {
    if (pos >= nodes_.size()) {
      return;
    }

    Axis axis;
    {
      auto const bb = std::accumulate(
        first,
        last,
        aabb_type(),
        [](auto const& bb, auto const& value) {
          return bb + static_cast<aabb_type>(static_cast<vector3_type>(value));
        }
      );
      auto const x = bb.max().x() - bb.min().x();
      auto const y = bb.max().y() - bb.min().y();
      auto const z = bb.max().z() - bb.min().z();
      if (x > y && x > z) {
        axis = Axis::X;
      } else if (y > z) {
        axis = Axis::Y;
      } else {
        axis = Axis::Z;
      }
    }

    std::sort(first, last, [axis](auto const& a, auto const& b){
      switch (axis) {
      case Axis::X:
        return
          static_cast<vector3_type>(a).x() < static_cast<vector3_type>(b).x();
      case Axis::Y:
        return
          static_cast<vector3_type>(a).y() < static_cast<vector3_type>(b).y();
      case Axis::Z:
        return
          static_cast<vector3_type>(a).z() < static_cast<vector3_type>(b).z();
      }
    });

    std::size_t const size = std::distance(first, last);
    std::size_t left_size = 0;
    std::size_t right_size = 0;
    while (left_size + right_size + 1 < size) {
      left_size = std::min(size - 1 - right_size, (left_size << 1) + 1);
      right_size = std::min(size - 1 - left_size, (right_size << 1) + 1);
    }

    auto const middle = first + left_size;
    nodes_.at(pos).T::operator=(*middle);
    nodes_.at(pos).axis = axis;

    BuildKDTree(first, middle, pos * 2 + 1);
    BuildKDTree(middle + 1, last, pos * 2 + 2);
  }

  void
  Search(
    vector3_type const& search_point,
    std::size_t k,
    std::size_t pos,
    RealType& squared_radius
  ) const
  {
    if (pos >= nodes_.size()) {
      return;
    }

    auto const& node = nodes_.at(pos);
    auto const node_point = static_cast<vector3_type>(node);

    RealType plane_distance;
    switch (node.axis) {
    case Axis::X:
      plane_distance = node_point.x() - search_point.x();
      break;
    case Axis::Y:
      plane_distance = node_point.y() - search_point.y();
      break;
    case Axis::Z:
      plane_distance = node_point.z() - search_point.z();
      break;
    }

    auto const near = pos * 2 + 1 + (plane_distance < 0);
    auto const far = pos * 2 + 1 + (plane_distance >= 0);

    Search(search_point, k, near, squared_radius);

    if (SquaredLength(node_point - search_point) < squared_radius) {
      search_buffer_.push_back(node);
      std::push_heap(
        search_buffer_.begin(),
        search_buffer_.end(),
        Comparator(search_point)
      );

      while (search_buffer_.size() > k) {
        std::pop_heap(
          search_buffer_.begin(),
          search_buffer_.end(),
          Comparator(search_point)
        );
        search_buffer_.pop_back();
      }

      if (search_buffer_.size() >= k) {
        squared_radius = SquaredLength(
          static_cast<vector3_type>(search_buffer_.front()) - search_point
        );
      }
    }

    if (plane_distance * plane_distance < squared_radius) {
      Search(search_point, k, far, squared_radius);
    }
  }
};

}
}
}
