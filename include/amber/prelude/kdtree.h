// Copyright (c) 2016 TAKAMORI Kaede <etheriqa@gmail.com>
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
#include <functional>
#include <numeric>
#include <vector>

#include "amber/prelude/aabb.h"
#include "amber/prelude/axis.h"
#include "amber/prelude/vector3.h"

namespace amber {
namespace prelude {

template <typename T, typename U>
class KDTree
{
public:
  using Converter = std::function<Vector3<T>(U)>;

  KDTree() noexcept;
  explicit KDTree(const Converter& converter) noexcept;

  void Build(std::vector<U>& values);

  void Search(
    const Vector3<T>& point,
    const T& radius,
    std::size_t k,
    std::vector<U>& heap
  ) const;

private:
  struct Node;
  class Comparator;

  std::vector<Node> nodes_;
  Converter converter_;

  static const Vector3<T> DefaultConverter(const U& value) noexcept;

  void BuildKDTree(
    typename std::vector<U>::iterator first,
    typename std::vector<U>::iterator last,
    std::size_t pos
  );
  void SearchKDTree(
    const Vector3<T>& point,
    T& squared_radius,
    std::size_t pos,
    std::size_t k,
    std::vector<U>& heap
  ) const;
};

template <typename T, typename U>
struct KDTree<T, U>::Node
{
  Axis axis;
  U value;

  Node() noexcept;
  Node(Axis axis, const U& value) noexcept;
};

template <typename T, typename U>
class KDTree<T, U>::Comparator
{
public:
  Comparator(const Vector3<T>& point, const Converter& converter) noexcept;

  bool operator()(const U& x, const U& y) const noexcept;

private:
  Vector3<T> point_;
  Converter converter_;
};



template <typename T, typename U>
KDTree<T, U>::KDTree() noexcept
: nodes_()
, converter_(DefaultConverter)
{}

template <typename T, typename U>
KDTree<T, U>::KDTree(const Converter& converter) noexcept
: nodes_()
, converter_(converter)
{}

template <typename T, typename U>
void
KDTree<T, U>::Build(std::vector<U>& values)
{
  nodes_.resize(values.size());
  BuildKDTree(values.begin(), values.end(), 0);
}

template <typename T, typename U>
void
KDTree<T, U>::Search(
  const Vector3<T>& point,
  const T& radius,
  std::size_t k,
  std::vector<U>& heap
) const
{
  heap.clear();
  auto squared_radius = radius * radius;
  return SearchKDTree(point, squared_radius, 0, k, heap);
}

template <typename T, typename U>
const Vector3<T>
KDTree<T, U>::DefaultConverter(const U& value) noexcept
{
  return static_cast<Vector3<T>>(value);
}

template <typename T, typename U>
void
KDTree<T, U>::BuildKDTree(
  typename std::vector<U>::iterator first,
  typename std::vector<U>::iterator last,
  std::size_t pos
)
{
  if (pos >= nodes_.size()) {
    return;
  }

  Axis axis;
  {
    const auto bb = std::accumulate(
      first,
      last,
      AABB<T>::Empty(),
      [&](const auto& bb, const auto& value){
        return bb + AABB<T>(converter_(value));
      }
    );
    const auto size = Size(bb);
    if (size.X() > size.Y() && size.X() > size.Z()) {
      axis = Axis::X;
      std::sort(first, last, [&](const auto& x, const auto& y){
        return converter_(x).X() < converter_(y).X();
      });
    } else if (size.Y() > size.Z()) {
      axis = Axis::Y;
      std::sort(first, last, [&](const auto& x, const auto& y){
        return converter_(x).Y() < converter_(y).Y();
      });
    } else {
      axis = Axis::Z;
      std::sort(first, last, [&](const auto& x, const auto& y){
        return converter_(x).Z() < converter_(y).Z();
      });
    }
  }

  std::size_t left_size = 0;
  std::size_t right_size = 0;
  {
    const std::size_t size = std::distance(first, last);
    while (left_size + right_size + 1 < size) {
      left_size = std::min(size - 1 - right_size, (left_size << 1) + 1);
      right_size = std::min(size - 1 - left_size, (right_size << 1) + 1);
    }
  }

  const auto middle = first + left_size;
  nodes_[pos] = Node(axis, *middle);

  BuildKDTree(first, middle, pos * 2 + 1);
  BuildKDTree(middle+ 1, last, pos * 2 + 2);
}

template <typename T, typename U>
void
KDTree<T, U>::SearchKDTree(
  const Vector3<T>& point,
  T& squared_radius,
  std::size_t pos,
  std::size_t k,
  std::vector<U>& heap
) const
{
  if (pos >= nodes_.size()) {
    return;
  }

  const auto& node = nodes_[pos];
  const auto split = converter_(node.value);

  T split_distance = 0;
  switch (node.axis) {
  case Axis::X:
    split_distance = split.X() - point.X();
    break;
  case Axis::Y:
    split_distance = split.Y() - point.Y();
    break;
  case Axis::Z:
    split_distance = split.Z() - point.Z();
    break;
  }

  const auto near = pos * 2 + 1 + (split_distance < 0);
  const auto far = pos * 2 + 1 + (split_distance >= 0);

  SearchKDTree(point, squared_radius, near, k, heap);

  if (SquaredLength(split - point) < squared_radius) {
    heap.emplace_back(node.value);
    std::push_heap(heap.begin(), heap.end(), Comparator(point, converter_));

    if (heap.size() > k) {
      std::pop_heap(heap.begin(), heap.end(), Comparator(point, converter_));
      heap.pop_back();
    }

    if (heap.size() == k) {
      squared_radius = SquaredLength(converter_(heap.front()) - point);
    }
  }

  if (split_distance * split_distance < squared_radius) {
    SearchKDTree(point, squared_radius, far, k, heap);
  }
}

template <typename T, typename U>
KDTree<T, U>::Node::Node() noexcept
: axis(Axis::X)
, value()
{}

template <typename T, typename U>
KDTree<T, U>::Node::Node(Axis axis, const U& value) noexcept
: axis(axis)
, value(value)
{}

template <typename T, typename U>
KDTree<T, U>::Comparator::Comparator(
  const Vector3<T>& point,
  const Converter& converter
) noexcept
: point_(point)
, converter_(converter)
{}

template <typename T, typename U>
bool
KDTree<T, U>::Comparator::operator()( const U& x, const U& y) const noexcept
{
  return
    SquaredLength(converter_(x) - point_) <
    SquaredLength(converter_(y) - point_);
}

}
}
