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
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <vector>

#include "amber/prelude/aabb.h"
#include "amber/prelude/vector3.h"
#include "amber/raytracer/acceleration.h"

namespace amber {
namespace raytracer {

template <typename T, typename Object>
class BVH
: public Acceleration<T, Object>
{
public:
  explicit BVH(std::vector<Object>&& objects) noexcept;

  BVH(const BVH&) = delete;
  BVH& operator=(const BVH&) = delete;

  std::tuple<Hit<T>, const Object*>
  Cast(const Ray<T>& ray, T distance) const noexcept;

private:
  static constexpr T kMaxSplit         = 15.0;
  static constexpr T kTraverseCost     = 2.0;
  static constexpr T kIntersectionCost = 1.0;

  using ObjectIterator = typename std::vector<Object>::iterator;
  class Node;
  struct Split;

  std::vector<Object> objects_;
  Node root_;

  static Node BuildBVH(
    ObjectIterator first,
    ObjectIterator last
  ) noexcept;
  static Node CreateNode(
    ObjectIterator first,
    ObjectIterator last,
    const AABB<T>& bb
  ) noexcept;
  static const AABB<T> BoundingBox(
    ObjectIterator first,
    ObjectIterator last
  ) noexcept;
  static Split FindSplit(
    ObjectIterator first,
    ObjectIterator last,
    const AABB<T>& bb
  ) noexcept;
  static Split FindSplitAxis(
    ObjectIterator first,
    ObjectIterator last,
    const AABB<T>& bb,
    std::function<T(Vector3<T>)> axis
  ) noexcept;
  static const T SurfaceAreaHeuristic(
    const AABB<T>& bb_parent,
    const AABB<T>& bb_left,
    const AABB<T>& bb_right,
    std::size_t n_left,
    std::size_t n_right
  ) noexcept;
};

template <typename T, typename Object>
class BVH<T, Object>::Node
{
public:
  Node(
    std::unique_ptr<Node>&& left,
    std::unique_ptr<Node>&& right,
    const AABB<T>& bb
  ) noexcept;
  Node(
    ObjectIterator first,
    ObjectIterator last,
    const AABB<T>& bb
  ) noexcept;

  const AABB<T>& bb() const noexcept { return bb_; }

  std::tuple<Hit<T>, const Object*>
  Cast(const Ray<T>& ray, T distance) const noexcept;

private:
  std::unique_ptr<Node> left_, right_;
  ObjectIterator first_, last_;
  AABB<T> bb_;
};

template <typename T, typename Object>
struct BVH<T, Object>::Split
{
  T cost;
  ObjectIterator middle;
  AABB<T> bb_left, bb_right;

  Split() noexcept;
};



template <typename T, typename Object>
BVH<T, Object>::BVH(std::vector<Object>&& objects) noexcept
: objects_(std::move(objects))
, root_(BuildBVH(objects_.begin(), objects_.end()))
{}

template <typename T, typename Object>
std::tuple<Hit<T>, const Object*>
BVH<T, Object>::Cast(const Ray<T>& ray, T distance) const noexcept
{
  return root_.Cast(ray, distance);
}

template <typename T, typename Object>
auto
BVH<T, Object>::BuildBVH(
  ObjectIterator first,
  ObjectIterator last
) noexcept
-> Node
{
  return CreateNode(first, last, BoundingBox(first, last));
}

template <typename T, typename Object>
auto
BVH<T, Object>::CreateNode(
  ObjectIterator first,
  ObjectIterator last,
  const AABB<T>& bb
) noexcept
-> Node
{
  const auto split = FindSplit(first, last, bb);

  if (split.cost > std::distance(first, last) * kIntersectionCost) {
    // Create a leaf
    return Node(first, last, bb);
  } else {
    // Create an edge
    return Node(
      std::make_unique<Node>(CreateNode(first, split.middle, split.bb_left)),
      std::make_unique<Node>(CreateNode(split.middle, last, split.bb_right)),
      bb
    );
  }
}

template <typename T, typename Object>
const AABB<T>
BVH<T, Object>::BoundingBox(
  ObjectIterator first,
  ObjectIterator last
) noexcept
{
  return std::accumulate(
    first,
    last,
    AABB<T>::Empty(),
    [](const auto& bb, const auto& object){ return bb + object.BoundingBox(); }
  );
}

template <typename T, typename Object>
auto
BVH<T, Object>::FindSplit(
  ObjectIterator first,
  ObjectIterator last,
  const AABB<T>& bb
) noexcept
-> Split
{
  const auto split_x =
    FindSplitAxis(first, last, bb, [](const auto& v){ return v.X(); });
  const auto split_y =
    FindSplitAxis(first, last, bb, [](const auto& v){ return v.Y(); });
  const auto split_z =
    FindSplitAxis(first, last, bb, [](const auto& v){ return v.Z(); });

  if (split_x.cost < split_y.cost && split_x.cost < split_z.cost) {
    std::sort(
      first,
      last,
      [](const auto& a, const auto& b){
        return a.Center().X() < b.Center().X();
      }
    );
    return split_x;
  } else if (split_y.cost < split_x.cost) {
    std::sort(
      first,
      last,
      [](const auto& a, const auto& b){
        return a.Center().Y() < b.Center().Y();
      }
    );
    return split_y;
  } else {
    std::sort(
      first,
      last,
      [](const auto& a, const auto& b){
        return a.Center().Z() < b.Center().Z();
      }
    );
    return split_z;
  }
}

template <typename T, typename Object>
auto
BVH<T, Object>::FindSplitAxis(
  ObjectIterator first,
  ObjectIterator last,
  const AABB<T>& bb,
  std::function<T(Vector3<T>)> axis
) noexcept
-> Split
{
  std::sort(
    first,
    last,
    [&](const auto& a, const auto& b){
      return axis(a.Center()) < axis(b.Center());
    }
  );

  const auto n_splits =
    std::min<std::size_t>(kMaxSplit, std::log2(std::distance(first, last)));

  Split split;

  for (std::size_t i = 0; i < n_splits; i++) {
    const auto split_point =
      axis(bb.Min()) +
      (axis(bb.Max()) - axis(bb.Min())) * (i + 1) / (n_splits + 1);

    const auto middle = std::lower_bound(
      first,
      last,
      split_point,
      [&](const auto& object, const auto split_point){
        return axis(object.Center()) < split_point;
      }
    );

    const auto bb_left = BoundingBox(first, middle);
    const auto bb_right = BoundingBox(middle, last);
    const auto n_left = std::distance(first, middle);
    const auto n_right = std::distance(middle, last);
    const auto cost =
      SurfaceAreaHeuristic(bb, bb_left, bb_right, n_left, n_right);

    if (cost < split.cost) {
      split.cost = cost;
      split.middle = middle;
      split.bb_left = bb_left;
      split.bb_right = bb_right;
    }
  }

  return split;
}

template <typename T, typename Object>
const T
BVH<T, Object>::SurfaceAreaHeuristic(
  const AABB<T>& bb_parent,
  const AABB<T>& bb_left,
  const AABB<T>& bb_right,
  std::size_t n_left,
  std::size_t n_right
) noexcept
{
  return
    2 * kTraverseCost +
    (n_left * SurfaceArea(bb_left) + n_right * SurfaceArea(bb_right) ) /
    SurfaceArea(bb_parent) * kIntersectionCost;
}

template <typename T, typename Object>
BVH<T, Object>::Node::Node(
  std::unique_ptr<Node>&& left,
  std::unique_ptr<Node>&& right,
  const AABB<T>& bb
) noexcept
: left_(std::move(left))
, right_(std::move(right))
, first_()
, last_()
, bb_(bb)
{}

template <typename T, typename Object>
BVH<T, Object>::Node::Node(
  ObjectIterator first,
  ObjectIterator last,
  const AABB<T>& bb
) noexcept
: left_()
, right_()
, first_(first)
, last_(last)
, bb_(bb)
{}

template <typename T, typename Object>
std::tuple<Hit<T>, const Object*>
BVH<T, Object>::Node::Cast(const Ray<T>& ray, T distance) const noexcept
{
  if (first_ != last_) {
    Hit<T> closest_hit;
    const Object* closest_object = nullptr;

    std::for_each(first_, last_, [&](const auto& object){
      const auto hit = object.Intersect(ray);
      if (hit && hit.distance < distance) {
        distance = hit.distance;
        closest_hit = hit;
        closest_object = &object;
      }
    });

    return std::make_tuple(closest_hit, closest_object);
  }

  bool left_hit = false;
  T left_in = std::numeric_limits<T>::max();
  if (left_) {
    std::tie(left_hit, left_in, std::ignore) =
      Intersect(left_->bb(), ray, distance);
  }

  bool right_hit = false;
  T right_in = std::numeric_limits<T>::max();
  if (right_) {
    std::tie(right_hit, right_in, std::ignore) =
      Intersect(right_->bb(), ray, distance);
  }

  if (!left_hit && !right_hit) {
    return std::make_tuple(Hit<T>(), nullptr);
  } else if (left_hit && !right_hit) {
    return left_->Cast(ray, distance);
  } else if (!left_hit && right_hit) {
    return right_->Cast(ray, distance);
  }

  const auto& near = left_in < right_in ? left_ : right_;
  const auto& far = left_in < right_in ? right_ : left_;

  Hit<T> near_hit;
  const Object* near_object = nullptr;
  std::tie(near_hit, near_object) = near->Cast(ray, distance);
  if (!near_hit) {
    return far->Cast(ray, distance);
  }
  if (near_hit.distance < std::max(left_in, right_in)) {
    return std::make_tuple(near_hit, near_object);
  }

  Hit<T> far_hit;
  const Object* far_object = nullptr;
  std::tie(far_hit, far_object) = far->Cast(ray, near_hit.distance);
  if (!far_hit) {
    return std::make_tuple(near_hit, near_object);
  } else {
    return std::make_tuple(far_hit, far_object);
  }
}

template <typename T, typename Object>
BVH<T, Object>::Split::Split() noexcept
: cost(std::numeric_limits<T>::max())
, middle()
, bb_left(AABB<T>::Empty())
, bb_right(AABB<T>::Empty())
{}

}
}
