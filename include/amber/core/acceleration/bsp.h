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
#include <iterator>
#include <limits>
#include <vector>

#include "core/acceleration.h"

namespace amber {
namespace core {
namespace acceleration {

template <
  typename Object,
  size_t LeafCapacity = 16,
  size_t MaxDepth = 24
>
class BSP : public Acceleration<Object>
{
public:
private:
  using typename Acceleration<Object>::aabb_type;
  using typename Acceleration<Object>::hit_type;
  using typename Acceleration<Object>::ray_type;

  using real_type = typename Object::real_type;

  enum struct Axis {
    X = 0,
    Y = 1,
    Z = 2,
  };

  struct Node {
    using object_list_type = std::vector<Object>;

    Node *m_left, *m_right;
    object_list_type *m_objects;
    aabb_type m_voxel;

    template <typename InputIterator>
    Node(InputIterator first, InputIterator last)
      : Node(first, last, aabb(first, last), Axis::X, 1) {}

    template <typename InputIterator>
    Node(InputIterator first,
         InputIterator last,
         aabb_type const& voxel, Axis axis, size_t depth)
      : m_left(nullptr),
        m_right(nullptr),
        m_objects(nullptr),
        m_voxel(voxel) {
      if (static_cast<size_t>(std::distance(first, last)) <= LeafCapacity ||
          depth > MaxDepth) {
        m_objects = new object_list_type(first, last);
        return;
      }

      auto left_voxel = voxel;
      auto right_voxel = voxel;
      auto const i = static_cast<size_t>(axis);
      left_voxel.max()[i] = (voxel.min()[i] + voxel.max()[i]) / 2;
      right_voxel.min()[i] = (voxel.min()[i] + voxel.max()[i]) / 2;
      switch (axis) {
      case Axis::X:
        axis = Axis::Y;
        break;
      case Axis::Y:
        axis = Axis::Z;
        break;
      case Axis::Z:
        axis = Axis::X;
        break;
      }

      {
        std::vector<Object> objects;
        std::copy_if(first, last, std::back_inserter(objects),
          [&](auto const& object){ return left_voxel * object.BoundingBox(); });
        m_left = new Node(objects.begin(),
                          objects.end(),
                          left_voxel,
                          axis,
                          depth + 1);
      }

      {
        std::vector<Object> objects;
        std::copy_if(first, last, std::back_inserter(objects),
          [&](auto const& object){ return right_voxel * object.BoundingBox(); });
        m_right = new Node(objects.begin(),
                           objects.end(),
                           right_voxel,
                           axis,
                           depth + 1);
      }
    }

    ~Node() {
      delete m_objects;
      delete m_left;
      delete m_right;
    }

    std::tuple<hit_type, Object>
    cast(ray_type const& ray, real_type t_max) const noexcept {
      if (m_objects != nullptr) {
        return Acceleration<Object>::Traverse(m_objects->begin(),
                                              m_objects->end(),
                                              ray,
                                              t_max);
      }

      bool left_hit, right_hit;
      real_type t_left, t_right;
      std::tie(left_hit, t_left, std::ignore) =
        m_left->m_voxel.Intersect(ray, t_max);
      std::tie(right_hit, t_right, std::ignore) =
        m_right->m_voxel.Intersect(ray, t_max);

      if (!left_hit && !right_hit) {
        return std::make_tuple(hit_type(), Object());
      } else if (left_hit && !right_hit) {
        return m_left->cast(ray, t_max);
      } else if (!left_hit && right_hit) {
        return m_right->cast(ray, t_max);
      }

      auto const& near = t_left < t_right ? m_left : m_right;
      auto const& far = t_left < t_right ? m_right : m_left;

      hit_type near_hit;
      Object near_object;
      std::tie(near_hit, near_object) = near->cast(ray, t_max);
      if (!near_hit) {
        return far->cast(ray, t_max);
      } else if (near_hit.distance < std::max(t_left, t_right)) {
        return std::make_tuple(near_hit, near_object);
      }

      hit_type far_hit;
      Object far_object;
      std::tie(far_hit, far_object) = far->cast(ray, near_hit.distance);
      if (far_hit) {
        return std::make_tuple(far_hit, far_object);
      } else {
        return std::make_tuple(near_hit, near_object);
      }
    }

    template <typename InputIterator>
    static aabb_type
    aabb(InputIterator first, InputIterator last) noexcept {
      aabb_type aabb;
      std::for_each(first, last, [&](auto const& object){
        aabb += object.BoundingBox();
      });
      return aabb;
    }

  };

  Node root_;

public:
  template <typename InputIterator>
  BSP(InputIterator first, InputIterator last) : root_(first, last) {}

  void Write(std::ostream& os) const noexcept
  {
    os
      << "BSP(leaf_capacity=" << LeafCapacity
      << ", max_depth=" << MaxDepth
      << ")";
  }

  aabb_type const&
  BoundingBox() const noexcept
  {
    return root_.m_voxel;
  }

  std::tuple<hit_type, Object>
  Cast(const ray_type& ray) const noexcept
  {
    return root_.cast(ray, std::numeric_limits<real_type>::max());
  }
};

}
}
}
