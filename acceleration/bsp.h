/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <limits>
#include <sstream>
#include <vector>
#include "acceleration/acceleration.h"

namespace amber {
namespace acceleration {

template <typename Object, size_t LeafCapacity = 16, size_t MaxDepth = 24>
class BSP : public Acceleration<Object>
{
public:
  using acceleration_type  = Acceleration<Object>;

  using hit_type           = typename acceleration_type::hit_type;
  using object_type        = typename acceleration_type::object_type;
  using ray_type           = typename acceleration_type::ray_type;
  using real_type          = typename acceleration_type::object_type::real_type;

  using object_buffer_type = std::vector<Object>;

private:
  enum struct Axis
  {
    X = 0,
    Y = 1,
    Z = 2,
  };

  struct Node
  {
    using aabb_type = typename object_type::aabb_type;

    Node *m_left, *m_right;
    object_buffer_type *m_objects;
    aabb_type m_voxel;

    explicit Node(const object_buffer_type& objects) :
      Node(objects, aabb(objects), Axis::X, 1)
    {}

    Node(const object_buffer_type& objects, const aabb_type& voxel, Axis axis, size_t depth) :
      m_left(nullptr),
      m_right(nullptr),
      m_objects(nullptr),
      m_voxel(voxel)
    {
      if (objects.size() <= LeafCapacity || depth > MaxDepth) {
        m_objects = new object_buffer_type(objects);
        return;
      }

      auto left_voxel = voxel;
      auto right_voxel = voxel;
      const auto i = static_cast<size_t>(axis);
      left_voxel.max[i] = (voxel.min[i] + voxel.max[i]) / 2;
      right_voxel.min[i] = (voxel.min[i] + voxel.max[i]) / 2;
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

      object_buffer_type left_objects, right_objects;
      for (const auto& object : objects) {
        const auto object_voxel = object.aabb();
        if (left_voxel * object_voxel) {
          left_objects.push_back(object);
        }
        if (right_voxel * object_voxel) {
          right_objects.push_back(object);
        }
      }

      m_left = new Node(left_objects, left_voxel, axis, depth + 1);
      m_right = new Node(right_objects, right_voxel, axis, depth + 1);
    }

    ~Node()
    {
      delete m_objects;
      delete m_left;
      delete m_right;
    }

    std::tuple<hit_type, object_type> cast(const ray_type& ray, real_type t_max) const noexcept
    {
      if (m_objects != nullptr) {
        return acceleration_type::traverse(m_objects->begin(), m_objects->end(), ray, t_max);
      }

      bool left_hit, right_hit;
      real_type t_left, t_right;
      std::tie(left_hit, t_left, std::ignore) = m_left->m_voxel.intersect(ray, t_max);
      std::tie(right_hit, t_right, std::ignore) = m_right->m_voxel.intersect(ray, t_max);

      if (!left_hit && !right_hit) {
        return std::make_tuple(hit_type(), object_type());
      } else if (left_hit && !right_hit) {
        return m_left->cast(ray, t_max);
      } else if (!left_hit && right_hit) {
        return m_right->cast(ray, t_max);
      }

      const auto& near = t_left < t_right ? m_left : m_right;
      const auto& far = t_left < t_right ? m_right : m_left;

      hit_type near_hit;
      object_type near_object;
      std::tie(near_hit, near_object) = near->cast(ray, t_max);
      if (!near_hit) {
        return far->cast(ray, t_max);
      } else if (near_hit.distance < std::max(t_left, t_right)) {
        return std::make_tuple(near_hit, near_object);
      }

      hit_type far_hit;
      object_type far_object;
      std::tie(far_hit, far_object) = far->cast(ray, near_hit.distance);
      if (far_hit) {
        return std::make_tuple(far_hit, far_object);
      } else {
        return std::make_tuple(near_hit, near_object);
      }
    }

    static aabb_type aabb(const object_buffer_type& objects) noexcept
    {
      aabb_type aabb;
      for (const auto& object : objects) {
        aabb += object.aabb();
      }
      return aabb;
    }
  };

  Node *m_root;

public:
  static std::string to_string() noexcept
  {
    std::stringstream ss;
    ss << "BSP(leaf_capacity=" << LeafCapacity << ", max_depth=" << MaxDepth << ")";
    return ss.str();
  }

  explicit BSP(const object_buffer_type& objects) : m_root(new Node(objects)) {}

  ~BSP()
  {
    delete m_root;
  }

  std::tuple<hit_type, object_type> cast(const ray_type& ray) const noexcept
  {
    return m_root->cast(ray, std::numeric_limits<real_type>::max());
  }
};

}
}
