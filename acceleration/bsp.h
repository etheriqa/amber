#pragma once

#include <sstream>
#include <vector>
#include "acceleration/acceleration.h"

namespace amber {
namespace acceleration {

template <class Object, size_t LeafCapacity = 16, size_t MaxDepth = 24>
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
  enum Axis { kAxisX, kAxisY, kAxisZ };

  struct Node
  {
    using aabb_type = typename object_type::aabb_type;

    Node *m_left, *m_right;
    aabb_type m_aabb;
    object_buffer_type m_objects;

    explicit Node(const object_buffer_type& objects) :
      Node(objects, aabb_type::universal(), kAxisX, 1)
    {}

    Node(const object_buffer_type& objects, const aabb_type& aabb, Axis axis, size_t depth) :
      m_left(nullptr), m_right(nullptr), m_aabb(aabb)
    {
      auto objects_aabb = aabb_type::empty();
      for (const auto& object : objects) {
        objects_aabb += object.aabb();
      }
      m_aabb *= objects_aabb;

      if (objects.size() <= LeafCapacity || depth > MaxDepth) {
        m_objects = objects;
        return;
      }

      auto left_aabb = m_aabb;
      auto right_aabb = m_aabb;
      switch (axis) {
      case kAxisX:
        left_aabb.max.x = (m_aabb.min.x + m_aabb.max.x) / 2;
        right_aabb.min.x = (m_aabb.min.x + m_aabb.max.x) / 2;
        axis = kAxisY;
        break;
      case kAxisY:
        left_aabb.max.y = (m_aabb.min.y + m_aabb.max.y) / 2;
        right_aabb.min.y = (m_aabb.min.y + m_aabb.max.y) / 2;
        axis = kAxisZ;
        break;
      case kAxisZ:
        left_aabb.max.z = (m_aabb.min.z + m_aabb.max.z) / 2;
        right_aabb.min.z = (m_aabb.min.z + m_aabb.max.z) / 2;
        axis = kAxisX;
        break;
      }

      object_buffer_type left_objects, right_objects;
      for (const auto& object : objects) {
        const auto object_aabb = object.aabb();
        if (left_aabb * object_aabb) {
          left_objects.push_back(object);
        }
        if (right_aabb * object_aabb) {
          right_objects.push_back(object);
        }
      }

      m_left = new Node(left_objects, left_aabb, axis, depth + 1);
      m_right = new Node(right_objects, right_aabb, axis, depth + 1);
    }

    ~Node()
    {
      delete m_left;
      delete m_right;
    }

    std::tuple<hit_type, object_type> cast(const ray_type& ray) const noexcept
    {
      if (m_left == nullptr || m_right == nullptr) {
        return acceleration_type::traverse(m_objects.begin(), m_objects.end(), ray);
      }

      bool hit_left, hit_right;
      real_type t_left, t_right;
      std::tie(hit_left, t_left, std::ignore) = m_left->m_aabb.intersect(ray);
      std::tie(hit_right, t_right, std::ignore) = m_right->m_aabb.intersect(ray);

      if (!hit_left && !hit_right) {
        return std::make_tuple(hit_type(), object_type());
      }
      if (hit_left && !hit_right) {
        return m_left->cast(ray);
      }
      if (!hit_left && hit_right) {
        return m_right->cast(ray);
      }

      const auto& near_child = t_left < t_right ? m_left : m_right;
      hit_type near_hit;
      object_type near_object;
      std::tie(near_hit, near_object) = near_child->cast(ray);
      if (near_hit && aabb_type(near_hit.position) * near_child->m_aabb) {
        return std::make_tuple(near_hit, near_object);
      }

      const auto& far_child = t_left < t_right ? m_right : m_left;
      hit_type far_hit;
      object_type far_object;
      std::tie(far_hit, far_object) = far_child->cast(ray);
      if (far_hit && (!near_hit || far_hit.distance < near_hit.distance)) {
        return std::make_tuple(far_hit, far_object);
      } else if (near_hit) {
        return std::make_tuple(near_hit, near_object);
      }

      return std::make_tuple(hit_type(), object_type());
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
    return m_root->cast(ray);
  }
};

}
}
