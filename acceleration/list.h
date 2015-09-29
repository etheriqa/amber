#pragma once

#include <vector>
#include "acceleration/acceleration.h"

namespace amber {
namespace acceleration {

template <class Object, class ObjectBuffer = std::vector<Object>>
class List : public Acceleration<Object>
{
public:
  using acceleration_type  = Acceleration<Object>;
  using object_buffer_type = ObjectBuffer;

  using hit_type           = typename acceleration_type::hit_type;
  using object_type        = typename acceleration_type::object_type;
  using ray_type           = typename acceleration_type::ray_type;

private:
  ObjectBuffer m_objects;

public:
  static std::tuple<hit_type, object_type> cast(const object_buffer_type& objects, const ray_type& ray) noexcept
  {
    hit_type closest_hit;
    object_type closest_object;

    for (const auto& object : objects) {
      const auto hit = object.intersect(ray);
      if (hit && (!closest_hit || hit.distance < closest_hit.distance)) {
        closest_hit = hit;
        closest_object = object;
      }
    }

    return std::make_tuple(closest_hit, closest_object);
  }

  explicit List(const object_buffer_type& objects) : m_objects(objects) {}

  std::tuple<hit_type, object_type> cast(const ray_type& ray) const noexcept
  {
    return cast(m_objects, ray);
  }
};

}
}
