#pragma once

#include <algorithm>
#include <tuple>

namespace amber {
namespace acceleration {

template <typename Object>
struct Acceleration
{
  using object_type   = Object;

  using hit_type      = typename object_type::hit_type;
  using ray_type      = typename object_type::ray_type;

  virtual std::tuple<hit_type, object_type> cast(const ray_type&) const noexcept = 0;

  virtual bool test_visibility(const ray_type& ray, const object_type& object) const noexcept
  {
    return std::get<1>(cast(ray)) == object;
  }

protected:
  template <typename InputIterator>
  static std::tuple<hit_type, object_type> traverse(InputIterator first, InputIterator last, const ray_type& ray) noexcept
  {
    hit_type closest_hit;
    object_type closest_object;

    std::for_each(first, last, [&](const auto& object){
      const auto hit = object.intersect(ray);
      if (hit && (!closest_hit || hit.distance < closest_hit.distance)) {
        closest_hit = hit;
        closest_object = object;
      }
    });

    return std::make_tuple(closest_hit, closest_object);
  }
};


}
}
