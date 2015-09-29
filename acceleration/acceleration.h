#pragma once

#include <iterator>
#include <tuple>

namespace amber {
namespace acceleration {

template <class Object>
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
};

}
}
