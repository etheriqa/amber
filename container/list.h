#pragma once

#include <tuple>
#include "container/object.h"
#include "hit.h"
#include "ray.h"

namespace amber {
namespace container {

template <class Flux>
class List
{
public:
  using object_list_type   = ObjectList<Flux>;

  using flux_type          = typename object_list_type::flux_type;
  using object_type        = typename object_list_type::object_type;
  using real_type          = typename object_list_type::real_type;

  using hit_type           = Hit<real_type>;
  using ray_type           = Ray<real_type>;

private:
  object_list_type m_objects;

public:
  List(object_list_type&& objects) : m_objects(std::move(objects)) {}

  auto begin() const noexcept
  {
    return m_objects.begin();
  }

  auto end() const noexcept
  {
    return m_objects.end();
  }

  std::tuple<hit_type, object_type> intersect(const ray_type& ray) const noexcept
  {
    hit_type closest_hit;
    object_type closest_object;

    for (const auto& object : m_objects) {
      const auto hit = object.shape->intersect(ray);
      if (hit && (!closest_hit || hit.distance < closest_hit.distance)) {
        closest_hit = hit;
        closest_object = object;
      }
    }

    return std::make_tuple(closest_hit, closest_object);
  }
};

}
}
