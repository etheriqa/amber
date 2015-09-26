#pragma once

#include <algorithm>
#include <vector>
#include "random.h"
#include "ray.h"

namespace amber {

template <typename Container>
class LightSet
{
public:
  using container_type  = Container;

  using object_type     = typename container_type::object_type;
  using real_type       = typename container_type::real_type;

private:
  struct Object
  {
    real_type accumulated_area;
    object_type object;

    Object(real_type accumulated_area, const object_type& object) :
      accumulated_area(accumulated_area),
      object(object)
    {}
  };

  std::vector<Object> m_objects;

public:
  LightSet(const container_type& container)
  {
    real_type accumulated_area = 0;
    for (const auto& object : container) {
      if (!object.is_emissive()) {
        continue;
      }

      accumulated_area += object.surface_area();
      m_objects.push_back(Object(accumulated_area, object));
    }
  }

  real_type total_area() const
  {
    return m_objects.back().accumulated_area;
  }

  object_type sample(Random& random) const
  {
    const auto it = std::upper_bound(
      m_objects.begin(),
      m_objects.end(),
      Object(random.uniform(total_area()), object_type()),
      [](const Object& a, const Object& b){ return a.accumulated_area < b.accumulated_area; }
    );
    return it->object;
  }
};

}
