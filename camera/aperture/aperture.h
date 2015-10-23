#pragma once

#include <string>
#include "random.h"

namespace amber {
namespace camera {
namespace aperture {

template <typename RealType>
struct Aperture
{
  using aperture_type = Aperture<RealType>;
  using real_type     = RealType;

  using vector3_type  = geometry::Vector3<real_type>;

  virtual std::string to_string() const = 0;
  virtual vector3_type sample_point(Random& g) const = 0;
};

}
}
}