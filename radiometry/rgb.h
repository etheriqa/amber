#pragma once

#include "geometry/vector.h"

namespace amber {
namespace radiometry {

template <typename RealType>
using RGB = geometry::Vector3<RealType>;

using SRGB = RGB<unsigned char>;

}
}
