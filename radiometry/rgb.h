#pragma once

#include "vector.h"

namespace amber {
namespace radiometry {

template <typename RealType>
using RGB = Vector3<RealType>;

using SRGB = RGB<unsigned char>;

}
}
