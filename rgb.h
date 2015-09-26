#pragma once

#include <algorithm>
#include <cmath>
#include "vector.h"

namespace amber {

template <typename RealType>
using RGB = Vector3<RealType>;

using SRGB = RGB<unsigned char>;

}
