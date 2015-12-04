// Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include "core/writer.h"
#include "core/ray.h"

namespace amber {
namespace core {

template <typename RealType>
class Lens : public Writer
{
public:
  using ray_type          = Ray<RealType>;
  using unit_vector3_type = UnitVector3<RealType>;
  using vector3_type      = Vector3<RealType>;

  RealType static constexpr kFocalLength = 0.050;

  virtual ~Lens() {}

  virtual RealType const SensorDistance() const noexcept = 0;

  virtual
  unit_vector3_type const // direction
  Outgoing(
    vector3_type const&,     // sensor_point
    vector3_type const&,     // aperture_point
    vector3_type const&,     // origin
    unit_vector3_type const& // axis
  ) const noexcept = 0;

  virtual
  vector3_type const // sensor_point
  Incoming(
    unit_vector3_type const&, // direction
    vector3_type const&, // aperture_point
    vector3_type const&, // origin
    unit_vector3_type const&  // axis
  ) const noexcept = 0;
};

}
}
