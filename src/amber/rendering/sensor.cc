// Copyright (c) 2016 TAKAMORI Kaede <etheriqa@gmail.com>
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

#include "amber/prelude/sampling.h"
#include "amber/prelude/vector2.h"
#include "amber/rendering/sensor.h"

namespace amber {
namespace rendering {

Sensor::Sensor(
  pixel_size_type pixel_width,
  pixel_size_type pixel_height,
  real_type scene_width,
  real_type scene_height
) noexcept
: pixel_width_(pixel_width)
, pixel_height_(pixel_height)
, scene_width_(scene_width)
, scene_height_(scene_height)
{}

const pixel_size_type
Sensor::Size() const noexcept
{
  return pixel_width_ * pixel_height_;
}

const real_type
Sensor::SceneArea() const noexcept
{
  return scene_width_ * scene_height_;
}

Pixel
Sensor::ResponsePixel(const Vector2& point) const noexcept
{
  const auto uv = Vector2(
    point.X() / scene_width_ + static_cast<real_type>(0.5),
    point.Y() / scene_height_ + static_cast<real_type>(0.5)
  );

  if (Min(uv) < 0 || Max(uv) >= 1) {
    return Pixel();
  }

  return UVToPixel(uv);
}

auto
Sensor::Bind(const Pixel& pixel) const noexcept
-> PixelBound
{
  return PixelBound(*this, pixel);
}

std::tuple<Vector2, Pixel>
Sensor::Uniform(Sampler& sampler) const
{
  const auto uv = Vector2(
    prelude::Uniform<real_type>(sampler),
    prelude::Uniform<real_type>(sampler)
  );

  return std::make_tuple(UVToPoint(uv), UVToPixel(uv));
}

Pixel
Sensor::UVToPixel(const Vector2& uv) const noexcept
{
  return Pixel(
    std::min<pixel_size_type>(pixel_width_ - 1, uv.X() * pixel_width_),
    std::min<pixel_size_type>(pixel_height_ - 1, uv.Y() * pixel_height_)
  );
}

const Vector2
Sensor::UVToPoint(const Vector2& uv) const noexcept
{
  return Vector2(
    (uv.X() - static_cast<real_type>(0.5)) * scene_width_,
    (uv.Y() - static_cast<real_type>(0.5)) * scene_height_
  );
}

Sensor::PixelBound::PixelBound(
  const Sensor& sensor,
  const Pixel& pixel
) noexcept
: Sensor(sensor)
, pixel_(pixel)
{}

std::tuple<Vector2, Pixel>
Sensor::PixelBound::Uniform(Sampler& sampler) const
{
  const auto uv = Vector2(
    (pixel_.X() + prelude::Uniform<real_type>(sampler)) / pixel_width_,
    (pixel_.Y() + prelude::Uniform<real_type>(sampler)) / pixel_height_
  );

  return std::make_tuple(UVToPoint(uv), pixel_);
}

}
}
