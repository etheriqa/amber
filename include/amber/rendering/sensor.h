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

#pragma once

#include <tuple>

#include "amber/prelude/image.h"
#include "amber/prelude/pixel.h"
#include "amber/rendering/forward.h"

namespace amber {
namespace rendering {

/** Image sensor.
 */
class Sensor
{
private:
  class PixelBound;

public:
  /** Constructor.
   */
  Sensor(
    pixel_size_type pixel_width,
    pixel_size_type pixel_height,
    real_type scene_width,
    real_type scene_height
  ) noexcept;

  /** Factory.
   */
  template <typename Radiant>
  Image<Radiant> CreateImage() const;

  /** Queries.
   */
  const pixel_size_type Size() const noexcept;
  const real_type SceneArea() const noexcept;
  Pixel ResponsePixel(const Vector2& point) const noexcept;

  /** Binds the sample space to a specific pixel.
   */
  PixelBound Bind(const Pixel& pixel) const noexcept;

  /** Samples a point on the sensor.
   */
  virtual std::tuple<Vector2, Pixel> Uniform(Sampler& sampler) const;

private:
  pixel_size_type pixel_width_, pixel_height_;
  real_type scene_width_, scene_height_;

  Pixel UVToPixel(const Vector2& uv) const noexcept;
  const Vector2 UVToPoint(const Vector2& uv) const noexcept;
};

/** Image sensor bound a specified pixel.
 */
class Sensor::PixelBound
: public Sensor
{
public:
  /** Constructor.
   */
  PixelBound(const Sensor& sensor, const Pixel& pixel) noexcept;

  /** Samples a point on a region that the specified pixel is contributed.
   */
  std::tuple<Vector2, Pixel> Uniform(Sampler& sampler) const;

private:
  Pixel pixel_;
};





template <typename Radiant>
Image<Radiant>
Sensor::CreateImage() const
{
  return Image<Radiant>(pixel_width_, pixel_height_);
}

}
}
