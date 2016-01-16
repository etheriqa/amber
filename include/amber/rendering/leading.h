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

#include "amber/prelude/forward.h"
#include "amber/prelude/ray.h"
#include "amber/prelude/vector3.h"
#include "amber/rendering/forward.h"

namespace amber {
namespace rendering {

/** Leading event.
 */
template <typename Radiant>
class Leading
{
public:
  /** Constructor.
   */
  Leading(
    const Vector3& position,
    const UnitVector3& normal,
    const UnitVector3& direction_in,
    const Radiant& weight
  ) noexcept;

  /** Queries.
   */
  const rendering::Ray Ray() const noexcept;
  const Vector3 Position() const noexcept;
  const UnitVector3 Normal() const noexcept;
  const UnitVector3 DirectionIn() const noexcept;
  const Radiant Weight() const noexcept;

private:
  Vector3 position_;
  UnitVector3 normal_;
  UnitVector3 direction_in_;
  Radiant weight_;
};





template <typename Radiant>
Leading<Radiant>::Leading(
  const Vector3& position,
  const UnitVector3& normal,
  const UnitVector3& direction_in,
  const Radiant& weight
) noexcept
: position_(position)
, normal_(normal)
, direction_in_(direction_in)
, weight_(weight)
{}

template <typename Radiant>
const rendering::Ray
Leading<Radiant>::Ray() const noexcept
{
  return rendering::Ray(position_, direction_in_);
}

template <typename Radiant>
const Vector3
Leading<Radiant>::Position() const noexcept
{
  return position_;
}

template <typename Radiant>
const UnitVector3
Leading<Radiant>::Normal() const noexcept
{
  return normal_;
}

template <typename Radiant>
const UnitVector3
Leading<Radiant>::DirectionIn() const noexcept
{
  return direction_in_;
}

template <typename Radiant>
const Radiant
Leading<Radiant>::Weight() const noexcept
{
  return weight_;
}

}
}
