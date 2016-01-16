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

#include <limits>

#include <boost/operators.hpp>

#include "amber/prelude/forward.h"

namespace amber {
namespace prelude {

class Pixel
{
public:
  Pixel() noexcept;
  Pixel(pixel_size_type x, pixel_size_type y) noexcept;

  const pixel_size_type X() const noexcept { return x_; }
  const pixel_size_type Y() const noexcept { return y_; }

  operator bool() const noexcept;

private:
  pixel_size_type x_, y_;
};

template <typename T>
class PixelValue
: private boost::field_operators2<PixelValue<T>, T>
{
public:
  PixelValue() noexcept;
  PixelValue(const T& value) noexcept;
  PixelValue(const class Pixel& pixel, const T& value) noexcept;

  PixelValue<T>& operator+=(const T& value) noexcept;
  PixelValue<T>& operator-=(const T& value) noexcept;
  PixelValue<T>& operator*=(const T& value) noexcept;
  PixelValue<T>& operator/=(const T& value) noexcept;

  const class Pixel Pixel() const noexcept { return pixel_; }
  const T& Value() const noexcept { return value_; }

  operator bool() const noexcept;

private:
  class Pixel pixel_;
  T value_;
};



inline
Pixel::Pixel() noexcept
: x_(std::numeric_limits<pixel_size_type>::max())
, y_(std::numeric_limits<pixel_size_type>::max())
{}

inline
Pixel::Pixel(pixel_size_type x, pixel_size_type y) noexcept
: x_(x)
, y_(y)
{}

inline
Pixel::operator bool() const noexcept
{
  return
    x_ < std::numeric_limits<pixel_size_type>::max() &&
    y_ < std::numeric_limits<pixel_size_type>::max();
}

template <typename T>
PixelValue<T>::PixelValue() noexcept
: pixel_()
, value_(0)
{}

template <typename T>
PixelValue<T>::PixelValue(const T& value) noexcept
: pixel_()
, value_(value)
{}

template <typename T>
PixelValue<T>::PixelValue(const class Pixel& pixel, const T& value) noexcept
: pixel_(pixel)
, value_(value)
{}

template <typename T>
PixelValue<T>&
PixelValue<T>::operator+=(const T& value) noexcept
{
  value_ += value;
  return *this;
}

template <typename T>
PixelValue<T>&
PixelValue<T>::operator-=(const T& value) noexcept
{
  value_ -= value;
  return *this;
}

template <typename T>
PixelValue<T>&
PixelValue<T>::operator*=(const T& value) noexcept
{
  value_ *= value;
  return *this;
}

template <typename T>
PixelValue<T>&
PixelValue<T>::operator/=(const T& value) noexcept
{
  value_ /= value;
  return *this;
}

template <typename T>
PixelValue<T>::operator bool() const noexcept
{
  return Positive(value_);
}

}
}
