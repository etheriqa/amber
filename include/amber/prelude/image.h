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

#include <valarray>

#include <boost/operators.hpp>

#include "amber/prelude/image_sparse.h"
#include "amber/prelude/pixel.h"

namespace amber {
namespace prelude {

template <typename T>
class Image
: private boost::addable<Image<T>>
, private boost::multiplicative2<Image<T>, T>
{
public:
  Image(pixel_size_type width, pixel_size_type height);

  Image<T>& operator+=(const Image<T>& image);
  Image<T>& operator+=(const SparseImage<T>& image);
  Image<T>& operator*=(const T& multiplier) noexcept;
  Image<T>& operator/=(const T& divisor) noexcept;

  T& operator[](const Pixel& pixel);
  const T& operator[](const Pixel& pixel) const;

  pixel_size_type Width() const noexcept { return width_; }
  pixel_size_type Height() const noexcept { return height_; }
  pixel_size_type Size() const noexcept { return width_ * height_; }

  void Clear() noexcept;

private:
  pixel_size_type width_, height_;
  std::valarray<T> values_;
};



template <typename T>
Image<T>::Image(pixel_size_type width, pixel_size_type height)
: width_(width)
, height_(height)
, values_(static_cast<T>(0), width * height)
{}

template <typename T>
Image<T>&
Image<T>::operator+=(const Image<T>& image)
{
  if (image.width_ != width_ || image.height_ != height_) {
    throw std::out_of_range("Image::operator+=: mismatch dimensions");
  }

  values_ += image.values_;
  return *this;
}

template <typename T>
Image<T>&
Image<T>::operator+=(const SparseImage<T>& image)
{
  for (const auto& e : image) {
    (*this)[e.Pixel()] += e.Value();
  }

  return *this;
}

template <typename T>
Image<T>&
Image<T>::operator*=(const T& multiplier) noexcept
{
  values_ *= multiplier;
  return *this;
}

template <typename T>
Image<T>&
Image<T>::operator/=(const T& divisor) noexcept
{
  values_ /= divisor;
  return *this;
}

template <typename T>
T&
Image<T>::operator[](const Pixel& pixel)
{
  return const_cast<T&>(static_cast<const Image<T>&>(*this)[pixel]);
}

template <typename T>
const T&
Image<T>::operator[](const Pixel& pixel) const
{
  if (pixel.X() >= width_ || pixel.Y() >= height_) {
    throw std::out_of_range("Image::operator[]: invalid pixel given");
  }

  return values_[pixel.X() + pixel.Y() * width_];
}

template <typename T>
void
Image<T>::Clear() noexcept
{
  values_ = static_cast<T>(0);
}

}
}
