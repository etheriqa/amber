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

#include <cmath>
#include <numeric>
#include <vector>

#include <boost/operators.hpp>

#include "core/accumulator.h"

namespace amber {
namespace core {

template <typename Radiant>
class Image
: private boost::addable<Image<Radiant>>
{
private:
  std::size_t width_, height_;
  std::vector<Radiant> pixels_;

public:
  Image(std::size_t const width, std::size_t const height) noexcept;

  std::size_t const& width() const noexcept { return width_; }
  std::size_t const& height() const noexcept { return height_; }
  std::size_t const size() const noexcept { return width_ * height_; }

  Image<Radiant>& operator+=(Image<Radiant> const& rhs);

  Radiant& at(std::size_t const u, std::size_t const v);
  Radiant const& at(std::size_t const u, std::size_t const v) const;

  Radiant const TotalPower() const noexcept;
  Image<Radiant> const DownSample(std::size_t const n) const;
};

template <typename Radiant>
typename Radiant::value_type
MeanSquareError(
  Image<Radiant> const& image,
  Image<Radiant> const& reference
)
{
  if (image.width() != reference.width() ||
      image.height() != reference.height()) {
    throw std::out_of_range("MeanSquareError: invalid dimensions");
  }

  Accumulator<typename Radiant::value_type> mse;

  for (std::size_t v = 0; v < image.height(); v++) {
    for (std::size_t u = 0; u < image.width(); u++) {
      mse += MeanSquareError(image.at(u, v), reference.at(u, v));
    }
  }

  return mse / image.size();
}

template <typename Radiant>
typename Radiant::value_type
AverageRelativeError(
  Image<Radiant> const& image,
  Image<Radiant> const& reference
)
{
  if (image.width() != reference.width() ||
      image.height() != reference.height()) {
    throw std::out_of_range("AverageRelativeError: invalid dimensions");
  }

  Accumulator<typename Radiant::value_type> are;

  for (std::size_t v = 0; v < image.height(); v++) {
    for (std::size_t u = 0; u < image.width(); u++) {
      are += AverageRelativeError(image.at(u, v), reference.at(u, v));
    }
  }

  return are / image.size();
}

template <typename Radiant>
Image<Radiant>::Image(
  std::size_t const width,
  std::size_t const height
) noexcept
: width_(width)
, height_(height)
, pixels_(width * height)
{}

template <typename Radiant>
Image<Radiant>&
Image<Radiant>::operator+=(Image<Radiant> const& rhs)
{
  if (rhs.width_ != width_ || rhs.height_ != height_) {
    throw std::out_of_range("amber::Image::operator+=: invalid dimensions");
  }

  for (std::size_t i = 0; i < width_ * height_; i++) {
    pixels_[i] += rhs.pixels_[i];
  }

  return *this;
}

template <typename Radiant>
Radiant&
Image<Radiant>::at(std::size_t const u, std::size_t const v)
{
  if (u >= width_ || v >= height_) {
    throw std::out_of_range("amber::Image::at: invalid coordinates");
  }
  return pixels_.at(u + v * width_);
}

template <typename Radiant>
Radiant const&
Image<Radiant>::at(std::size_t const u, std::size_t const v) const
{
  if (u >= width_ || v >= height_) {
    throw std::out_of_range("amber::Image::at: invalid coordinates");
  }
  return pixels_.at(u + v * width_);
}

template <typename Radiant>
Radiant const
Image<Radiant>::TotalPower() const noexcept
{
  return std::accumulate(pixels_.begin(), pixels_.end(), Radiant());
}

template <typename Radiant>
Image<Radiant> const
Image<Radiant>::DownSample(std::size_t const n) const
{
  if ((width_ % n) * (height_ % n) > 0) {
    throw std::invalid_argument(
      "amber::Image::DownSample: invalid n");
  }
  Image<Radiant> image(width_ / n, height_ / n);
  for (std::size_t i = 0; i < width_; i++) {
    for (std::size_t j = 0; j < height_; j++) {
      image.at(i / n, j / n) = at(i, j);
    }
  }
  return image;
}

}
}
