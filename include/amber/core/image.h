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

#include <numeric>
#include <vector>

#include <boost/operators.hpp>

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
  Image(std::size_t width, std::size_t height) noexcept
    : width_(width), height_(height), pixels_(width * height) {}

  std::size_t const& width() const noexcept { return width_; }
  std::size_t const& height() const noexcept { return height_; }

  Image<Radiant>& operator+=(Image<Radiant> const& rhs)
  {
    if (rhs.width_ != width_ || rhs.height_ != height_) {
      throw std::out_of_range("amber::Image::operator+=: invalid dimensions");
    }

    for (std::size_t i = 0; i < width_ * height_; i++) {
      pixels_[i] += rhs.pixels_[i];
    }

    return *this;
  }

  Radiant& at(std::size_t x, std::size_t y) {
    if (x >= width_ || y >= height_) {
      throw std::out_of_range("amber::Image::at: invalid coordinates");
    }
    return pixels_.at(x + y * width_);
  }

  Radiant const& at(std::size_t x, std::size_t y) const {
    if (x >= width_ || y >= height_) {
      throw std::out_of_range("amber::Image::at: invalid coordinates");
    }
    return pixels_.at(x + y * width_);
  }

  Radiant const totalPower() const noexcept {
    return std::accumulate(pixels_.begin(), pixels_.end(), Radiant());
  }

  Image<Radiant> downSample(std::size_t n) const {
    if ((width_ % n) * (height_ % n) > 0) {
      throw std::invalid_argument(
        "amber::Image::downSample: invalid n");
    }
    Image<Radiant> image(width_ / n, height_ / n);
    for (std::size_t i = 0; i < width_; i++) {
      for (std::size_t j = 0; j < height_; j++) {
        image.at(i / n, j / n) = at(i, j);
      }
    }
    return image;
  }
};

}
}
