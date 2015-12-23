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

#include <vector>

#include "core/image.h"

namespace amber {
namespace core {
namespace component {

template <typename Radiant>
class ImageAverageBuffer
{
private:
  std::size_t width_, height_;
  std::size_t n_;
  std::vector<Image<Radiant>> buffer_;

public:
  ImageAverageBuffer(std::size_t const width, std::size_t const height) noexcept
  : width_(width)
  , height_(height)
  , n_(0)
  , buffer_()
  {}

  operator Image<Radiant>() const noexcept;

  void Buffer(Image<Radiant>&& image);
};

template <typename Radiant>
ImageAverageBuffer<Radiant>::operator Image<Radiant>() const noexcept
{
  Image<Radiant> image(width_, height_);

  for (std::size_t i = 0; i < buffer_.size(); i++) {
    if (n_ & (static_cast<std::size_t>(1) << i)) {
      image += buffer_.at(i);
    }
  }

  for (std::size_t v = 0; v < height_; v++) {
    for (std::size_t u = 0; u < width_; u++) {
      image.at(u, v) /= n_;
    }
  }

  return image;
}

template <typename Radiant>
void
ImageAverageBuffer<Radiant>::Buffer(Image<Radiant>&& image)
{
  n_++;

  for (std::size_t i = 0;; i++) {
    if (n_ & (static_cast<std::size_t>(1) << i)) {
      if (n_ == (n_ & -n_)) {
        buffer_.emplace_back(std::move(image));
      } else {
        buffer_.at(i) = std::move(image);
      }
      break;
    }

    image += buffer_.at(i);
  }
}

}
}
}
