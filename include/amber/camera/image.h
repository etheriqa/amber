/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <vector>

namespace amber {
namespace camera {

template <typename Radiant>
class Image {
private:
  size_t width_, height_;
  std::vector<Radiant> pixels_;

public:
  Image(size_t width, size_t height) noexcept
    : width_(width), height_(height), pixels_(width * height) {}

  size_t const& width() const noexcept { return width_; }
  size_t const& height() const noexcept { return height_; }

  Radiant& at(size_t x, size_t y) {
    if (x >= width_ || y >= height_) {
      throw std::out_of_range("amber::camera::Image::at: invalid coordinates");
    }
    return pixels_.at(x + y * width_);
  }

  Radiant const& at(size_t x, size_t y) const {
    if (x >= width_ || y >= height_) {
      throw std::out_of_range("amber::camera::Image::at: invalid coordinates");
    }
    return pixels_.at(x + y * width_);
  }

  Image<Radiant> downSample(size_t n) const {
    if ((width_ % n) * (height_ % n) > 0) {
      throw std::invalid_argument(
        "amber::camera::Image::downSample: invalid n");
    }
    Image<Radiant> image(width_ / n, height_ / n);
    for (size_t i = 0; i < width_; i++) {
      for (size_t j = 0; j < height_; j++) {
        image.at(i / n, j / n) = at(i, j);
      }
    }
    return image;
  }
};

}
}
