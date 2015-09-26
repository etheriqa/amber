#pragma once

#include <cmath>
#include <fstream>
#include <vector>
#include "rgb.h"

namespace amber {

template <class Pixel>
class Image
{
public:
  using pixel_type = Pixel;
  using image_type = Image<Pixel>;

  const size_t m_width,
               m_height;
private:
  std::vector<pixel_type> m_pixel;

public:
  Image(size_t w, size_t h) :
    m_width(w), m_height(h), m_pixel(w * h)
  {}

  const pixel_type& pixel(size_t x, size_t y) const
  {
    return m_pixel[x + y * m_width];
  }

  pixel_type& pixel(size_t x, size_t y)
  {
    return m_pixel[x + y * m_width];
  }

  void expose(size_t x, size_t y, const pixel_type& p)
  {
    pixel(x, y) += p;
  }

  image_type down_sample(size_t n) const
  {
    image_type image(m_width / n, m_height / n);
    for (size_t i = 0; i < m_width; i++) {
      for (size_t j = 0; j < m_height; j++) {
        image.expose(i / n, j / n, pixel(i, j) / static_cast<pixel_type>(n * n));
      }
    }
    return image;
  }
};

void save_ppm(const char* filename, const Image<SRGB>& image)
{
  std::ofstream ofs(filename, std::ofstream::trunc);

  ofs << "P3" << std::endl;
  ofs << image.m_width << " " << image.m_height << std::endl;
  ofs << 255 << std::endl;

  for (size_t j = 0; j < image.m_height; j++) {
    for (size_t i = 0; i < image.m_width; i++) {
      ofs << static_cast<size_t>(image.pixel(i, j).x) << ' ' << static_cast<size_t>(image.pixel(i, j).y) << ' ' << static_cast<size_t>(image.pixel(i, j).z) << std::endl;
    }
  }
}

template <typename RealType>
void save_rgbe(const char* filename, const Image<RGB<RealType>>& image)
{
  std::ofstream ofs(filename, std::ofstream::trunc);

  ofs << "#?RADIANCE" << std::endl;
  ofs << "FORMAT=32-bit_rle_rgbe" << std::endl << std::endl;
  ofs << "-Y " << image.m_height << " +X " << image.m_width << std::endl;

  for (size_t j = 0; j < image.m_height; j++) {
    for (size_t i = 0; i < image.m_width; i++) {
      const auto& p = image.pixel(i, j);

      int exponent;
      const auto significand = std::frexp(max(p), &exponent) * 256 / max(p);

      ofs << static_cast<unsigned char>(significand * p.x);
      ofs << static_cast<unsigned char>(significand * p.y);
      ofs << static_cast<unsigned char>(significand * p.z);
      ofs << static_cast<unsigned char>(exponent + 128);
    }
  }
}

}
