#pragma once

#include <vector>

namespace amber {
namespace camera {

template <typename Pixel>
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

}
}
