#pragma once

#include "image.h"
#include "vector.h"

namespace amber {

template <typename Radiant>
class Sensor
{
public:
  using radiant_type    = Radiant;

  using real_type       = typename radiant_type::real_type;

  using image_reference = Image<radiant_type>*;
  using vector3_type    = Vector3<real_type>;

  static constexpr real_type kFilmSize = 0.036;

private:
  image_reference m_image;
  real_type       m_width,
                  m_height;

public:
  Sensor(image_reference i, real_type fs = kFilmSize) :
    m_image(i),
    m_width(fs),
    m_height(fs / i->m_width * i->m_height)
  {}

  size_t image_width() const
  {
    return m_image->m_width;
  }

  size_t image_height() const
  {
    return m_image->m_height;
  }

  size_t image_pixels() const
  {
    return m_image->m_width * m_image->m_height;
  }

  vector3_type sample_point(size_t x, size_t y) const
  {
    return vector3_type(
      - ((x + real_type(0.5)) / image_width() - real_type(0.5)) * m_width,
      ((y + real_type(0.5)) / image_height() - real_type(0.5)) * m_height,
      0
    );
  }

  void expose(size_t x, size_t y, const radiant_type& power) const
  {
    m_image->expose(x, y, power);
  }
};

}
