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

#include <iterator>
#include <vector>

#include "core/primitive/triangle.h"

namespace amber {

template <typename RealType>
class Model
{
public:
  using real_type     = RealType;

  using triangle_type = core::primitive::Triangle<real_type>;
  using vector3_type  = core::Vector3<real_type>;

  class InputIterator : std::iterator<std::input_iterator_tag, triangle_type*>
  {
private:
    const Model* m_model;
    size_t m_position;

public:
    explicit InputIterator(const Model* model, size_t position) :
      m_model(model), m_position(position)
    {}

    InputIterator& operator++() noexcept
    {
      if (m_position > 0 && ++m_position >= m_model->m_faces.size()) {
        m_position = 0;
      }

      return *this;
    }

    InputIterator operator++(int) noexcept
    {
      auto it = *this;
      return ++it;
    }

    triangle_type* operator*() noexcept
    {
      if (m_position == 0) {
        return nullptr;
      }

      size_t i0, i1, i2;
      std::tie(i0, i1, i2) = m_model->m_faces[m_position - 1];
      // FIXME memory leak
      return new triangle_type(m_model->m_vertices[i0], m_model->m_vertices[i1], m_model->m_vertices[i2]);
    }

    bool operator==(const InputIterator& it) const noexcept
    {
      return m_model == it.m_model && m_position == it.m_position;
    }

    bool operator!=(const InputIterator& it) const noexcept
    {
      return !(*this == it);
    }
  };

private:
  std::vector<vector3_type> m_vertices;
  std::vector<std::tuple<size_t, size_t, size_t>> m_faces;

public:
  explicit Model() {}

  void add_vertex(real_type x, real_type y, real_type z) noexcept
  {
    m_vertices.push_back(vector3_type(x, y, z));
  }

  void add_face(size_t i0, size_t i1, size_t i2) noexcept
  {
    m_faces.push_back(std::make_tuple(i0, i1, i2));
  }

  InputIterator begin() const noexcept
  {
    return InputIterator(this, 1);
  }

  InputIterator end() const noexcept
  {
    return InputIterator(this, 0);
  }
};

}
