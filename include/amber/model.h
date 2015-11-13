/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <iterator>
#include <vector>
#include "primitive/triangle.h"

namespace amber {

template <typename RealType>
class Model
{
public:
  using real_type     = RealType;

  using triangle_type = primitive::Triangle<real_type>;
  using vector3_type  = Vector3<real_type>;

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
