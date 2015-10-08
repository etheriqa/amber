#pragma once

#include <functional>
#include "material/material.h"
#include "shape/shape.h"

namespace amber {

template <typename Shape, typename Material>
class Object
{
public:
  using shape_type              = Shape;
  using material_type           = Material;

  using aabb_type               = typename shape_type::aabb_type;
  using hit_type                = typename shape_type::hit_type;
  using initial_ray_sample_type = typename shape_type::initial_ray_sample_type;
  using ray_type                = typename shape_type::ray_type;
  using real_type               = typename shape_type::real_type;
  using flux_type               = typename material_type::flux_type;
  using scattering_sample_type  = typename material_type::ScatteringSample;
  using vector3_type            = typename material_type::vector3_type;

  using shape_reference         = shape_type*;
  using material_reference      = material_type*;

  struct Hash
  {
    size_t operator()(const Object& o) const noexcept
    {
      return std::hash<size_t>()(reinterpret_cast<size_t>(o.m_shape)) +
        std::hash<size_t>()(reinterpret_cast<size_t>(o.m_material));
    }
  };

  struct EqualTo
  {
    bool operator()(const Object& a, const Object& b) const noexcept
    {
      return a == b;
    }
  };

private:
  shape_reference m_shape;
  material_reference m_material;

public:
  explicit Object() : m_shape(nullptr), m_material(nullptr) {}

  Object(const shape_reference& shape, const material_reference& material) :
    m_shape(shape), m_material(material)
  {}

  bool operator==(const Object& o) const noexcept
  {
    return m_shape == o.m_shape && m_material == o.m_material;
  }

  bool operator!=(const Object& o) const noexcept
  {
    return !(*this == o);
  }

  real_type surface_area() const noexcept
  {
    return m_shape->surface_area();
  }

  aabb_type aabb() const noexcept
  {
    return m_shape->aabb();
  }

  hit_type intersect(const ray_type& ray) const noexcept
  {
    return m_shape->intersect(ray);
  }

  initial_ray_sample_type sample_initial_ray(Random& random) const
  {
    return m_shape->sample_initial_ray(random);
  }

  bool is_emissive() const noexcept
  {
    return m_material->is_emissive();
  }

  bool is_specular() const noexcept
  {
    return m_material->is_specular();
  }

  flux_type emittance() const noexcept
  {
    return m_material->emittance();
  }

  flux_type bsdf(const vector3_type& direction_i, const vector3_type& direction_o, const vector3_type& normal) const noexcept
  {
    return m_material->bsdf(direction_i, direction_o, normal);
  }

  scattering_sample_type sample_scattering(const vector3_type& direction_i, const vector3_type& normal, Random& random) const
  {
    return m_material->sample_scattering(direction_i, normal, random);
  }
};

}
