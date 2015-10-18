#pragma once

#include <functional>
#include "material/material.h"
#include "primitive/primitive.h"

namespace amber {

template <typename Primitive, typename Material>
class Object
{
public:
  using primitive_type          = Primitive;
  using material_type           = Material;

  using aabb_type               = typename primitive_type::aabb_type;
  using hit_type                = typename primitive_type::hit_type;
  using initial_ray_sample_type = typename primitive_type::initial_ray_sample_type;
  using ray_type                = typename primitive_type::ray_type;
  using real_type               = typename primitive_type::real_type;
  using radiant_type            = typename material_type::radiant_type;
  using scattering_sample_type  = typename material_type::ScatteringSample;
  using vector3_type            = typename material_type::vector3_type;

  using primitive_reference     = primitive_type*;
  using material_reference      = material_type*;

  struct Hash
  {
    size_t operator()(const Object& o) const noexcept
    {
      return std::hash<size_t>()(reinterpret_cast<size_t>(o.m_primitive)) +
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
  primitive_reference m_primitive;
  material_reference m_material;

public:
  explicit Object() : m_primitive(nullptr), m_material(nullptr) {}

  Object(const primitive_reference& primitive, const material_reference& material) :
    m_primitive(primitive), m_material(material)
  {}

  bool operator==(const Object& o) const noexcept
  {
    return m_primitive == o.m_primitive && m_material == o.m_material;
  }

  bool operator!=(const Object& o) const noexcept
  {
    return !(*this == o);
  }

  real_type surface_area() const noexcept
  {
    return m_primitive->surface_area();
  }

  aabb_type aabb() const noexcept
  {
    return m_primitive->aabb();
  }

  hit_type intersect(const ray_type& ray) const noexcept
  {
    return m_primitive->intersect(ray);
  }

  initial_ray_sample_type sample_initial_ray(Random& random) const
  {
    return m_primitive->sample_initial_ray(random);
  }

  bool is_emissive() const noexcept
  {
    return m_material->is_emissive();
  }

  material::SurfaceType surface_type() const noexcept
  {
    return m_material->surface_type();
  }

  radiant_type emittance() const noexcept
  {
    return m_material->emittance();
  }

  radiant_type bsdf(const vector3_type& direction_i, const vector3_type& direction_o, const vector3_type& normal) const noexcept
  {
    return m_material->bsdf(direction_i, direction_o, normal);
  }

  scattering_sample_type sample_scattering(const vector3_type& direction_i, const vector3_type& normal, Random& random) const
  {
    return m_material->sample_scattering(direction_i, normal, random);
  }
};

}
