#pragma once

#include <vector>
#include "material/material.h"
#include "shape/shape.h"
#include "vector.h"

namespace amber {
namespace container {

template <class Flux>
struct Object
{
  using flux_type               = Flux;
  using real_type               = typename Flux::real_type;

  using aabb_type               = AABB<real_type>;
  using hit_type                = Hit<real_type>;
  using initial_ray_sample_type = InitialRaySample<real_type>;
  using material_type           = material::Material<flux_type>;
  using ray_sample_type         = RaySample<flux_type>;
  using ray_type                = Ray<real_type>;
  using shape_type              = shape::Shape<real_type>;
  using vector3_type            = Vector3<real_type>;

  using material_reference      = material_type*;
  using shape_reference         = shape_type*;

  shape_reference shape;
  material_reference material;

  explicit Object() : shape(nullptr), material(nullptr) {}

  Object(shape_reference shape, material_reference material) :
    shape(shape), material(material)
  {}

  operator bool() const noexcept
  {
    return shape == nullptr || material == nullptr;
  }

  real_type surface_area() const noexcept
  {
    return shape->surface_area();
  }

  aabb_type aabb() const noexcept
  {
    return shape->aabb();
  }

  hit_type intersect(const ray_type& ray) const noexcept
  {
    return shape->intersect(ray);
  }

  initial_ray_sample_type sample_initial_ray(Random& random) const
  {
    return shape->sample_initial_ray(random);
  }

  bool is_emissive() const noexcept
  {
    return material->is_emissive();
  }

  bool is_specular() const noexcept
  {
    return material->is_specular();
  }

  flux_type emittance() const noexcept
  {
    return material->emittance();
  }

  flux_type bsdf(const vector3_type& direction_i, const vector3_type& direction_o, const vector3_type& normal) const noexcept
  {
    return material->bsdf(direction_i, direction_o, normal);
  }

  ray_sample_type sample_ray_bsdf(const hit_type& hit, const ray_type& ray, Random& random) const
  {
    return material->sample_ray_bsdf(hit, ray, random);
  }
};

template <class Flux>
class ObjectList
{
public:
  using object_type        = Object<Flux>;

  using flux_type          = typename object_type::flux_type;
  using material_reference = typename object_type::material_reference;
  using real_type          = typename object_type::real_type;
  using shape_reference    = typename object_type::shape_reference;

private:
  std::vector<object_type> m_objects;

public:
  ObjectList() {}

  auto begin() const noexcept
  {
    return m_objects.begin();
  }

  auto end() const noexcept
  {
    return m_objects.end();
  }

  void insert(shape_reference shape, material_reference material) noexcept
  {
    m_objects.push_back(object_type(shape, material));
  }
};

}
}
