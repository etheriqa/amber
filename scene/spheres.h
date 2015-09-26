#pragma once

#include <cmath>
#include "constant.h"
#include "material/lambertian.h"
#include "material/light.h"
#include "material/refraction.h"
#include "material/specular.h"
#include "rgb.h"
#include "shape/circle.h"
#include "shape/cylinder.h"
#include "shape/sphere.h"
#include "vector.h"

namespace amber {
namespace scene {

template <class Container>
Container spheres()
{
  using Vector3    = Vector3<typename Container::real_type>;

  using Circle     = shape::Circle<typename Container::real_type>;
  using Cylinder   = shape::Cylinder<typename Container::real_type>;
  using Sphere     = shape::Sphere<typename Container::real_type>;

  using RGB        = RGB<typename Container::real_type>;

  using Lambertian = material::Lambertian<RGB>;
  using Light      = material::Light<RGB>;
  using Refraction = material::Refraction<RGB>;
  using Specular   = material::Specular<RGB>;

  using ObjectList = container::ObjectList<RGB>;

  ObjectList objects;

  const auto a = Vector3(std::cos((0.5 + 0. * 2 / 3) * kPI), std::sin((0.5 + 0. * 2 / 3) * kPI), -1);
  const auto b = Vector3(std::cos((0.5 + 1. * 2 / 3) * kPI), std::sin((0.5 + 1. * 2 / 3) * kPI), -1);
  const auto c = Vector3(std::cos((0.5 + 2. * 2 / 3) * kPI), std::sin((0.5 + 2. * 2 / 3) * kPI), -1);

  // floor
  objects.insert(
    new Circle({
      Vector3(0, 0, -1),
      Vector3(0, 0, 1),
      5
    }),
    new Lambertian(RGB(.5, .5, .5))
  );

  // lights
  objects.insert(
    new Cylinder(a + Vector3(0, 0, 4), Vector3(0, 0, -1), 0.25, 1),
    new Lambertian(RGB(.1, .1, .1))
  );
  objects.insert(
    new Cylinder(b + Vector3(0, 0, 4), Vector3(0, 0, -1), 0.25, 1),
    new Lambertian(RGB(.1, .1, .1))
  );
  objects.insert(
    new Cylinder(c + Vector3(0, 0, 4), Vector3(0, 0, -1), 0.25, 1),
    new Lambertian(RGB(.1, .1, .1))
  );
  objects.insert(
    new Circle(a + Vector3(0, 0, 4), Vector3(0, 0, -1), 0.25),
    new Light(RGB(1, 0, 0))
  );
  objects.insert(
    new Circle(b + Vector3(0, 0, 4), Vector3(0, 0, -1), 0.25),
    new Light(RGB(0, 1, 0))
  );
  objects.insert(
    new Circle(c + Vector3(0, 0, 4), Vector3(0, 0, -1), 0.25),
    new Light(RGB(0, 0, 1))
  );

  // spheres
  const auto radius = 0.25;
  const auto material = new Refraction(1.5);
  objects.insert(
    new Sphere(a + Vector3(0, 0, radius), radius),
    material
  );
  objects.insert(
    new Sphere(b + Vector3(0, 0, radius), radius),
    material
  );
  objects.insert(
    new Sphere(c + Vector3(0, 0, radius), radius),
    material
  );

  return Container(std::move(objects));
}

}
}
