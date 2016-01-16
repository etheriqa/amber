// Copyright (c) 2016 TAKAMORI Kaede <etheriqa@gmail.com>
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

#include "amber/etude/cornel_box.h"
#include "amber/prelude/matrix4.h"
#include "amber/raytracer/acceleration_bvh.h"
#include "amber/scene/lens_thin.h"
#include "amber/scene/material_diffuse_light.h"
#include "amber/scene/material_eye.h"
#include "amber/scene/material_lambertian.h"
#include "amber/scene/material_phong.h"
#include "amber/scene/material_refraction.h"
#include "amber/scene/material_specular.h"
#include "amber/scene/primitive_sphere.h"
#include "amber/scene/primitive_triangle.h"
#include "amber/scene/scene.h"

namespace amber {
namespace etude {

RGBScene
CornelBox(
  real_type focal_length,
  real_type aperture_radius,
  std::size_t aperture_n_blades
)
{
  std::vector<std::unique_ptr<Primitive>> primitives;
  std::vector<std::unique_ptr<RGBMaterial>> materials;
  std::vector<RGBObject> objects;

  // lens
  auto lens = scene::MakeThinLens<RGB>(
    Matrix4(
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 4,
      0, 0, 0, 1
    ),
    focal_length,
    4, // focus distance
    aperture_radius,
    aperture_n_blades
  );
  materials.emplace_back(scene::MakeEye<RGB>());
  for (const auto& object : lens->ApertureObjects()) {
    objects.emplace_back(*object);
  }

  // light source
  materials.emplace_back(scene::MakeDiffuseLight(RGB(1e11, 1e11, 1e11)));
  primitives.emplace_back(scene::MakeTriangle(
    Vector3( 0.01, 0.99,  0.01),
    Vector3(-0.01, 0.99,  0.01),
    Vector3(-0.01, 0.99, -0.01)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());
  primitives.emplace_back(scene::MakeTriangle(
    Vector3(-0.01, 0.99, -0.01),
    Vector3( 0.01, 0.99, -0.01),
    Vector3( 0.01, 0.99,  0.01)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());

  // left wall
  materials.emplace_back(scene::MakeLambertian(RGB(.5, 0, 0)));
  primitives.emplace_back(scene::MakeTriangle(
    Vector3(-1,  1,  1),
    Vector3(-1, -1,  1),
    Vector3(-1, -1, -1)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());
  primitives.emplace_back(scene::MakeTriangle(
    Vector3(-1, -1, -1),
    Vector3(-1,  1, -1),
    Vector3(-1,  1,  1)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());

  // right wall
  materials.emplace_back(scene::MakeLambertian(RGB(0, .5, 0)));
  primitives.emplace_back(scene::MakeTriangle(
    Vector3(1,  1,  1),
    Vector3(1,  1, -1),
    Vector3(1, -1, -1)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());
  primitives.emplace_back(scene::MakeTriangle(
    Vector3(1, -1, -1),
    Vector3(1, -1,  1),
    Vector3(1,  1,  1)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());

  // back wall
  materials.emplace_back(scene::MakePhong(RGB(.95), 256));
  primitives.emplace_back(scene::MakeTriangle(
    Vector3( 1,  1, -1),
    Vector3(-1,  1, -1),
    Vector3(-1, -1, -1)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());
  primitives.emplace_back(scene::MakeTriangle(
    Vector3(-1, -1, -1),
    Vector3( 1, -1, -1),
    Vector3( 1,  1, -1)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());

  // floor wall
  materials.emplace_back(scene::MakeLambertian(RGB(.5)));
  primitives.emplace_back(scene::MakeTriangle(
    Vector3( 1, -1,  1),
    Vector3( 1, -1, -1),
    Vector3(-1, -1, -1)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());
  primitives.emplace_back(scene::MakeTriangle(
    Vector3(-1, -1, -1),
    Vector3(-1, -1,  1),
    Vector3( 1, -1,  1)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());

  // ceiling wall
  materials.emplace_back(scene::MakeLambertian(RGB(.5)));
  primitives.emplace_back(scene::MakeTriangle(
    Vector3( 1, 1,  1),
    Vector3(-1, 1,  1),
    Vector3(-1, 1, -1)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());
  primitives.emplace_back(scene::MakeTriangle(
    Vector3(-1, 1, -1),
    Vector3( 1, 1, -1),
    Vector3( 1, 1,  1)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());

  // water surface
  materials.emplace_back(scene::MakeRefraction<RGB>(1.333));
  primitives.emplace_back(scene::MakeTriangle(
    Vector3( 1, -0.5,  1),
    Vector3( 1, -0.5, -1),
    Vector3(-1, -0.5, -1)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());
  primitives.emplace_back(scene::MakeTriangle(
    Vector3(-1, -0.5, -1),
    Vector3(-1, -0.5,  1),
    Vector3( 1, -0.5,  1)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());
  primitives.emplace_back(scene::MakeTriangle(
    Vector3( 1, -0.5, 1),
    Vector3(-1, -0.5, 1),
    Vector3(-1, -1, 1)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());
  primitives.emplace_back(scene::MakeTriangle(
    Vector3(-1, -1, 1),
    Vector3( 1, -1, 1),
    Vector3( 1, -0.5, 1)
  ));
  objects.emplace_back(primitives.back().get(), materials.back().get());

  // diffuse sphere
  primitives.emplace_back(scene::MakeSphere(Vector3( 0.4, -0.6, -0.5), 0.4));
  materials.emplace_back(scene::MakeLambertian(RGB(.5)));
  objects.emplace_back(primitives.back().get(), materials.back().get());

  // specular sphere
  primitives.emplace_back(scene::MakeSphere(Vector3(-0.4, -0.7,  0.1), 0.3));
  materials.emplace_back(scene::MakeSpecular(RGB(.95)));
  objects.emplace_back(primitives.back().get(), materials.back().get());

  // refraction sphere
  primitives.emplace_back(scene::MakeSphere(Vector3( 0.1, -0.8,  0.6), 0.2));
  materials.emplace_back(scene::MakeRefraction<RGB>(1.125));
  objects.emplace_back(primitives.back().get(), materials.back().get());

  return RGBScene::Create<raytracer::BVH<real_type, RGBObject>>(
    std::move(primitives),
    std::move(materials),
    std::move(objects),
    std::move(lens)
  );
}

}
}
