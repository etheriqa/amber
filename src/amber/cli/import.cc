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

#include <algorithm>
#include <iterator>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "amber/cli/import.h"
#include "amber/prelude/matrix4.h"
#include "amber/prelude/vector3.h"
#include "amber/scene/lens_pinhole.h"
#include "amber/scene/lens_thin.h"
#include "amber/scene/material_diffuse_light.h"
#include "amber/scene/material_lambertian.h"
#include "amber/scene/material_phong.h"
#include "amber/scene/material_specular.h"
#include "amber/scene/object.h"
#include "amber/scene/primitive_triangle.h"

namespace amber {
namespace cli {

std::tuple<
  std::vector<std::unique_ptr<scene::Primitive>>,
  std::vector<std::unique_ptr<scene::Material<RGB>>>,
  std::vector<scene::Object<RGB>>,
  std::unique_ptr<scene::Lens<RGB>>
>
ImportScene(const std::string& filename)
{
  Assimp::Importer importer;
  const auto scene = importer.ReadFile(
    filename.data(),
    aiProcess_Triangulate |
    aiProcess_PreTransformVertices |
    0
  );
  if (scene == nullptr) {
    throw std::runtime_error(
      std::string("ImportScene: ") + importer.GetErrorString());
  }

  std::vector<std::unique_ptr<scene::Primitive>> primitives;
  std::vector<std::unique_ptr<scene::Material<RGB>>> materials;
  std::vector<scene::Object<RGB>> objects;

  std::transform(
    scene->mMaterials,
    scene->mMaterials + scene->mNumMaterials,
    std::back_inserter(materials),
    [&](const auto& material){
      aiColor4D color;
      int iv;
      float fv;

      // TODO refraction

      // DiffuseLight
      if (aiReturn_SUCCESS == material->Get(AI_MATKEY_COLOR_EMISSIVE, color) &&
          !color.IsBlack())
      {
        return scene::MakeDiffuseLight(RGB(color[0], color[1], color[2]));
      }

      // Specular
      if (aiReturn_SUCCESS == material->Get(AI_MATKEY_COLOR_REFLECTIVE, color) &&
          aiReturn_SUCCESS == material->Get(AI_MATKEY_REFLECTIVITY, fv) &&
          !(color = color * fv).IsBlack())
      {
        return scene::MakeSpecular(RGB(color[0], color[1], color[2]));
      }

      // Phong
      if (aiReturn_SUCCESS == material->Get(AI_MATKEY_SHADING_MODEL, iv) &&
          iv == aiShadingMode_Phong &&
          aiReturn_SUCCESS == material->Get(AI_MATKEY_COLOR_SPECULAR, color) &&
          aiReturn_SUCCESS == material->Get(AI_MATKEY_SHININESS, fv))
      {
        return scene::MakePhong(RGB(color[0], color[1], color[2]), fv);
      }

      // Lambertian
      if (aiReturn_SUCCESS == material->Get(AI_MATKEY_COLOR_DIFFUSE, color)) {
        return scene::MakeLambertian(RGB(color[0], color[1], color[2]));
      }

      return scene::MakeLambertian(RGB(.5));
    }
  );

  std::for_each(
    scene->mMeshes,
    scene->mMeshes + scene->mNumMeshes,
    [&](const auto& mesh){
      if (mesh->mPrimitiveTypes != aiPrimitiveType_TRIANGLE) {
        return;
      }
      for (std::size_t i = 0; i < mesh->mNumVertices; i += 3) {
        const auto& vs = mesh->mVertices + i;
        primitives.emplace_back(scene::MakeTriangle(
          Vector3(vs[0][0], vs[0][1], vs[0][2]),
          Vector3(vs[1][0], vs[1][1], vs[1][2]),
          Vector3(vs[2][0], vs[2][1], vs[2][2])
        ));
        objects.emplace_back(
          primitives.back().get(),
          materials[mesh->mMaterialIndex].get()
        );
      }
    }
  );

  Matrix4 transform;
  {
    if (!scene->HasCameras()) {
      throw std::runtime_error("scene file has no cameras");
    }
    const auto& camera = scene->mCameras[0];
    const auto zaxis = -camera->mLookAt;
    const auto xaxis = camera->mLookAt ^ camera->mUp;
    const auto yaxis = zaxis ^ xaxis;
    const auto& position = camera->mPosition;
    transform = Matrix4(
      xaxis.x, yaxis.x, zaxis.x, position.x,
      xaxis.y, yaxis.y, zaxis.y, position.y,
      xaxis.z, yaxis.z, zaxis.z, position.z,
      0, 0, 0, 1
    );
  }
  auto lens = scene::MakeThinLens<RGB>(
    transform,
    0.050,
    4,
    0.010,
    6
  );
  for (const auto& object : lens->ApertureObjects()) {
    objects.emplace_back(*object);
  }

  return std::make_tuple(
    std::move(primitives),
    std::move(materials),
    std::move(objects),
    std::move(lens)
  );
}

}
}
