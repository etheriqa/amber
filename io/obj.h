#pragma once

#include <fstream>
#include <limits>
#include "model.h"

namespace amber {
namespace io {

template <typename RealType>
Model<RealType> import_obj(const char* filename)
{
  std::ifstream ifs(filename);

  Model<RealType> model;

  char c;
  while (ifs >> c) {
    if (c == 'v') {
      RealType x, y, z;
      ifs >> x >> y >> z;
      model.add_vertex(x, y, z);
    }
    if (c == 'f') {
      size_t i0, i1, i2;
      ifs >> i0 >> i1 >> i2;
      model.add_face(i0 - 1, i1 - 1, i2 - 1);
    }
    ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  return model;
}

}
}
