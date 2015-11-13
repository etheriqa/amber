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
    ifs.ignore(std::numeric_limits<std::streamsize>::max(), '
');
  }

  return model;
}

}
}
