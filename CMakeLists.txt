cmake_minimum_required(VERSION 2.8)

project(amber)

set(CMAKE_CXX_FLAGS "-std=c++14 -Werror -Wpedantic -Wall -Wextra -Wno-unused-parameter -mavx")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g2")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g0 -DNDEBUG")

if (AMBER_LTO)
  if(CMAKE_CXX_COMPILER_ID STREQUAL Clang)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -flto")
  endif()
  if(CMAKE_CXX_COMPILER_ID STREQUAL GNU)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -flto")
  endif()
endif()

# pthread
find_package(Threads REQUIRED)

# Google Test
find_package(GTest)
if(GTEST_FOUND)
  enable_testing()
  include_directories(${GTEST_INCLUDE_DIRS})
endif()

# Boost
find_package(Boost REQUIRED program_options)
include_directories(${Boost_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV REQUIRED core highgui)
include_directories(${OpenCV_INCLUDE_DIRS})

# assimp
find_package(assimp REQUIRED)
include_directories(${ASSIMP_INCLUDE_DIRS})
link_directories(${ASSIMP_LIBRARY_DIRS})

list(APPEND SOURCES src/amber/cli/algorithm_factory.cc)
list(APPEND SOURCES src/amber/cli/application.cc)
list(APPEND SOURCES src/amber/cli/context.cc)
list(APPEND SOURCES src/amber/cli/image.cc)
list(APPEND SOURCES src/amber/cli/import.cc)
list(APPEND SOURCES src/amber/cli/option.cc)
list(APPEND SOURCES src/amber/etude/cornel_box.cc)
list(APPEND SOURCES src/amber/postprocess/filmic.cc)
list(APPEND SOURCES src/amber/postprocess/gamma.cc)
list(APPEND SOURCES src/amber/postprocess/reinhard.cc)
list(APPEND SOURCES src/amber/prelude/aabb.cc)
list(APPEND SOURCES src/amber/prelude/parallel.cc)
list(APPEND SOURCES src/amber/rendering/algorithm_bdpt.cc)
list(APPEND SOURCES src/amber/rendering/algorithm_lt.cc)
list(APPEND SOURCES src/amber/rendering/algorithm_pssmlt.cc)
list(APPEND SOURCES src/amber/rendering/algorithm_sppm.cc)
list(APPEND SOURCES src/amber/rendering/algorithm_mppm.cc)
list(APPEND SOURCES src/amber/rendering/algorithm_pt.cc)
list(APPEND SOURCES src/amber/rendering/algorithm_ups.cc)
list(APPEND SOURCES src/amber/rendering/sensor.cc)
list(APPEND SOURCES src/amber/scene/lens_pinhole.cc)
list(APPEND SOURCES src/amber/scene/lens_thin.cc)
list(APPEND SOURCES src/amber/scene/material_lambertian.cc)
list(APPEND SOURCES src/amber/scene/material_phong.cc)
list(APPEND SOURCES src/amber/scene/material_refraction.cc)
list(APPEND SOURCES src/amber/scene/material_specular.cc)
list(APPEND SOURCES src/amber/scene/primitive_cylinder.cc)
list(APPEND SOURCES src/amber/scene/primitive_disk.cc)
list(APPEND SOURCES src/amber/scene/primitive_sphere.cc)
list(APPEND SOURCES src/amber/scene/primitive_triangle.cc)

include_directories(include)

add_library(libamber ${SOURCES})
set_target_properties(libamber PROPERTIES
  OUTPUT_NAME amber)

add_executable(amber src/amber.cc)
target_link_libraries(amber
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${ASSIMP_LIBRARIES}
  libamber)

# define some useful custom targets
set(REFERENCE_IMAGE reference.exr)
set(AMBER_COMMAND_OPTION ${AMBER_COMMAND_OPTION} --initial-radius 0.016)
set(AMBER_COMMAND_OPTION ${AMBER_COMMAND_OPTION} --spp 16)
if(EXISTS ${REFERENCE_IMAGE})
  set(AMBER_COMMAND_OPTION ${AMBER_COMMAND_OPTION} --reference ${REFERENCE_IMAGE})
endif()
set(AMBER_COMMAND time ./amber ${AMBER_COMMAND_OPTION})

add_custom_target(pt
  COMMAND ${AMBER_COMMAND} --algorithm pt --output pt
  DEPENDS amber
)

add_custom_target(lt
  COMMAND ${AMBER_COMMAND} --algorithm lt --output lt
  DEPENDS amber
)

add_custom_target(bdpt
  COMMAND ${AMBER_COMMAND} --algorithm bdpt --output bdpt
  DEPENDS amber
)

add_custom_target(pssmlt
  COMMAND ${AMBER_COMMAND} --algorithm pssmlt --output pssmlt
  DEPENDS amber
)

add_custom_target(mmlt
  COMMAND ${AMBER_COMMAND} --algorithm mmlt --output mmlt
  DEPENDS amber
)

add_custom_target(pm
  COMMAND ${AMBER_COMMAND} --algorithm pm --output pm --photons 1048576
  DEPENDS amber
)

add_custom_target(ppm
  COMMAND ${AMBER_COMMAND} --algorithm ppm --output ppm
  DEPENDS amber
)

add_custom_target(sppm
  COMMAND ${AMBER_COMMAND} --algorithm sppm --output sppm
  DEPENDS amber
)

add_custom_target(mppm
  COMMAND ${AMBER_COMMAND} --algorithm mppm --output mppm
  DEPENDS amber
)

add_custom_target(ups
  COMMAND ${AMBER_COMMAND} --algorithm ups --output ups
  DEPENDS amber
)
