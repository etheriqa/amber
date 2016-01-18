cmake_minimum_required(VERSION 2.8)

project(amber CXX)

set(CMAKE_CXX_FLAGS "-std=c++1y -pedantic -Wall -mavx")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -flto")

set(TARGET amber)

# pthread
list(APPEND LIBRARIES pthread)

# Google Test
find_package(GTest)
if(GTEST_FOUND)
    enable_testing()
    include_directories(${GTEST_INCLUDE_DIRS})
endif()

# Boost
find_package(Boost REQUIRED program_options)
include_directories(${Boost_INCLUDE_DIRS})
list(APPEND LIBRARIES ${Boost_LIBRARIES})

# OpenCV
find_package(OpenCV REQUIRED core highgui)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND LIBRARIES ${OpenCV_LIBS})

# assimp
find_package(assimp REQUIRED)
include_directories(${ASSIMP_INCLUDE_DIRS})
link_directories(${ASSIMP_LIBRARY_DIRS})
list(APPEND LIBRARIES ${ASSIMP_LIBRARIES})

include_directories(include)

add_executable(${TARGET}
    src/amber/cli/algorithm_factory.cc
    src/amber/cli/application.cc
    src/amber/cli/context.cc
    src/amber/cli/image.cc
    src/amber/cli/import.cc
    src/amber/cli/option.cc
    src/amber/etude/cornel_box.cc
    src/amber/postprocess/filmic.cc
    src/amber/postprocess/gamma.cc
    src/amber/postprocess/reinhard.cc
    src/amber/prelude/aabb.cc
    src/amber/prelude/parallel.cc
    src/amber/rendering/algorithm_bdpt.cc
    src/amber/rendering/algorithm_lt.cc
    src/amber/rendering/algorithm_pssmlt.cc
    src/amber/rendering/algorithm_sppm.cc
    src/amber/rendering/algorithm_mppm.cc
    # src/amber/rendering/algorithm_mmlt.cc
    src/amber/rendering/algorithm_pt.cc
    src/amber/rendering/algorithm_ups.cc
    src/amber/rendering/sensor.cc
    src/amber/scene/lens_pinhole.cc
    src/amber/scene/lens_thin.cc
    src/amber/scene/material_lambertian.cc
    src/amber/scene/material_phong.cc
    src/amber/scene/material_refraction.cc
    src/amber/scene/material_specular.cc
    src/amber/scene/primitive_cylinder.cc
    src/amber/scene/primitive_disk.cc
    src/amber/scene/primitive_sphere.cc
    src/amber/scene/primitive_triangle.cc
    src/main.cc
)

target_link_libraries(${TARGET} ${LIBRARIES})


# define some useful custom targets
set(REFERENCE_IMAGE reference.exr)
set(AMBER_COMMAND_OPTION ${AMBER_COMMAND_OPTION} --initial-radius 0.008)
set(AMBER_COMMAND_OPTION ${AMBER_COMMAND_OPTION} --spp 16)
if(EXISTS ${REFERENCE_IMAGE})
    set(AMBER_COMMAND_OPTION ${AMBER_COMMAND_OPTION} --reference ${REFERENCE_IMAGE})
endif()
set(AMBER_COMMAND time ./${TARGET} ${AMBER_COMMAND_OPTION})

add_custom_target(pt
    COMMAND ${AMBER_COMMAND} --algorithm pt --output pt
    DEPENDS ${TARGET}
)

add_custom_target(lt
    COMMAND ${AMBER_COMMAND} --algorithm lt --output lt
    DEPENDS ${TARGET}
)

add_custom_target(bdpt
    COMMAND ${AMBER_COMMAND} --algorithm bdpt --output bdpt
    DEPENDS ${TARGET}
)

add_custom_target(pssmlt
    COMMAND ${AMBER_COMMAND} --algorithm pssmlt --output pssmlt
    DEPENDS ${TARGET}
)

add_custom_target(mmlt
    COMMAND ${AMBER_COMMAND} --algorithm mmlt --output mmlt
    DEPENDS ${TARGET}
)

add_custom_target(pm
    COMMAND ${AMBER_COMMAND} --algorithm pm --output pm --photons 1048576
    DEPENDS ${TARGET}
)

add_custom_target(ppm
    COMMAND ${AMBER_COMMAND} --algorithm ppm --output ppm
    DEPENDS ${TARGET}
)

add_custom_target(sppm
    COMMAND ${AMBER_COMMAND} --algorithm sppm --output sppm
    DEPENDS ${TARGET}
)

add_custom_target(mppm
    COMMAND ${AMBER_COMMAND} --algorithm mppm --output mppm
    DEPENDS ${TARGET}
)

add_custom_target(ups
    COMMAND ${AMBER_COMMAND} --algorithm ups --output ups
    DEPENDS ${TARGET}
)
