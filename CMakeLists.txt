cmake_minimum_required(VERSION 2.8)

project(amber CXX)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++1y -pedantic -Wall -mavx")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(include/amber)

add_executable(amber
    src/amber/cli/export.cc
    src/amber/cli/import.cc
    src/amber/cli/option.cc
    src/amber/scene/cornel_box.cc
    src/amber/scene/cornel_box_complex.cc
    src/main.cc
)

target_link_libraries(amber pthread)

find_package(Boost REQUIRED program_options)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_libraries(amber ${Boost_LIBRARIES})
endif()

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(amber ${OpenCV_LIBS})
endif()

set(REFERENCE_IMAGE reference.exr)

set(AMBER_COMMAND_OPTION ${AMBER_COMMAND_OPTION} --exposure 1000)
set(AMBER_COMMAND_OPTION ${AMBER_COMMAND_OPTION} --initial-radius 0.008)
set(AMBER_COMMAND_OPTION ${AMBER_COMMAND_OPTION} --spp 65536)
set(AMBER_COMMAND_OPTION ${AMBER_COMMAND_OPTION} --time 10)
if(EXISTS ${REFERENCE_IMAGE})
    set(AMBER_COMMAND_OPTION ${AMBER_COMMAND_OPTION} --reference ${REFERENCE_IMAGE})
endif()
set(AMBER_COMMAND time ./amber ${AMBER_COMMAND_OPTION})

add_custom_target(pt
    COMMAND ${AMBER_COMMAND} --shader pt --output pt
    DEPENDS amber
)

add_custom_target(lt
    COMMAND ${AMBER_COMMAND} --shader lt --output lt
    DEPENDS amber
)

add_custom_target(bdpt
    COMMAND ${AMBER_COMMAND} --shader bdpt --output bdpt
    DEPENDS amber
)

add_custom_target(pssmlt
    COMMAND ${AMBER_COMMAND} --shader pssmlt --output pssmlt
    DEPENDS amber
)

add_custom_target(mmlt
    COMMAND ${AMBER_COMMAND} --shader mmlt --output mmlt
    DEPENDS amber
)

add_custom_target(pm
    COMMAND ${AMBER_COMMAND} --shader pm --output pm --photons 1048576
    DEPENDS amber
)

add_custom_target(ppm
    COMMAND ${AMBER_COMMAND} --shader ppm --output ppm
    DEPENDS amber
)

add_custom_target(sppm
    COMMAND ${AMBER_COMMAND} --shader sppm --output sppm
    DEPENDS amber
)

add_custom_target(mppm
    COMMAND ${AMBER_COMMAND} --shader mppm --output mppm
    DEPENDS amber
)

add_custom_target(ups
    COMMAND ${AMBER_COMMAND} --shader ups --output ups
    DEPENDS amber
)
