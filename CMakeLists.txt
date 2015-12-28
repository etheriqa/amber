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

set(AMBER_COMMAND_OPTION ${AMBER_COMMAND_OPTION} --exposure 1000)
set(AMBER_COMMAND_OPTION ${AMBER_COMMAND_OPTION} --initial-radius 0.008)
set(AMBER_COMMAND_OPTION ${AMBER_COMMAND_OPTION} --spp 65536)
set(AMBER_COMMAND_OPTION ${AMBER_COMMAND_OPTION} --time 10)
set(AMBER_COMMAND time ./amber ${AMBER_COMMAND_OPTION})

add_custom_target(pt
    COMMAND ${AMBER_COMMAND} --shader pt --output pt
    COMMAND open pt.png
    DEPENDS amber
)

add_custom_target(lt
    COMMAND ${AMBER_COMMAND} --shader lt --output lt
    COMMAND open lt.png
    DEPENDS amber
)

add_custom_target(bdpt
    COMMAND ${AMBER_COMMAND} --shader bdpt --output bdpt
    COMMAND open bdpt.png
    DEPENDS amber
)

add_custom_target(pssmlt
    COMMAND ${AMBER_COMMAND} --shader pssmlt --output pssmlt
    COMMAND open pssmlt.png
    DEPENDS amber
)

add_custom_target(mmlt
    COMMAND ${AMBER_COMMAND} --shader mmlt --output mmlt
    COMMAND open mmlt.png
    DEPENDS amber
)

add_custom_target(pm
    COMMAND ${AMBER_COMMAND} --shader pm --output pm --photons 1048576
    COMMAND open pm.png
    DEPENDS amber
)

add_custom_target(ppm
    COMMAND ${AMBER_COMMAND} --shader ppm --output ppm
    COMMAND open ppm.png
    DEPENDS amber
)

add_custom_target(sppm
    COMMAND ${AMBER_COMMAND} --shader sppm --output sppm
    COMMAND open sppm.png
    DEPENDS amber
)

add_custom_target(mppm
    COMMAND ${AMBER_COMMAND} --shader mppm --output mppm
    COMMAND open mppm.png
    DEPENDS amber
)

add_custom_target(ups
    COMMAND ${AMBER_COMMAND} --shader ups --output ups
    COMMAND open ups.png
    DEPENDS amber
)
