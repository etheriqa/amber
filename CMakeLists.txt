cmake_minimum_required(VERSION 2.8)

project(amber CXX)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++1y -Wall -Wextra -mavx")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(include/amber)

add_executable(amber src/main.cc)

target_link_libraries(amber pthread)

add_custom_target(pt
    COMMAND time ./amber --algorithm pt --spp 512
    COMMAND convert pt.ppm pt.png
    COMMAND open pt.png
    DEPENDS amber
)

add_custom_target(bdpt
    COMMAND time ./amber --algorithm bdpt --spp 64
    COMMAND convert bdpt.ppm bdpt.png
    COMMAND open bdpt.png
    DEPENDS amber
)

add_custom_target(pm
    COMMAND time ./amber --algorithm pm --photons 8388608
    COMMAND convert pm.ppm pm.png
    COMMAND open pm.png
    DEPENDS amber
)

add_custom_target(pssmlt
    COMMAND time ./amber --algorithm pssmlt --mutations 16777216
    COMMAND convert pssmlt.ppm pssmlt.png
    COMMAND open pssmlt.png
    DEPENDS amber
)

add_custom_target(ppm
    COMMAND time ./amber --algorithm ppm --photons 1048576 --iterations 16
    COMMAND convert ppm.ppm ppm.png
    COMMAND open ppm.png
    DEPENDS amber
)

add_custom_target(sppm
    COMMAND time ./amber --algorithm sppm --photons 1048576 --iterations 16
    COMMAND convert sppm.ppm sppm.png
    COMMAND open sppm.png
    DEPENDS amber
)
