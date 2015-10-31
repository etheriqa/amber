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
    COMMAND ./amber --algorithm pt --name pt
    COMMAND convert pt.ppm pt.png
    COMMAND open pt.png
    DEPENDS amber
)

add_custom_target(bdpt
    COMMAND ./amber --algorithm bdpt --name bdpt
    COMMAND convert bdpt.ppm bdpt.png
    COMMAND open bdpt.png
    DEPENDS amber
)

add_custom_target(pm
    COMMAND ./amber --algorithm pm --name pm
    COMMAND convert pm.ppm pm.png
    COMMAND open pm.png
    DEPENDS amber
)

add_custom_target(pssmlt
    COMMAND ./amber --algorithm pssmlt --name pssmlt
    COMMAND convert pssmlt.ppm pssmlt.png
    COMMAND open pssmlt.png
    DEPENDS amber
)

add_custom_target(ppm
    COMMAND ./amber --algorithm ppm --name ppm
    COMMAND convert ppm.ppm ppm.png
    COMMAND open ppm.png
    DEPENDS amber
)
