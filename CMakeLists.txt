cmake_minimum_required(VERSION 2.8)

project(amber CXX)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++1y -pedantic -Wall -Wno-unused-local-typedef -mavx")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(include/amber)

add_executable(amber src/main.cc)

target_link_libraries(amber pthread)

find_package(Boost REQUIRED program_options)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
    target_link_libraries(amber ${Boost_LIBRARIES})
endif()

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(amber ${OpenCV_LIBS})
endif()

add_custom_target(pt
    COMMAND time ./amber --shader pt --output pt --spp 256
    COMMAND open pt.png
    DEPENDS amber
)

add_custom_target(lt
    COMMAND time ./amber --shader lt --output lt --samples 67108864
    COMMAND open lt.png
    DEPENDS amber
)

add_custom_target(bdpt
    COMMAND time ./amber --shader bdpt --output bdpt --spp 64
    COMMAND open bdpt.png
    DEPENDS amber
)

add_custom_target(pssmlt
    COMMAND time ./amber --shader pssmlt --output pssmlt --mutations 16777216
    COMMAND open pssmlt.png
    DEPENDS amber
)

add_custom_target(pm
    COMMAND time ./amber --shader pm --output pm --photons 8388608
    COMMAND open pm.png
    DEPENDS amber
)

add_custom_target(ppm
    COMMAND time ./amber --shader ppm --output ppm --photons 1048576 --passes 16
    COMMAND open ppm.png
    DEPENDS amber
)

add_custom_target(sppm
    COMMAND time ./amber --shader sppm --output sppm --photons 1048576 --passes 16
    COMMAND open sppm.png
    DEPENDS amber
)
