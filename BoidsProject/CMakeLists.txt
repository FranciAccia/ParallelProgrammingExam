cmake_minimum_required(VERSION 3.24)
project(BoidsProject)

set(CMAKE_CXX_STANDARD 17)

find_package(SFML 2.5 COMPONENTS system window graphics REQUIRED)

add_executable(BoidsProject main.cpp)
target_link_libraries(BoidsProject PRIVATE sfml-system sfml-window sfml-graphics)
