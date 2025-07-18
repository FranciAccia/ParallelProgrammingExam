cmake_minimum_required(VERSION 3.24)
project(BoidsProject)

set(CMAKE_CXX_STANDARD 17)

add_executable(BoidsProject main.cpp)

set(SFML_DIR "/Users/acciafranci/Documents/AI UNIFI/PARALLEL PROGRAMMING/SFML-3.0.0/lib/cmake/SFML")
#For Audio, an additional added library is needed (e.g. Vorbis)
find_package(SFML 3 COMPONENTS System Window Graphics REQUIRED)
include_directories("/Users/acciafranci/Documents/AI UNIFI/PARALLEL PROGRAMMING/SFML-3.0.0/include")
link_directories("/Users/acciafranci/Documents/AI UNIFI/PARALLEL PROGRAMMING/SFML-3.0.0/lib")
target_link_libraries(BoidsProject PRIVATE SFML::System SFML::Window SFML::Graphics)