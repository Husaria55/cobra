cmake_minimum_required(VERSION 3.20)

project(cobra LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(cobra_lib
  src/config.cpp
  src/flight_log.cpp
  src/mavlink.cpp
)
target_include_directories(cobra_lib PUBLIC include)
target_compile_options(cobra_lib PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(cobra src/main.cpp)
target_link_libraries(cobra PRIVATE cobra_lib)

enable_testing()
add_executable(cobra_mavlink_test tests/mavlink_test.cpp)
target_link_libraries(cobra_mavlink_test PRIVATE cobra_lib)
add_test(NAME cobra_mavlink_test COMMAND cobra_mavlink_test)
