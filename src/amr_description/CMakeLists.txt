cmake_minimum_required(VERSION 3.8)
project(amr_description)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# install the executable in the lib folder to make it detectable through setup.bash
install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
