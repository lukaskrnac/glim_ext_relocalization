cmake_minimum_required(VERSION 3.16)
project(glim_relocalization VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(glim_relocalization SHARED
  src/relocalization_module.cpp
)

target_include_directories(glim_relocalization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(glim_relocalization
  glim::glim
  gtsam
  nlohmann_json::nlohmann_json
)

# Ak build prostredie nemá nlohmann_json ako CMake target (bežné pri systémovom
# libnlohmann-json3-dev), odkomentuj toto miesto to a zmaž find_package/target_link vyššie:
# target_include_directories(glim_relocalization PUBLIC /usr/include/nlohmann)

install(TARGETS glim_relocalization
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(COMMAND ament_package)
  ament_package()
endif()
