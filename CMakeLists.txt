cmake_minimum_required(VERSION 3.16)
project(glim_relocalization VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
# nlohmann_json je header-only - nepoužívame find_package/CMake target, len
# systémové hlavičky nainštalované cez "apt install nlohmann-json3-dev".
# /usr/include je v predvolenej include ceste kompilátora, takže samotný
# #include <nlohmann/json.hpp> v .cpp súbore stačí bez ďalšej CMake väzby.

add_library(glim_relocalization SHARED
  src/relocalization_module.cpp
)

target_include_directories(glim_relocalization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(glim_relocalization
  glim::glim
  gtsam
)

# Ak by build hlásil "nlohmann/json.hpp: No such file or directory", znamená
# to, že v tomto prostredí nie je nainštalovaný balík nlohmann-json3-dev
# (v CI workflow sa inštaluje pred týmto krokom cez apt-get).

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
