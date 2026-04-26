# Copyright 2026 bburda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.16)
project(ota_update_plugin CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCompat)
find_package(ros2_medkit_gateway REQUIRED)
find_package(nlohmann_json REQUIRED)

# CatalogClient uses cpp-httplib for HTTP. Use gateway's vendored copy as fallback.
set(_gw_vendored "${ros2_medkit_gateway_DIR}/../vendored/cpp_httplib")
medkit_find_cpp_httplib(VENDORED_DIR "${_gw_vendored}")
unset(_gw_vendored)

# Static core library: plugin + tests both link against this.
add_library(ota_update_plugin_core STATIC
  src/ota_update_plugin.cpp
  src/catalog_client.cpp
  src/operation_dispatcher.cpp
  src/process_runner.cpp
)
target_include_directories(ota_update_plugin_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
ament_target_dependencies(ota_update_plugin_core ros2_medkit_gateway)
target_link_libraries(ota_update_plugin_core
  PUBLIC
    nlohmann_json::nlohmann_json
    cpp_httplib_target
)
set_target_properties(ota_update_plugin_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# MODULE target: loaded via dlopen at runtime by PluginManager.
# Symbols from gateway_lib are resolved from the host process at runtime.
add_library(ota_update_plugin MODULE src/plugin_exports.cpp)
target_link_libraries(ota_update_plugin PRIVATE ota_update_plugin_core)
set_target_properties(ota_update_plugin PROPERTIES
  PREFIX ""
  OUTPUT_NAME "ota_update_plugin"
)
# Allow unresolved symbols - they resolve from the host process at runtime
target_link_options(ota_update_plugin PRIVATE
  -Wl,--unresolved-symbols=ignore-all
)

install(TARGETS ota_update_plugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ota_update_plugin
    test/test_operation_dispatcher.cpp
    test/test_catalog_client.cpp
    test/test_plugin_smoke.cpp
  )
  target_link_libraries(test_ota_update_plugin ota_update_plugin_core)
  target_include_directories(test_ota_update_plugin PRIVATE src)
endif()

ament_package()
