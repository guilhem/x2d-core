cmake_minimum_required(VERSION 3.16)

file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/library.properties" X2D_VERSION_LINES REGEX "^version=")
list(LENGTH X2D_VERSION_LINES X2D_VERSION_COUNT)
if(NOT X2D_VERSION_COUNT EQUAL 1)
  message(FATAL_ERROR "library.properties must contain exactly one version= entry")
endif()
string(REGEX REPLACE "^version=" "" X2D_CORE_VERSION "${X2D_VERSION_LINES}")
if(NOT X2D_CORE_VERSION MATCHES "^(0|[1-9][0-9]*)\\.(0|[1-9][0-9]*)\\.(0|[1-9][0-9]*)$")
  message(FATAL_ERROR "library.properties version must be X.Y.Z without leading zeros")
endif()

set(X2D_RELEASE_TAG "" CACHE STRING "Optional release tag to validate against the library version")
if(NOT X2D_RELEASE_TAG STREQUAL "" AND NOT X2D_RELEASE_TAG STREQUAL "v${X2D_CORE_VERSION}")
  message(FATAL_ERROR "Release tag must be v${X2D_CORE_VERSION}, got '${X2D_RELEASE_TAG}'")
endif()

project(x2d_core VERSION "${X2D_CORE_VERSION}" LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(x2d_core INTERFACE)
add_library(x2d::core ALIAS x2d_core)
set_target_properties(x2d_core PROPERTIES EXPORT_NAME core)
target_compile_features(x2d_core INTERFACE cxx_std_17)
target_include_directories(x2d_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

set(package_dir "${CMAKE_INSTALL_LIBDIR}/cmake/x2d-core")
install(TARGETS x2d_core EXPORT x2d-core-targets)
install(DIRECTORY src/x2d DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT x2d-core-targets NAMESPACE x2d:: DESTINATION ${package_dir})
configure_package_config_file(cmake/x2d-core-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/x2d-core-config.cmake
  INSTALL_DESTINATION ${package_dir})
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/x2d-core-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion ARCH_INDEPENDENT)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/x2d-core-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/x2d-core-config-version.cmake DESTINATION ${package_dir})

set(testing_default OFF)
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(testing_default ON)
endif()
option(X2D_BUILD_TESTING "Build X2D core checks" ${testing_default})
if(X2D_BUILD_TESTING)
  enable_testing()
  foreach(check IN ITEMS radio radio_runtime queue cc1101 controller)
    add_executable(check_${check} tests/check_${check}.cpp)
    target_link_libraries(check_${check} PRIVATE x2d::core)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(check_${check} PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    elseif(MSVC)
      target_compile_options(check_${check} PRIVATE /UNDEBUG)
    endif()
    add_test(NAME ${check} COMMAND check_${check})
  endforeach()
  add_test(NAME consumer COMMAND ${CMAKE_COMMAND}
    -DCORE_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}
    -DCORE_BINARY=${CMAKE_CURRENT_BINARY_DIR}
    -DCXX=${CMAKE_CXX_COMPILER}
    -DCTEST=${CMAKE_CTEST_COMMAND}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_consumer.cmake)
endif()
