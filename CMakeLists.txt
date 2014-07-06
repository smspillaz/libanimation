# /CMakeLists.txt
# Main CMake file for libwobbly. Creates targets for the
# library and C interface and builds tests as well.
#
# See LICENCE.md for Copyright information.

project (libwobbly)
cmake_minimum_required (VERSION 2.8 FATAL_ERROR)

# Set up CMAKE_MODULE_PATH to import Google Test and Google Mock
set (LIBWOBBLY_CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set (LIBWOBBLY_POLYSQUARE_CMAKE_MODULE_PATH
     ${LIBWOBBLY_CMAKE_MODULE_PATH}/common)

set (CMAKE_MODULE_PATH
     ${LIBWOBBLY_POLYSQUARE_CMAKE_MODULE_PATH}
     ${CMAKE_MODULE_PATH})

include (PolysquareCommon)

find_package (Boost REQUIRED)

polysquare_compiler_bootstrap ()
polysquare_rules_bootstrap (${LIBWOBBLY_POLYSQUARE_CMAKE_MODULE_PATH}
                            ${CMAKE_BINARY_DIR})
polysquare_gmock_bootstrap (${LIBWOBBLY_POLYSQUARE_CMAKE_MODULE_PATH})
polysquare_coverage_bootstrap (${LIBWOBBLY_POLYSQUARE_CMAKE_MODULE_PATH})

# Include directories
set (LIBWOBBLY_EXTERNAL_INCLUDE_DIRS
     ${Boost_INCLUDE_DIRS})
set (LIBWOBBLY_EXPORTED_INCLUDE_DIRS
     ${CMAKE_CURRENT_SOURCE_DIR}/include)
set (LIBWOBBLY_INTERNAL_INCLUDE_DIRS
     ${CMAKE_CURRENT_SOURCE_DIR}/src)
set (LIBWOBBLY_TEST_INCLUDE_DIRS
     ${GTEST_INCLUDE_DIR}
     ${GMOCK_INCLUDE_DIR})

# Link libraries
set (LIBWOBBLY_EXTERNAL_LIBRARIES
     ${Boost_LIBRARIES})

# Target names
set (LIBWOBBLY_LIBRARY smspillaz_wobbly)
set (LIBWOBBLY_MATCHERS_LIBRARY wobbly_matchers)
set (LIBWOBBLY_UNIT_TESTS_BINARY wobbly_unit_tests)

# Install package names
set (LIBWOBBLY_NAMESPACE smspillaz)
set (LIBWOBBLY_PACKAGE wobbly)

add_subdirectory (src)
add_subdirectory (matchers)
add_subdirectory (tests)
add_subdirectory (include)

set (LIBWOBBLY_PKGCONFIG_FILE
     ${CMAKE_CURRENT_SOURCE_DIR}/libwobbly.pc.in)
set (LIBWOBBLY_PKGCONFIG_FILE_OUTPUT
     ${CMAKE_CURRENT_BINARY_DIR}/libwobbly.pc)

configure_file (${LIBWOBBLY_PKGCONFIG_FILE}
                ${LIBWOBBLY_PKGCONFIG_FILE_OUTPUT}
                @ONLY)

install (FILES ${LIBWOBBLY_PKGCONFIG_FILE_OUTPUT}
         DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)
