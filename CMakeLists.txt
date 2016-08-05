# /CMakeLists.txt
# Main CMake file for libwobbly. Creates targets for the
# library and C interface and builds tests as well.
#
# See LICENCE.md for Copyright information.

project (libwobbly)
cmake_minimum_required (VERSION 2.8 FATAL_ERROR)

include (CheckCXXCompilerFlag)

# Set up CMAKE_MODULE_PATH to import Google Test and Google Mock
set (LIBWOBBLY_CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

set (CMAKE_MODULE_PATH
     ${LIBWOBBLY_CMAKE_MODULE_PATH}
     ${CMAKE_MODULE_PATH})

find_package (Boost REQUIRED)

# Use emscripten
option (WOBBLY_EMSCRIPTEN "Use emscripten" OFF)

# Single flag to enable or disable analysis checks
option (WOBBLY_ENABLE_ANALYSIS "Enable build-time analysis" ON)

if (WOBBLY_ENABLE_ANALYSIS)

    # Analysis checks not compatible with PCH
    set (POLYSQUARE_USE_PRECOMPILED_HEADERS OFF CACHE BOOL "" FORCE)

else (WOBBLY_ENABLE_ANALYSIS)

    set (POLYSQUARE_USE_VERAPP OFF CACHE BOOL "" FORCE)
    set (POLYSQUARE_USE_CLANG_TIDY OFF CACHE BOOL "" FORCE)
    set (POLYSQUARE_USE_CPPCHECK OFF CACHE BOOL "" FORCE)
    set (POLYSQUARE_USE_IWYU OFF CACHE BOOL "" FORCE)

endif (WOBBLY_ENABLE_ANALYSIS)

option (WOBBLY_UPDATE_SUBMODULES "Update submodules on configure" OFF)

if (WOBBLY_UPDATE_SUBMODULES)

    find_package (Git REQUIRED)
    execute_process (COMMAND
                     "${GIT_EXECUTABLE} submodule update --init --recursive"
                     WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

endif ()

# Don't include PolysquareCommon until we've at least updated submodules
include ("cmake/common/PolysquareCommon")

psq_compiler_bootstrap ()
psq_sanitizers_bootstrap ()
psq_rules_bootstrap (${LIBWOBBLY_POLYSQUARE_CMAKE_MODULE_PATH} ${CMAKE_BINARY_DIR})
psq_gmock_bootstrap (${LIBWOBBLY_POLYSQUARE_CMAKE_MODULE_PATH})
psq_coverage_bootstrap (${LIBWOBBLY_POLYSQUARE_CMAKE_MODULE_PATH})
psq_acceleration_bootstrap (${LIBWOBBLY_POLYSQUARE_CMAKE_MODULE_PATH})

check_cxx_compiler_flag ("-Wno-unused-local-typedefs"
                         WOBBLY_HAVE_NO_UNUSED_LOCAL_TYPEDEFS)

if (WOBBLY_HAVE_NO_UNUSED_LOCAL_TYPEDEFS)

    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-local-typedefs")

endif (WOBBLY_HAVE_NO_UNUSED_LOCAL_TYPEDEFS)

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

# Install package names
set (LIBWOBBLY_NAMESPACE windowfx
     CACHE STRING "Namespace owner of library")
set (LIBWOBBLY_PACKAGE wobbly)

# Target names
set (LIBWOBBLY_LIBRARY ${LIBWOBBLY_NAMESPACE}_wobbly)
set (LIBWOBBLY_MATCHERS_LIBRARY ${LIBWOBBLY_NAMESPACE}_wobbly_matchers)
set (LIBWOBBLY_UNIT_TESTS_BINARY
     ${LIBWOBBLY_NAMESPACE}_wobbly_unit_tests)

add_subdirectory (src)
add_subdirectory (matchers)
add_subdirectory (tests)
add_subdirectory (include)

set (LIBWOBBLY_PKGCONFIG_FILE
     ${CMAKE_CURRENT_SOURCE_DIR}/libwobbly.pc.in)
set (LIBWOBBLY_PKGCONFIG_FILE_OUTPUT
     ${CMAKE_CURRENT_BINARY_DIR}/lib${LIBWOBBLY_NAMESPACE}_wobbly.pc)

configure_file (${LIBWOBBLY_PKGCONFIG_FILE}
                ${LIBWOBBLY_PKGCONFIG_FILE_OUTPUT}
                @ONLY)

install (FILES ${LIBWOBBLY_PKGCONFIG_FILE_OUTPUT}
         DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)
