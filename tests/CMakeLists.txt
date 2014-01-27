# /tests/CMakeLists.txt
#
# Build the libwobbly unit tests
#
# See LICENCE.md for Copyright information.

set (LIBWOBBLY_UNIT_TESTS_INCLUDE_DIRS
     ${LIBWOBBLY_EXTERNAL_INCLUDE_DIRS}
     ${LIBWOBBLY_EXPORTED_INCLUDE_DIRS})

set (LIBWOBBLY_UNIT_TESTS_LIBRARIES
     ${LIBWOBBLY_EXTERNAL_LIBRARIES}
     ${LIBWOBBLY_LIBRARY})

set (LIBWOBBLY_UNIT_TESTS_SOURCES
     ${CMAKE_CURRENT_SOURCE_DIR}/wobbly.cpp)

polysquare_add_test (${LIBWOBBLY_UNIT_TESTS_BINARY}
                     INCLUDE_DIRECTORIES ${LIBWOBBLY_UNIT_TESTS_INCLUDE_DIRS}
                     SOURCES ${LIBWOBBLY_UNIT_TESTS_SOURCES}
                     LIBRARIES ${LIBWOBBLY_UNIT_TESTS_LIBRARIES}
                     MATCHERS ${LIBWOBBLY_MATCHERS_LIBRARY})

_polysquare_add_checks_to_target (${LIBWOBBLY_UNIT_TESTS_BINARY}
                                  CPPCHECK
                                  VERAPP
                                  WARN_ONLY
                                  INTERNAL_INCLUDE_DIRS
                                  ${LIBWOBBLY_EXPORTED_INCLUDE_DIRS})
