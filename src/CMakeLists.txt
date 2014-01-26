# /src/CMakeLists.txt
#
# Build the libwobbly library as a shared library.
#
# See LICENCE.md for Copyright information.

include_directories (${LIBWOBBLY_EXTERNAL_INCLUDE_DIRS}
                     ${LIBWOBBLY_EXPORTED_INCLUDE_DIRS})

set (LIBWOBBLY_SOURCES
     ${CMAKE_CURRENT_SOURCE_DIR}/wobbly.cpp)

add_library (${LIBWOBBLY_LIBRARY} SHARED
             ${LIBWOBBLY_SOURCES})

target_link_libraries (${LIBWOBBLY_LIBRARY}
                       ${LIBWOBBLY_EXTERNAL_LIBRARIES})

_polysquare_add_checks_to_target (${LIBWOBBLY_LIBRARY}
                                  CPPCHECK
                                  VERAPP
                                  WARN_ONLY
                                  INTERNAL_INCLUDE_DIRS
                                  ${LIBWOBBLY_EXPORTED_INCLUDE_DIRS})
