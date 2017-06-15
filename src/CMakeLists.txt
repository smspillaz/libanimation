# /src/CMakeLists.txt
#
# Build the libwobbly library as a shared library.
#
# See LICENCE.md for Copyright information.

set (LIBWOBBLY_SOURCES
     ${CMAKE_CURRENT_SOURCE_DIR}/wobbly.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/wobbly_internal.h)

if (WOBBLY_EMSCRIPTEN)

    list (APPEND LIBWOBBLY_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/emscripten.cpp)

endif (WOBBLY_EMSCRIPTEN)

psq_add_library (${LIBWOBBLY_LIBRARY} SHARED
                 SOURCES ${LIBWOBBLY_SOURCES}
                 INTERNAL_INCLUDE_DIRS
                 ${LIBWOBBLY_EXPORTED_INCLUDE_DIRS}
                 ${CMAKE_CURRENT_SOURCE_DIR}
                 EXTERNAL_INCLUDE_DIRS ${LIBWOBBLY_EXTERNAL_INCLUDE_DIRS}
                 LIBRARIES ${LIBWOBBLY_EXTERNAL_LIBRARIES}
                 CLANG_TIDY_DISABLE_CHECKS
                 # bug in clang-tidy version in travis-ci
                 misc-braces-around-statements)

psq_add_checked_sources (check_boost_geometry_prefix_header
                         SOURCES
                         ${CMAKE_CURRENT_SOURCE_DIR}/boost_geometry.h
                         INTERNAL_INCLUDE_DIRS
                         ${LIBWOBBLY_EXPORTED_INCLUDE_DIRS}
                         EXTERNAL_INCLUDE_DIRS
                         ${LIBWOBBLY_EXTERNAL_INCLUDE_DIRS}
                         NO_IWYU
                         CLANG_TIDY_DISABLE_CHECKS
                         # Disable unused-local-typedef as it trips up on
                         # BOOST_CONCEPT_ASSERT
                         clang-diagnostic-unused-local-typedef
                         misc-braces-around-statements)

set_target_properties (${LIBWOBBLY_LIBRARY} PROPERTIES
                       VERSION ${LIBWOBBLY_VERSION}
                       SOVERSION ${LIBWOBBLY_VERSION_MAJOR})

install (TARGETS ${LIBWOBBLY_LIBRARY}
         LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
