# /src/CMakeLists.txt
#
# Build the libwobbly library as a shared library.
#
# See LICENCE.md for Copyright information.

set (LIBWOBBLY_SOURCES
     ${CMAKE_CURRENT_SOURCE_DIR}/wobbly.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/wobbly_internal.h)

if (EMSCRIPTEN)

    list (APPEND LIBWOBBLY_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/emscripten.cpp)

endif (EMSCRIPTEN)

polysquare_add_library (${LIBWOBBLY_LIBRARY} SHARED
                        SOURCES ${LIBWOBBLY_SOURCES}
                        INTERNAL_INCLUDE_DIRS ${LIBWOBBLY_EXPORTED_INCLUDE_DIRS}
                        EXTERNAL_INCLUDE_DIRS ${LIBWOBBLY_EXTERNAL_INCLUDE_DIRS}
                        LIBRARIES ${LIBWOBBLY_EXTERNAL_LIBRARIES}
                        CLANG_TIDY_DISABLE_CHECKS
                        # Disable unused-local-typedef as it trips up on
                        # BOOST_CONCEPT_ASSERT
                        clang-diagnostic-unused-local-typedef)

install (TARGETS ${LIBWOBBLY_LIBRARY}
         LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
