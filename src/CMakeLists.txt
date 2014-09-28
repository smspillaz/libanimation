# /src/CMakeLists.txt
#
# Build the libwobbly library as a shared library.
#
# See LICENCE.md for Copyright information.

set (LIBWOBBLY_SOURCES
	 ${CMAKE_CURRENT_SOURCE_DIR}/emscripten.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/wobbly.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/wobbly_internal.h)

polysquare_add_library (${LIBWOBBLY_LIBRARY} SHARED
                        SOURCES ${LIBWOBBLY_SOURCES}
                        INTERNAL_INCLUDE_DIRS ${LIBWOBBLY_EXPORTED_INCLUDE_DIRS}
                        EXTERNAL_INCLUDE_DIRS ${LIBWOBBLY_EXTERNAL_INCLUDE_DIRS}
                        LIBRARIES ${LIBWOBBLY_EXTERNAL_LIBRARIES})

install (TARGETS ${LIBWOBBLY_LIBRARY}
         LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
