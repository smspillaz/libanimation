# /include/CMakeLists.txt
#
# Simple install and analysis rules for the headers.
#
# See LICENCE.md for Copyright information.

set (LIBWOBBLY_NAMESPACE_PUBLIC_HEADERS_DIR
     ${CMAKE_CURRENT_SOURCE_DIR}/${LIBWOBBLY_NAMESPACE})
set (LIBWOBBLY_PACKAGE_PUBLIC_HEADERS_DIR
     ${LIBWOBBLY_NAMESPACE_PUBLIC_HEADERS_DIR}/${LIBWOBBLY_PACKAGE})
set (LIBWOBBLY_HEADERS
     ${LIBWOBBLY_PACKAGE_PUBLIC_HEADERS_DIR}/wobbly.h)

polysquare_add_checked_sources (wobbly_headers
                                SOURCES ${LIBWOBBLY_HEADERS})

install (DIRECTORY ${LIBWOBBLY_NAMESPACE_PUBLIC_HEADERS_DIR}
         DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
