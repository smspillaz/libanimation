# /include/CMakeLists.txt
#
# Simple install and analysis rules for the headers.
#
# See LICENCE.md for Copyright information.

set (LIBWOBBLY_NAMESPACE_PUBLIC_HEADERS_DIR
     ${CMAKE_CURRENT_SOURCE_DIR})
set (LIBWOBBLY_PACKAGE_PUBLIC_HEADERS_DIR
     ${LIBWOBBLY_NAMESPACE_PUBLIC_HEADERS_DIR}/${LIBWOBBLY_PACKAGE})
set (LIBWOBBLY_HEADERS
     ${LIBWOBBLY_PACKAGE_PUBLIC_HEADERS_DIR}/wobbly.h)

psq_add_checked_sources (wobbly_headers
                         SOURCES ${LIBWOBBLY_HEADERS}
                         CLANG_TIDY_DISABLE_CHECKS
                         misc-braces-around-statements)

set (INSTALL_DIR
     ${CMAKE_INSTALL_PREFIX}/include/${LIBWOBBLY_NAMESPACE})
install (DIRECTORY ${LIBWOBBLY_PACKAGE_PUBLIC_HEADERS_DIR}
         DESTINATION ${INSTALL_DIR})
