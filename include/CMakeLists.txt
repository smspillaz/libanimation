# /include/CMakeLists.txt
#
# Simple install and analysis rules for the headers.
#
# See LICENCE.md for Copyright information.

set (LIBWOBBLY_HEADERS
     ${CMAKE_CURRENT_SOURCE_DIR}/wobbly.h)

polysquare_add_checked_sources (wobbly_headers
                                SOURCES ${LIBWOBBLY_HEADERS}
                                DESCRIPTION "wobbly headers")
