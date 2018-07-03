# /matchers/CMakeLists.txt
#
# Build the libwobbly matchers
#
# See LICENCE.md for Copyright information.

set (LIBWOBBLY_MATCHERS_LIBRARIES
     ${LIBWOBBLY_EXTERNAL_LIBRARIES})

set (LIBWOBBLY_MATCHERS_EXPORT_HEADERS
     ${CMAKE_CURRENT_SOURCE_DIR}/mathematical_model_matcher.h)
set (LIBWOBBLY_MATCHERS_SOURCES
     ${CMAKE_CURRENT_SOURCE_DIR}/mathematical_model_matcher.cpp
     ${LIBWOBBLY_MATCHERS_EXPORT_HEADERS})

psq_add_matcher (${LIBWOBBLY_MATCHERS_LIBRARY}
                 EXPORT_HEADER_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                 INTERNAL_INCLUDE_DIRS
                 ${LIBWOBBLY_EXPORTED_INCLUDE_DIRS}
                 ${LIBWOBBLY_INTERNAL_INCLUDE_DIRS}
                 EXTERNAL_INCLUDE_DIRS ${LIBWOBBLY_EXTERNAL_INCLUDE_DIRS}
                 SOURCES ${LIBWOBBLY_MATCHERS_SOURCES}
                 LIBRARIES ${LIBWOBBLY_MATCHERS_LIBRARIES}
                 CLANG_TIDY_DISABLE_CHECKS
                 misc-braces-around-statements)
