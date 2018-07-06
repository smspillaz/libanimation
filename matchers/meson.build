# /matchers/meson.build
#
# Build the mathematical_model_matcher static library.
#
# See /LICENCE.md for Copyright information.

api_version = '0.1'

mathematical_model_matcher_sources = [
  'mathematical_model_matcher.h',
  'mathematical_model_matcher.cpp'
]

mathematical_model_matcher_lib = static_library(
  'mathematical_model_matcher',
  mathematical_model_matcher_sources,
  dependencies: [
    gtest_project.get_variable('gtest_dep'),
    gtest_project.get_variable('gmock_dep'),
    wobbly_dep
  ],
  install: false
)

mathematical_model_matcher_dep = declare_dependency(
  link_with: mathematical_model_matcher_lib,
  include_directories: [ include_directories('.') ],
)

within_geometry_matcher_sources = [
  'within_geometry_matcher.h',
  'within_geometry_matcher.cpp'
]

within_geometry_matcher_lib = static_library(
  'within_geometry_matcher',
  within_geometry_matcher_sources,
  dependencies: [
    gtest_project.get_variable('gtest_dep'),
    gtest_project.get_variable('gmock_dep'),
    wobbly_dep
  ],
  install: false
)

within_geometry_matcher_dep = declare_dependency(
  link_with: within_geometry_matcher_lib,
  include_directories: [ include_directories('.') ],
)
