# /matchers/meson.build
#
# Copyright (C) 2017, 2018 Endless Mobile, Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
# Build the mathematical_model_matcher static library.

api_version = '0.1'

mathematical_model_matcher_sources = [
  'mathematical_model_matcher.h',
  'mathematical_model_matcher.cpp'
]

mathematical_model_matcher_lib = static_library(
  'mathematical_model_matcher',
  mathematical_model_matcher_sources,
  dependencies: [
    gtest_dep,
    gmock_dep,
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
    gtest_dep,
    gmock_dep,
    wobbly_dep
  ],
  install: false
)

within_geometry_matcher_dep = declare_dependency(
  link_with: within_geometry_matcher_lib,
  include_directories: [ include_directories('.') ],
)
