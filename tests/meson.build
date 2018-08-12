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
# Build the libanimation unit tests.

animation_test_sources = [
  'ostream_point_operator.h',
  'wobbly/anchor_test.cpp',
  'wobbly/constrainment_test.cpp',
  'wobbly/euler_integration_test.cpp',
  'wobbly/glib_api_test.cpp',
  'wobbly/mesh_interpolation_test.cpp',
  'wobbly/model_test.cpp',
  'wobbly/point_test.cpp',
  'wobbly/spring_test.cpp'
]

glib = dependency('glib-2.0')
gobject = dependency('gobject-2.0')

animation_test_executable = executable(
  'animation_test',
  animation_test_sources,
  dependencies: [
    gtest_dep,
    gtest_main_dep,
    gmock_dep,
    glib,
    gobject,
    mathematical_model_matcher_dep,
    within_geometry_matcher_dep,
    animation_dep,
    animation_glib_dep
  ],
  include_directories: [ tests_inc ]
)

test('animation_test', animation_test_executable)
