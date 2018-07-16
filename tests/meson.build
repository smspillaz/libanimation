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
# Build the libwobbly unit tests.

wobbly_test_sources = [
  'anchor_test.cpp',
  'constrainment_test.cpp',
  'euler_integration_test.cpp',
  'glib_api_test.cpp',
  'mesh_interpolation_test.cpp',
  'model_test.cpp',
  'ostream_point_operator.h',
  'point_test.cpp',
  'spring_test.cpp'
]

glib = dependency('glib-2.0')
gobject = dependency('gobject-2.0')

wobbly_test_executable = executable(
  'windowfx_wobbly_test',
  wobbly_test_sources,
  dependencies: [
    gtest_dep,
    gtest_main_dep,
    gmock_dep,
    glib,
    gobject,
    mathematical_model_matcher_dep,
    within_geometry_matcher_dep,
    wobbly_dep,
    wobbly_glib_dep
  ]
)

test('windowfx_wobbly_test', wobbly_test_executable)
