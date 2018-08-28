# /animation/meson.build
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
# Build the libanimation library.

api_version = '0'

animation_sources = []
animation_headers = []
animation_headers_subdir = 'animation'

subdir('wobbly')

animation_lib = shared_library(
  'animation',
  animation_sources,
  soversion: api_version,
  install: true,
  include_directories: [ animation_inc ]
)

animation_dep = declare_dependency(
  link_with: animation_lib,
  include_directories: [ animation_inc ],
)

pkg = import('pkgconfig')
pkg.generate(
  description: 'Library to provide 2D surface animations',
  name: 'libanimation',
  filebase: 'libanimation-' + api_version,
  version: meson.project_version(),
  libraries: animation_lib,
  install_dir: join_paths(get_option('libdir'), 'pkgconfig')
)
