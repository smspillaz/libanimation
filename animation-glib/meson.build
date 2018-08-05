# /animation-glib/meson.build
#
# Build the libanimation-glib library.
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

api_version = '0'

animation_glib_toplevel_headers = [
  'box.h',
  'vector.h'
]
animation_glib_toplevel_private_headers = [
  'constructor-helpers.h',
]
animation_glib_toplevel_introspectable_sources = [
  'box.cpp',
  'vector.cpp'
]
animation_glib_toplevel_private_sources = [
  'constructor-helpers.cpp'
]

animation_glib_introspectable_sources = []
animation_glib_private_sources = []
animation_glib_headers = []
animation_glib_private_headers = []
animation_glib_headers_subdir = 'animation-glib'

subdir('transform')
subdir('wobbly')
subdir('zoom')

animation_glib_introspectable_sources += files(animation_glib_toplevel_introspectable_sources)
animation_glib_private_sources += files(animation_glib_toplevel_private_sources)
animation_glib_headers += files(animation_glib_toplevel_headers)
animation_glib_private_headers += files(animation_glib_toplevel_private_headers)

install_headers(animation_glib_toplevel_headers, subdir: animation_glib_headers_subdir)

animation_glib_sources = animation_glib_introspectable_sources + animation_glib_private_sources

glib = dependency('glib-2.0')
gobject = dependency('gobject-2.0')

animation_glib_lib = shared_library(
  'animation-glib',
  animation_glib_sources,
  soversion: api_version,
  install: true,
  include_directories: [ animation_inc ],
  dependencies: [ glib, gobject, animation_dep ]
)

animation_glib_dep = declare_dependency(
  link_with: animation_glib_lib,
  include_directories: [ animation_inc ],
)

introspection_sources = [ animation_glib_introspectable_sources, animation_glib_headers ]

gnome = import('gnome')
gnome.generate_gir(
  animation_glib_lib,
  extra_args: ['--warn-all', '--warn-error'],
  identifier_prefix: 'Animation',
  include_directories: animation_inc,
  includes: ['GLib-2.0', 'GObject-2.0'],
  install: true,
  namespace: 'Animation',
  nsversion: api_version,
  sources: introspection_sources,
  symbol_prefix: 'animation'
)

pkg = import('pkgconfig')
pkg.generate(
  description: 'Library to provide 2D surface animations (GObject Binding)',
  name: 'libanimation-glib',
  filebase: 'libanimation-glib-' + api_version,
  version: meson.project_version(),
  libraries: animation_glib_lib,
  install_dir: join_paths(get_option('libdir'), 'pkgconfig')
)
