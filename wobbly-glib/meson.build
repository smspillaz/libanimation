# /wobbly-glib/meson.build
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
# Build the libwobbly library.

api_version = '0.3'

wobbly_introspectable_sources = [
  'anchor.cpp',
  'model.cpp',
  'vector.cpp'
]

wobbly_sources = wobbly_introspectable_sources + [
  'wobbly-anchor-private.h'
]

wobbly_headers = [
  'anchor.h',
  'model.h',
  'vector.h'
]

glib = dependency('glib-2.0')
gobject = dependency('gobject-2.0')

wobbly_glib_lib = shared_library(
  'windowfx_wobbly_glib',
  wobbly_sources,
  soversion: api_version,
  install: true,
  include_directories: [ wobbly_inc ],
  dependencies: [ glib, gobject, wobbly_dep ]
)

install_headers(wobbly_headers, subdir: 'wobbly-glib')

wobbly_glib_dep = declare_dependency(
  link_with: wobbly_glib_lib,
  include_directories: [ wobbly_inc ],
)

introspection_sources = [ wobbly_introspectable_sources, wobbly_headers ]

gnome = import('gnome')
gnome.generate_gir(
  wobbly_glib_lib,
  extra_args: ['--warn-all', '--warn-error'],
  identifier_prefix: 'Wobbly',
  include_directories: wobbly_inc,
  includes: ['GLib-2.0', 'GObject-2.0'],
  install: true,
  namespace: 'Wobbly',
  nsversion: api_version,
  sources: introspection_sources,
  symbol_prefix: 'wobbly'
)

pkg = import('pkgconfig')
pkg.generate(
  description: 'Library to provide a spring-controlled bezier mesh (GObject Binding)',
  name: 'libwindowfx_wobbly_glib',
  filebase: 'libwindowfx_wobbly_glib-' + api_version,
  version: meson.project_version(),
  libraries: wobbly_glib_lib,
  install_dir: join_paths(get_option('libdir'), 'pkgconfig')
)
