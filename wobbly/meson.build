# /wobbly/meson.build
#
# Build the libwobbly library.
#
# See /LICENCE.md for Copyright information.

api_version = '0.3'

wobbly_sources = [
  'wobbly_internal.h',
  'wobbly.cpp'
]

wobbly_headers = [
  'geometry.h',
  'geometry_traits.h',
  'wobbly.h'
]

wobbly_lib = shared_library(
  'windowfx_wobbly',
  wobbly_sources,
  soversion: api_version,
  install: true,
  include_directories: [ wobbly_inc ]
)

install_headers(wobbly_headers, subdir: 'wobbly')

wobbly_dep = declare_dependency(
  link_with: wobbly_lib,
  include_directories: [ wobbly_inc ],
)

pkg = import('pkgconfig')
pkg.generate(
  description: 'Library to provide a spring-controlled bezier mesh',
  name: 'libwindowfx_wobbly',
  filebase: 'libwindowfx_wobbly-' + api_version,
  version: meson.project_version(),
  libraries: wobbly_lib,
  install_dir: join_paths(get_option('libdir'), 'pkgconfig')
)
