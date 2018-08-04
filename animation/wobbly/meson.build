# /animation/wobbly/meson.build
#
# Build the libanimation library (wobbly windows component).
#
# See /LICENCE.md for Copyright information.

wobbly_sources = [
  'wobbly_internal.h',
  'wobbly.cpp'
]

wobbly_headers = [
  'geometry.h',
  'geometry_traits.h',
  'wobbly.h'
]

animation_sources += files(wobbly_sources)
animation_headers += files(wobbly_headers)

install_headers(wobbly_headers, subdir: join_paths(animation_headers_subdir, 'wobbly'))
