# /animation-glib/wobbly/meson.build
#
# Build the libanimation-glib library, wobbly component.
#
# See /LICENCE.md for Copyright information.

wobbly_introspectable_sources = [
  'anchor.cpp',
  'model.cpp',
  'vector.cpp'
]
wobbly_private_sources = ['wobbly-anchor-private.h']
wobbly_headers = [
  'anchor.h',
  'model.h',
  'vector.h'
]

animation_glib_introspectable_sources += files(wobbly_introspectable_sources)
animation_glib_private_sources += files(wobbly_private_sources)
animation_glib_headers += files(wobbly_headers)

install_headers(wobbly_headers, subdir: join_paths(animation_glib_headers_subdir, 'wobbly'))
