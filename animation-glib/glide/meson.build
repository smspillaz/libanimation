# /animation/glide/meson.build
#
# Build the libanimation library (glide animation component).
#
# See /LICENCE.md for Copyright information.

glide_introspectable_sources = files([
  'glide.cpp'
])

glide_headers = files([
  'glide.h'
])

animation_glib_introspectable_sources += glide_introspectable_sources
animation_glib_headers += glide_headers

install_headers(glide_headers, subdir: join_paths(animation_glib_headers_subdir, 'glide'))
