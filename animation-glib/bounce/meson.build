# /animation/bounce/meson.build
#
# Build the libanimation library (bounce animation component).
#
# See /LICENCE.md for Copyright information.

bounce_introspectable_sources = [
  'bounce.cpp'
]

bounce_headers = [
  'bounce.h'
]

animation_glib_introspectable_sources += files(bounce_introspectable_sources)
animation_glib_headers += files(bounce_headers)

install_headers(bounce_headers, subdir: join_paths(animation_headers_subdir, 'bounce'))
