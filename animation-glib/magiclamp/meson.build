# /animation/magiclamp/meson.build
#
# Build the libanimation library (magiclamp animation component).
#
# See /LICENCE.md for Copyright information.

magiclamp_introspectable_sources = files([
  'magiclamp.cpp'
])

magiclamp_headers = files([
  'magiclamp.h'
])

animation_glib_introspectable_sources += magiclamp_introspectable_sources
animation_glib_headers += magiclamp_headers

install_headers(magiclamp_headers, subdir: join_paths(animation_glib_headers_subdir, 'magiclamp'))
