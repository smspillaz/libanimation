# /animation/zoom/meson.build
#
# Build the libanimation library (zoom animation component).
#
# See /LICENCE.md for Copyright information.

zoom_introspectable_sources = files([
  'zoom.cpp'
])

zoom_headers = files([
  'zoom.h'
])

animation_glib_introspectable_sources += zoom_introspectable_sources
animation_glib_headers += zoom_headers

install_headers(zoom_headers, subdir: join_paths(animation_glib_headers_subdir, 'zoom'))
