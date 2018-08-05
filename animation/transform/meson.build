# /animation/wobbly/meson.build
#
# Build the libanimation library (transform animation base class component).
#
# See /LICENCE.md for Copyright information.

transform_headers = files([
  'transform.h'
])

animation_headers += transform_headers

install_headers(transform_headers, subdir: join_paths(animation_headers_subdir, 'transform'))
