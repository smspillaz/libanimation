# /animation/transform/meson.build
#
# Toplevel meson build file for libanimation.
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
# Build the libanimation library (transform animation base component

transform_introspectable_sources = files([
  'transform.cpp'
])

transform_headers = files([
  'transform.h'
])

animation_glib_introspectable_sources += transform_introspectable_sources
animation_glib_headers += transform_headers

install_headers(transform_headers, subdir: join_paths(animation_glib_headers_subdir, 'transform'))
