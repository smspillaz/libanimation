# /animation/glide/meson.build
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
# Build the libanimation library (glide animation component).

glide_sources = files([
  'glide.cpp'
])

glide_headers = files([
  'glide.h'
])

animation_sources += glide_sources
animation_headers += glide_headers

install_headers(glide_headers, subdir: join_paths(animation_headers_subdir, 'glide'))
