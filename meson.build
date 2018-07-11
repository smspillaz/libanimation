# /meson.build
#
# Toplevel meson build file for libwobbly.
#
# See /LICENCE.md for Copyright information.

project('libwobbly', 'cpp',
        version: '0.2.0',
        default_options : ['cpp_std=c++17'],
        license: 'LGPL2+',
        meson_version: '>= 0.40.0')

gtest_project = subproject('googletest')

wobbly_inc = include_directories('.')

subdir('wobbly')
subdir('wobbly-glib')
subdir('matchers')
subdir('tests')
