# /meson.build
#
# Toplevel meson build file for libwobbly.
#
# See /LICENCE.md for Copyright information.

project('libwobbly', 'cpp',
        version: '0.3.2',
        default_options : ['cpp_std=c++17'],
        license: 'LGPL2+',
        meson_version: '>= 0.40.0')

gtest_dep = dependency('gtest', required: false)
gtest_main_dep = dependency('gtest', main: true, required: false)
gmock_dep = dependency('gmock', required: false)

if not gtest_dep.found() or not gtest_main_dep.found() or not gmock_dep.found()
  gtest_project = subproject('googletest')
  gtest_dep = gtest_project.get_variable('gtest_dep')
  gtest_main_dep = gtest_project.get_variable('gtest_main_dep')
  gmock_dep = gtest_project.get_variable('gmock_dep')
endif

wobbly_inc = include_directories('.')

subdir('wobbly')
subdir('wobbly-glib')
subdir('matchers')
subdir('tests')
