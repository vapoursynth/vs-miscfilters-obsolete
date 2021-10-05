project('Miscfilters', 'cpp',
  default_options: ['buildtype=release', 'warning_level=2', 'b_ndebug=if-release', 'cpp_std=c++14'],
  meson_version : '>=0.48.0',
  version : '1'
)

sources = [
  'src/miscfilters.cpp'
]

dep = dependency('vapoursynth', version: '>=55').partial_dependency(compile_args : true, includes : true)

shared_module('miscfilters', sources,
  dependencies : dep,
  install : true,
  install_dir : join_paths(dep.get_pkgconfig_variable('libdir'), 'vapoursynth'),
  gnu_symbol_visibility : 'hidden'
)
