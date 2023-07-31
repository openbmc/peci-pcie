project(
  'peci-pcie',
  'cpp',
  version : '1.0',
  meson_version: '>=1.1.1',
  default_options: [
    'b_ndebug=if-release',
    'cpp_rtti=false',
    'cpp_std=c++20',
    'warning_level=3',
    'werror=true',
  ]
)

if(get_option('wait-for-os-standby').enabled())
  add_project_arguments('-DWAIT_FOR_OS_STANDBY', language : 'cpp')
endif

if(get_option('use-rdendpointcfg').enabled())
  add_project_arguments('-DUSE_RDENDPOINTCFG', language : 'cpp')
endif

sdbusplus = dependency('sdbusplus')

systemd = dependency('systemd', required: true)
systemd_system_unit_dir = systemd.get_variable(pkgconfig:
    'systemdsystemunitdir'
)

peci = dependency('libpeci')

boost = dependency('boost', version: '>=1.82')
add_project_arguments(
  [
    '-DBOOST_ASIO_DISABLE_THREADS',
    '-DBOOST_ASIO_EXCEPTION_DISABLE',
    '-DBOOST_ASIO_NO_DEPRECATED',
    '-DBOOST_NO_RTTI',
    '-DBOOST_NO_TYPEID',
  ],
  language : 'cpp'
)

bindir = get_option('prefix') + '/' + get_option('bindir')

executable(
  'peci-pcie',
  'src/peci_pcie.cpp',
  include_directories: include_directories('include'),
  dependencies: [
    boost,
    peci,
    sdbusplus,
  ],
  install: true,
  install_dir: bindir
)

subdir('service_files')
