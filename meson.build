project('ukgs', 'cpp',
  version: '0.1',
  default_options: ['warning_level=3', 'cpp_std=c++17'])

main_src = files('src/main.cpp')
comm_src = files('src/comms_manager.cpp')
udp_channel = files('include/channels/udp_channel.cpp')
rad_channel = files('include/channels/radio_channel.cpp')
ws_link = files('src/ws_link.cpp')

gpiod_dep = dependency('libgpiod', required: true)
boost_dep = dependency('boost', modules : ['system'], required: true)
json_dep = dependency('nlohmann_json', required: true)

executable('ukgs',
  [main_src,comm_src,udp_channel,rad_channel,ws_link],
  include_directories: include_directories('src'),
  dependencies: [gpiod_dep,boost_dep]
)