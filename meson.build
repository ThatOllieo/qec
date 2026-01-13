project('uk', 'cpp',
  version: '0.1',
  default_options: ['warning_level=3', 'cpp_std=c++17'])

main_src = files('src/main.cpp')
depl_src = files('src/deployment_watcher.cpp')
case_src = files('src/camera_service.cpp')
imu_src = files('src/imu.cpp')
comm_src = files('src/comms_manager.cpp')
udp_channel = files('include/channels/udp_channel.cpp')
rad_channel = files('include/channels/radio_channel.cpp')

libcamera_dep = dependency('libcamera', required: true)
jpeg = dependency('libjpeg', required: true)
gpiod_dep = dependency('libgpiod', required: true)

executable('uk',
  [main_src,depl_src,case_src,imu_src,comm_src,udp_channel,rad_channel],
  include_directories: include_directories('src'),
  dependencies: [libcamera_dep, jpeg, gpiod_dep]
)