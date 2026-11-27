project('airburst-exe', 'c',
        version : '1.0.0',
        default_options : ['warning_level=3', 'c_std=c11'])

cc = meson.get_compiler('c')
m_dep = cc.find_library('m', required : true)

dependencies = [
        dependency('airburst'),
        dependency('imu_lib'),
#        dependency('esp32cam_lib'),
        m_dep,
]

airburst_exe = executable(
        'airburst_exe',
        [
                'src/imu_test.c',
                'src/functions.c',
        ],
        dependencies : dependencies,
        install : true
)

test('test', airburst_exe)

# A111 Envelope object detection at ~0.30 m, with a UDP feed for tools/envelope_view.py
envelope_exe = executable(
        'envelope_exe',
        [
                'src/service_envelope.c',
        ],
        dependencies : [dependency('airburst'), m_dep],
        install : true
)
# Motion trigger (presence detector, Wave-to-exit based) with distance, LED and TCP/UDP link to tools/trigger_view.py
trigger_exe = executable(
        'trigger_exe',
        [
                'src/motion_trigger.c',
        ],
        dependencies : [dependency('airburst'), m_dep],
        install : true
)

# A111 distance detector 200-600 mm, full envelope per frame over UDP to tools/distance_view.py
distance_exe = executable(
        'distance_exe',
        [
                'src/distance_frames.c',
        ],
        dependencies : [dependency('airburst'), m_dep],
        install : true
)
