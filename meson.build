project('airburst-exe', 'c',
        version : '1.0.0',
        default_options : ['warning_level=3', 'c_std=c11'])

cc = meson.get_compiler('c')
m_dep = cc.find_library('m', required : true)

dependencies = [
        dependency('airburst'),
        dependency('imu_lib'),
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