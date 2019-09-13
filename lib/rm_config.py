import os

rm_config = {
    'rmstudio_path': os.path.join(os.path.expanduser('~'), 'rmstudio/'),

    'import':
    {
        'end_effector': True,
        'robot_arm': True,
        'force_sensor': True,
        'mobile_platform': True,
    },
    'end_effector':
    {
        'type': 'smarthand',
        'finger_length': 0.040,
        'finger_width_outer': 0.015,
        'finger_depth': 0.014,
        'camera': 'realsense_d410'
    },
    'robot_arm':
    {
        'type': 'ur5',
        'ip_address': '10.1.12.113',
        'max_linear_speed': 0.25,           # m/s
        'max_linear_accel': 1.2,            # m/s^2
        'max_joint_speed': 1.05,            # rad/s
        'max_joint_accel': 1.4,             # rad/s^2
        'default_linear_speed': 0.1,       # m/s
        'default_joint_speed': 0.7,        # rad/s
        'default_linear_accel': 0.8,
        'default_joint_accel': 0.8,
    },
    'force_sensor':
    {
        'type': 'onrobot',
        'ip_address': '10.1.12.135'
    },
    'mobile_platform':
    {
        'type': 'mir100',
        'ip_address': '10.1.12.186'
    },
}
