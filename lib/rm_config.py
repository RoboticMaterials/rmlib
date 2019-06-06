rm_config = {
    'import' :
    {
        'end_effector' : True,
        'robot_arm' : True,
        'force_sensor' : False,
        'mobile_platform' : False,
        'safty_skin' : False
    },
    
    'end_effector' : 
    {
        'type' : 'smarthand',
        'finger_offset': 0.040,
        'finger_width_outer' : 0.014,
        'finger_width_inner' : 0.0, # currently affects nothing
        'finger_depth' : 0.013
    },
    'robot_arm' : 
    {
        'type' : 'ur5',
        'ip_address' : '10.1.12.113',
        
        'max_linear_speed' : 0.25,           # m/s
        'max_linear_accel' : 1.2,            # m/s^2
        'max_joint_speed' : 1.05,            # rad/s
        'max_joint_accel' : 1.4,             # rad/s^2
        
        'default_linear_speed' : 0.20,       # m/s
        'default_linear_accel' : 0.80,       # m/s^2
        'default_joint_speed' : 1.05,        # rad/s
        'default_joint_accel' : 1.4          # rad/s^2
    },
    'force_sensor' :
    {
        'type' : 'onrobot',
        'ip_address' : '10.1.12.135'
    },
    'mobile_platform' :
    {
        'type' : 'mir_100',
        'ip_address' : '10.1.12'
    },
    'safety_skin' :
    {
        'type' : 'smartskin',
        'ip_address' : '10.1.12'
    },
}