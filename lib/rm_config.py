rm_config = {
    "robot_arm" : 
    {
        "robot_arm_type":"ur5",
        "robot_arm_ip":"10.1.12.113",
        
        "max_linear_speed": 0.25,           # m/s
        "max_linear_accel": 1.2,            # m/s^2
        "max_joint_speed": 1.05,            # rad/s
        "max_joint_accel": 1.4,             # rad/s^2
        
        "default_linear_speed": 0.25,       # m/s
        "default_linear_accel": 1.2,        # m/s^2
        "default_joint_speed": 1.05,        # rad/s
        "default_joint_accel": 1.4          # rad/s^2
    },
    "force_sensor" :
    {
        "force_sensor_ip":"10.1.12.135"
    },
    "imports" :
    {
        "realsense" : True,
        "motor_control" : True,
        "ur5" : True,
        "optoforce" : False,
        "assembly" : False,
        "binpicking" : False
    },
    "offsets" : 
    {
        "finger_offset": 0.040,
    }
}