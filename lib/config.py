config = {
    "robot_arm" : 
    {
        "robot_arm_type":"ur5",
        "robot_arm_ip":"10.1.12.113",
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
        "binpicking" : False
    },
    "offsets" : 
    {
        "finger_offset": 0.040,
    }
}