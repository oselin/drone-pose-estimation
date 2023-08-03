#!/usr/bin/env python3


def POSE_TOPIC_TEMPLATE(id): 
    return f"/drone{id}/mavros/local_position/pose"

def VEL_TOPIC_TEMPLATE(id):  
    return f"/drone{id}/mavros/setpoint_velocity/cmd_vel"

def DISTANCE_TOPIC_TEMPLATE(id): 
    return f"/drone{id}/mavros/distances"

def POSITION_TOPIC_TEMPLATE(id): 
    return f'/drone{id}/mavros/setpoint_position/local'

def VELOCITY_TOPIC_TEMPLATE(id): 
    return f'/drone{id}/mavros/setpoint_velocity/cmd_vel'

def SETMODE_TOPIC_TEMPLATE(id):  
    return f'/drone{id}/mavros/set_mode'

def ARMING_TOPIC_TEMPLATE(id):   
    return f'/drone{id}/mavros/cmd/arming'

def TAKEOFF_TOPIC_TEMPLATE(id):  
    return f'/drone{id}/mavros/cmd/takeoff'

def LAND_TOPIC_TEMPLATE(id):     
    return f'/drone{id}/mavros/cmd/land'