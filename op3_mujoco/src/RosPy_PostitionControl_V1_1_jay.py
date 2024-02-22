#!/usr/bin/env python3

import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import tkinter as tk
from threading import *
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64
import time


simend = 100

'''actuator_names = {
    1 : "r_sho_pitch",
    2 : "l_sho_pitch",
    3 : "r_sho_roll",
    4: "l_sho_roll",
    5: "r_el",
    6: "l_el",
    7: "r_hip_yaw",
    8: "l_hip_yaw",
    9: "r_hip_roll",
    10: "l_hip_roll",
    11: "r_hip_pitch",
    12: "l_hip_pitch",
    13: "r_knee",
    14: "l_knee",
    15: "r_ank_pitch",
    16: "l_ank_pitch",
    17: "r_ank_roll",
    18: "l_ank_roll",
    19: "head_pan",
    20: "head_tilt",
}'''

actuator_names = [
    "r_sho_pitch",
    "l_sho_pitch",
    "r_sho_roll",
    "l_sho_roll",
    "r_el",
    "l_el",
    "r_hip_yaw",
    "l_hip_yaw",
    "r_hip_roll",
    "l_hip_roll",
    "r_hip_pitch",
    "l_hip_pitch",
    "r_knee",
    "l_knee",
    "r_ank_pitch",
    "l_ank_pitch",
    "r_ank_roll",
    "l_ank_roll",
    "head_pan",
    "head_tilt"
    # "head_pan",
    # "head_tilt",
    # "l_sho_pitch",
    # "l_sho_roll",
    # "l_el",
    # "r_sho_pitch",
    # "r_sho_roll",
    # "r_el",
    # "l_hip_yaw",
    # "l_hip_roll",
    # "l_hip_pitch",
    # "l_knee",
    # "l_ank_pitch",
    # "l_ank_roll",
    # "r_hip_yaw",
    # "r_hip_roll",
    # "r_hip_pitch",
    # "r_knee",
    # "r_ank_pitch",
    # "r_ank_roll"
]

goal_joint_angle_rad = {
    "r_sho_pitch": 0.0,
    "l_sho_pitch": 0.0,
    "r_sho_roll": 0.0,
    "l_sho_roll": 0.0,
    "r_el": 0.0,
    "l_el": 0.0,
    "r_hip_yaw": 0.0,
    "l_hip_yaw": 0.0,
    "r_hip_roll": 0.0,
    "l_hip_roll": 0.0,
    "r_hip_pitch": 0.0,
    "l_hip_pitch": 0.0,
    "r_knee": 0.0,
    "l_knee": 0.0,
    "r_ank_pitch": 0.0,
    "l_ank_pitch": 0.0,
    "r_ank_roll": 0.0,
    "l_ank_roll": 0.0,
    "head_pan": 0.0,
    "head_tilt": 0.0
}

present_joint_angle_rad = {
    "r_sho_pitch": 0.0,
    "l_sho_pitch": 0.0,
    "r_sho_roll": 0.0,
    "l_sho_roll": 0.0,
    "r_el": 0.0,
    "l_el": 0.0,
    "r_hip_yaw": 0.0,
    "l_hip_yaw": 0.0,
    "r_hip_roll": 0.0,
    "l_hip_roll": 0.0,
    "r_hip_pitch": 0.0,
    "l_hip_pitch": 0.0,
    "r_knee": 0.0,
    "l_knee": 0.0,
    "r_ank_pitch": 0.0,
    "l_ank_pitch": 0.0,
    "r_ank_roll": 0.0,
    "l_ank_roll": 0.0,
    "head_pan": 0.0,
    "head_tilt": 0.0
}

present_joint_vel_rps = {
    "r_sho_pitch": 0.0,
    "l_sho_pitch": 0.0,
    "r_sho_roll": 0.0,
    "l_sho_roll": 0.0,
    "r_el": 0.0,
    "l_el": 0.0,
    "r_hip_yaw": 0.0,
    "l_hip_yaw": 0.0,
    "r_hip_roll": 0.0,
    "l_hip_roll": 0.0,
    "r_hip_pitch": 0.0,
    "l_hip_pitch": 0.0,
    "r_knee": 0.0,
    "l_knee": 0.0,
    "r_ank_pitch": 0.0,
    "l_ank_pitch": 0.0,
    "r_ank_roll": 0.0,
    "l_ank_roll": 0.0,
    "head_pan": 0.0,
    "head_tilt": 0.0
}

present_joint_effort_Nm = {
    "r_sho_pitch": 0.0,
    "l_sho_pitch": 0.0,
    "r_sho_roll": 0.0,
    "l_sho_roll": 0.0,
    "r_el": 0.0,
    "l_el": 0.0,
    "r_hip_yaw": 0.0,
    "l_hip_yaw": 0.0,
    "r_hip_roll": 0.0,
    "l_hip_roll": 0.0,
    "r_hip_pitch": 0.0,
    "l_hip_pitch": 0.0,
    "r_knee": 0.0,
    "l_knee": 0.0,
    "r_ank_pitch": 0.0,
    "l_ank_pitch": 0.0,
    "r_ank_roll": 0.0,
    "l_ank_roll": 0.0,
    "head_pan": 0.0,
    "head_tilt": 0.0
}

# class mujoco_ros:

#     def __init__(self):
#         self.publisher = rospy.Publisher('/robotis_op3/joint_states', JointState, queue_size=20)
#         self.initialize_subscribers(actuator_names)
#         self.rate = rospy.Rate(125)

#     def callback(self, data, joint): 
        
#         global goal_joint_angle_rad
#         goal_joint_angle_rad[joint] = data.data
#         #print(joint, "       ", data.data, "       ", goal_joint_angle_rad[joint])      # For test

#     def initialize_subscribers(self, actuators):
        
#         for joint in actuators:
#             topic = "/robotis_op3/"+joint+"_position/command" 
#             rospy.Subscriber(topic, Float64, self.callback, (joint))     
        

#     def publisher_fn(self, joint_feedback):  

#         # if not rospy.is_shutdown():

#         # Initialize JointState msg    
#         joint_states = JointState()
#         joint_states.header = Header()
#         joint_states.header.stamp = rospy.Time.now()
#         joint_states.name = actuator_names
#         joint_states.position = joint_feedback

#         #Publish JointState msg
#         self.publisher.publish(joint_states)

publisher = []


def publishJointStates(publisher, joint_feedback):  
    # if not rospy.is_shutdown():

    # Initialize JointState msg    
    joint_states = JointState()
    joint_states.header = Header()
    joint_states.header.stamp = rospy.Time.now()
    joint_states.name = actuator_names
    joint_states.position = joint_feedback

    #Publish JointState msg
    publisher.publish(joint_states)


def callback(data, joint): 
    
    global goal_joint_angle_rad
    goal_joint_angle_rad[joint] = data.data

def initializeSubscribers(actuators):
        
    for joint in actuators:
        topic = "/robotis_op3/"+joint+"_position/command" 
        rospy.Subscriber(topic, Float64, callback, (joint))     

def controller(model, data):

    global actuator_names
    global goal_joint_angle_rad
    
    for key in goal_joint_angle_rad:
        data.ctrl[actuator_names.index(key)] = goal_joint_angle_rad[key]

    # global joint_angles
    # joint_angles = []

    # # Assign joint values to joints in mujoco
    # if len(joint_angles) == len(actuator_names):
    #     for i in range(len(actuator_names)):
    #         #model.actuator_gainprm[i,0] = kp
    #         #model.actuator_biasprm[i,1] = -kp
    #         #print(i)
    #         data.ctrl[i] = joint_angles[i]
    
    '''
    joint_feedback=[]
    # Read the joint angles
    for i in range(len(actuator_names)):
        joint_feedback.append(data.qpos[i])
    
    publisher_fn(joint_feedback)
    '''

    for key in present_joint_angle_rad:
        present_joint_angle_rad[key] = data.qpos[actuator_names.index(key)]
        present_joint_vel_rps[key] = data.qvel[actuator_names.index(key)]
        present_joint_effort_Nm[key] = data.qacc[actuator_names.index(key)] # not effot Govind plz check

    joint_states = JointState()
    joint_states.header = Header()
    joint_states.header.stamp = rospy.Time.now() # why is it always zero?
    joint_states.name = list(present_joint_angle_rad.keys())
    joint_states.position = list(present_joint_angle_rad.values())
    joint_states.velocity = list(present_joint_vel_rps.values())
    joint_states.effort = list(present_joint_effort_Nm.values())
   
    #Publish JointState msg
    publisher.publish(joint_states)

def run_mujoco_sim():

    #get the full XML path
    current_folder = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(current_folder, "robotis_op3", "scene.xml")

    # MuJoCo data structures
    model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
    data = mj.MjData(model)                     # MuJoCo data
    cam = mj.MjvCamera()                        # Abstract camera
    opt = mj.MjvOption()                        # visualization options

    # Init GLFW, create window, make OpenGL context current, request v-sync
    glfw.init()
    window = glfw.create_window(1200, 900, "Demo", None, None)
    glfw.make_context_current(window)
    glfw.swap_interval(1)

    # initialize visualization data structures
    mj.mjv_defaultCamera(cam)
    mj.mjv_defaultOption(opt)
    scene = mj.MjvScene(model, maxgeom=10000)
    context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

    # Set the controller
    mj.set_mjcb_control(controller)

    while not glfw.window_should_close(window): # and not rospy.is_shutdown():
        simstart = data.time
        
        while (data.time - simstart < 1.0/60.0):
            mj.mj_step(model, data)

        if (data.time>=simend):
            #print("Exit Sim!")
            break
            
        # get framebuffer viewport
        viewport_width, viewport_height = glfw.get_framebuffer_size(
            window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

        # Update scene and render
        mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
        mj.mjr_render(viewport, scene, context)

        # swap OpenGL buffers (blocking call due to v-sync)
        glfw.swap_buffers(window)

        # process pending GUI events, call GLFW callbacks
        glfw.poll_events()


    glfw.terminate() 
'''
def threading():
    t1=Thread(target=run_mujoco_sim) 
    t1.start() 
    global break_out
    break_out = False
'''



# Start listner
rospy.init_node("MuJoCo_Listner")

publisher = rospy.Publisher('/robotis_op3/joint_states', JointState, queue_size=20)
initializeSubscribers(actuator_names)


# Initialize ROS
run_mujoco_sim()
