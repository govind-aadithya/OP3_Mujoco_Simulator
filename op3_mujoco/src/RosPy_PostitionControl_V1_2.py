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

class mujoco_ros:
    
    def callback(self, data, joint): 
        
        self.joint_angle[self.actuator_names.index(joint)] = data.data

    def __init__(self):
        self.actuator_names = [
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
            ]  
        
        self.joint_angle  = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
                             0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]  
        
        self.joint_vel    = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
                             0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        
        self.joint_torque = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
                             0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
                
        self.publisher = rospy.Publisher('/robotis_op3/joint_states', JointState, queue_size=20)
                    
        for joint in self.actuator_names:
            topic = "/robotis_op3/"+joint+"_position/command" 
            rospy.Subscriber(topic, Float64, self.callback, (joint))     

    def publisher_fn(self, joint_feedback):  
        
        # Initialize JointState msg    
        self.joint_states = JointState()
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = self.actuator_names
        self.joint_states.position = joint_feedback
        self.joint_states.velocity = joint_feedback
        self.joint_states.effort = joint_feedback

        #print(rospy.Time.now())
        print("----------------------------------------------------------------")
        print(self.joint_states)
        
        # Publish JointState msg
        self.publisher.publish(self.joint_states)


def controller(model, data):

    global node  
    
    # Assign joint values to joints in mujoco
    for i in range(len(node.actuator_names)):
        #print(i)
        data.ctrl[i] = node.joint_angle[i]
    
    joint_feedback=[]
    # Read the joint angles
    for i in range(len(node.actuator_names)):
        joint_feedback.append(data.qpos[i])
    
    node.publisher_fn(joint_feedback)
    

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


# Start listner
rospy.init_node("MuJoCo_Listner")

#print(actuator_names)
node = mujoco_ros()


# Initialize ROS
run_mujoco_sim()
