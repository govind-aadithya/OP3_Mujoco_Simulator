#!/usr/bin/env python3

import mujoco as mj
import mujoco_viewer
import numpy as np
import os
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64
import time


simend = 100

class mujoco_ros:
    
    def callback(self, data, joint): 
        
        self.joint_angle[self.actuator_names.index(joint)] = data.data
        #print("Recieving Data!!")

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

    def publisher_fn(self, joint_pos_feedback, joint_vel_feedback, joint_torque_feedback):  
        
        # Initialize JointState msg    
        self.joint_states = JointState()
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = self.actuator_names
        self.joint_states.position = joint_pos_feedback
        self.joint_states.velocity = joint_vel_feedback
        self.joint_states.effort = joint_torque_feedback

        # Publish JointState msg
        #print("-------------------------------------------------")
        #print(self.joint_states)
        self.publisher.publish(self.joint_states)

def controller(model, data):

    global node  
    
    # Assign joint values to joints in mujoco
    for i in range(len(node.actuator_names)):
        #print(i)
        data.ctrl[i] = node.joint_angle[i]
    
    print("Quat data: ", data.sensordata[-4:])
    print("______________________________________")

    joint_pos_feedback = []
    joint_vel_feedback = []
    joint_torque_feedback = []
    # Read the joint angles
    for i in range(len(node.actuator_names)):
        joint_pos_feedback.append(data.qpos[i])
        joint_vel_feedback.append(data.qvel[i])
        joint_torque_feedback.append(data.qfrc_actuator[i])
    
    node.publisher_fn(joint_pos_feedback, joint_vel_feedback, joint_torque_feedback)


def run_mujoco_sim():     
    current_folder = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(current_folder, "robotis_op3", "scene.xml")

    # MuJoCo data structures
    model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
    data = mj.MjData(model)

    # create the viewer object
    viewer = mujoco_viewer.MujocoViewer(model, data, title="OP3_Simulator")
    
    # configure viewer
    viewer._render_every_frame = False
    # viewer._contacts = True                 # Visualize contact forces default


    #set the controller
    mj.set_mjcb_control(controller)

    # simulate and render
    for _ in range(100000):
        if viewer.is_alive:
            mj.mj_step(model, data)
            viewer.render()
        else:
            break

    # close
    viewer.close()

# Start listner
rospy.init_node("MuJoCo_Listner")

#print(actuator_names)
node = mujoco_ros()


# Initialize ROS
run_mujoco_sim()

