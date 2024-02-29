#!/usr/bin/env python3

import mujoco as mj
import mujoco_viewer
import numpy as np
import os
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import Pose
import time
import glfw


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
                
        self.publisher_Joint = rospy.Publisher('/robotis_op3/joint_states', JointState, queue_size=1)
        self.publisher_Pose = rospy.Publisher('/robotis_op3/torso_orientation', Pose, queue_size=1)
                    
        for joint in self.actuator_names:
            topic = "/robotis_op3/"+joint+"_position/command" 
            rospy.Subscriber(topic, Float64, self.callback, (joint))     

    def publisher_fn(self, joint_pos_feedback, joint_vel_feedback, joint_torque_feedback, pose):  
        
        # Initialize JointState msg    
        self.joint_states = JointState()
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = self.actuator_names
        self.joint_states.position = joint_pos_feedback
        self.joint_states.velocity = joint_vel_feedback
        self.joint_states.effort = joint_torque_feedback

        # Publish JointState msg

        #print(self.joint_states)
        self.publisher_Joint.publish(self.joint_states)

        # Initializing Quaternion msg
        self.torso_pose = Pose()
        # Position
        self.torso_pose.position.x = pose[0]
        self.torso_pose.position.y = pose[1]
        self.torso_pose.position.z = pose[2]

        # Orienataion
        self.torso_pose.orientation.x = pose[3]
        self.torso_pose.orientation.y = pose[4]
        self.torso_pose.orientation.z = pose[5]                
        self.torso_pose.orientation.w = pose[6]

        self.publisher_Pose.publish(self.torso_pose)


def controller(model, data):

    global node, key_press_flag  
    
    data.qfrc_applied.fill(0)
    
    # Assign joint values to joints in mujoco
    for i in range(len(node.actuator_names)):
        #print(i)
        data.ctrl[i] = node.joint_angle[i]
    
    pose = data.sensordata[len(node.actuator_names):]

    joint_pos_feedback = []
    joint_vel_feedback = []
    joint_torque_feedback = []
    # Read the joint angles
    for i in range(len(node.actuator_names)):
        joint_pos_feedback.append(data.qpos[i])
        joint_vel_feedback.append(data.qvel[i])
        joint_torque_feedback.append(data.qfrc_actuator[i])

    node.publisher_fn(joint_pos_feedback, joint_vel_feedback, joint_torque_feedback, pose)

    # Reset Model
    if (glfw.get_key(viewer.window,glfw.KEY_BACKSPACE) == glfw.PRESS):
        mj.mj_resetData(model, data)

    # Apply perturbations
    # Key Map:   N = Forward Push; B = Backward Push; Z = Left Push; X = Right Push
    # Force and torque are defined for torso position in global frame; mj.mj_applyFT transforms this to generalized coordinates 
    force_x = 100.0
    force_y = 100.0
    force_z = 100.0
    
    
    if (glfw.get_key(viewer.window,glfw.KEY_N) == 1):
        key_press_flag[0] = 1  
        force = np.array([force_x, 0.0, 0.0])
        torque = np.array([0.0, 0.0, 0.0])
        if key_press_flag[0] != key_press_flag[1]:
            mj.mj_applyFT(model, data, force, torque, np.array(pose[0:3]), 1, data.qfrc_applied)

    if (glfw.get_key(viewer.window,glfw.KEY_B) == 1):
        key_press_flag[0] = 1
        force = np.array([-force_x, 0.0, 0.0])
        torque = np.array([0.0, 0.0, 0.0])
        if key_press_flag[0] != key_press_flag[1]:
            mj.mj_applyFT(model, data, force, torque, np.array(pose[0:3]), 1, data.qfrc_applied)
    
    if (glfw.get_key(viewer.window,glfw.KEY_Z) == 1):
        key_press_flag[0] = 1
        force = np.array([0.0, force_y, 0.0])
        torque = np.array([0.0, 0.0, 0.0])
        if key_press_flag[0] != key_press_flag[1]:
            mj.mj_applyFT(model, data, force, torque, np.array(pose[0:3]), 1, data.qfrc_applied)

    if (glfw.get_key(viewer.window,glfw.KEY_X) == 1):
        key_press_flag[0] = 1
        force = np.array([0.0, -force_y, 0.0])
        torque = np.array([0.0, 0.0, 0.0])
        if key_press_flag[0] != key_press_flag[1]:
            mj.mj_applyFT(model, data, force, torque, np.array(pose[0:3]), 1, data.qfrc_applied)

    if (glfw.get_key(viewer.window,glfw.KEY_N) == glfw.RELEASE and glfw.get_key(viewer.window,glfw.KEY_B) == glfw.RELEASE and glfw.get_key(viewer.window,glfw.KEY_Z) == glfw.RELEASE and glfw.get_key(viewer.window,glfw.KEY_X) == glfw.RELEASE):
        key_press_flag[0] = 0
    
    key_press_flag[1]=key_press_flag[0]

    print(time.time())
    

def run_mujoco_sim():     

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

key_press_flag = [0, 0]

## Load MuJoCo Model and data structures
current_folder = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(current_folder, "robotis_op3", "scene.xml")

model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data, title="OP3_Simulator")

#print(actuator_names)
node = mujoco_ros()


# Initialize ROS
run_mujoco_sim()

