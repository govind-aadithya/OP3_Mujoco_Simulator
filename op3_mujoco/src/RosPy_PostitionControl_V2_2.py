#!/usr/bin/env python3

import mujoco as mj
import mujoco_viewer
import numpy as np
import os
import rospy
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header, Float64
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
        self.publisher_Imu = rospy.Publisher('/robotis_op3/Sim_IMU', Imu, queue_size=1)
                    
        for joint in self.actuator_names:
            topic = "/robotis_op3/"+joint+"_position/command" 
            rospy.Subscriber(topic, Float64, self.callback, (joint))     

    def publisher_fn(self, joint_pos_feedback, joint_vel_feedback, joint_torque_feedback, imu_msg_data):  
        
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


        # Initialize IMU msg
        self.imu_msg = Imu()

        # Angular Vel
        self.imu_msg.angular_velocity.x, self.imu_msg.angular_velocity.y, self.imu_msg.angular_velocity.z = imu_msg_data[0]

        # Linear Acceleration
        self.imu_msg.linear_acceleration.x, self.imu_msg.linear_acceleration.y, self.imu_msg.linear_acceleration.z = imu_msg_data[1]
        
        # Quat msg
        self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z, self.imu_msg.orientation.w = imu_msg_data[2]

        # Publish IMU Msg
        self.publisher_Imu.publish(self.imu_msg)


def controller(model, data):

    global node, key_press_flag 
    
    # Reset applied force to 0 at each iteration
    data.qfrc_applied.fill(0)
    
    # Assign joint values to joints in mujoco
    for i in range(len(node.actuator_names)):
        #print(i)
        data.ctrl[i] = node.joint_angle[i]
    
    
    # Extract sensor values required for IMU   
    imu_msg_data =[]        # order = Gyro(3x1), Accel(3x1), Orientation(4x1)
    for i in range(3):      # extract Pos, Gyro, Accel 
        if i == 0:
            pose = data.sensordata[int(len(node.actuator_names)+(3*i)):int(len(node.actuator_names)+(3*(i+1)))] 
        else:   
            imu_msg_data.append(data.sensordata[int(len(node.actuator_names)+(3*i)):int(len(node.actuator_names)+(3*(i+1)))])
    imu_msg_data.append(data.sensordata[-4:])

    '''print("Pos       : ", pose)
    print("Imu Gyro  : ", imu_msg_data[0])
    print("Imu Accel : ", imu_msg_data[1])
    print("Imu Quat  : ", imu_msg_data[2])
    print("*********************************************************")
    '''

    joint_pos_feedback = []
    joint_vel_feedback = []
    joint_torque_feedback = []
    # Read the joint angles
    for i in range(len(node.actuator_names)):
        joint_pos_feedback.append(data.qpos[i])
        joint_vel_feedback.append(data.qvel[i])
        joint_torque_feedback.append(data.qfrc_actuator[i])

    node.publisher_fn(joint_pos_feedback, joint_vel_feedback, joint_torque_feedback, imu_msg_data)

    # Reset Model
    if (glfw.get_key(viewer.window,glfw.KEY_BACKSPACE) == glfw.PRESS):
        mj.mj_resetData(model, data)

    # Apply perturbations
    # Key Map:   N = Forward Push; B = Backward Push; Z = Left Push; X = Right Push
    # Force and torque are defined for torso position in global frame; mj.mj_applyFT transforms this to generalized coordinates 
    force_x = 500.0
    force_y = 500.0
    force_z = 500.0
    
    # Limit button push to single trigger of perturbation
    
    if (glfw.get_key(viewer.window,glfw.KEY_N) == 1):
        # Record key state
        key_press_flag[0] = 1  
        force = np.array([force_x, 0.0, 0.0])
        torque = np.array([0.0, 0.0, 0.0])
        # Key press state lock
        if key_press_flag[0] != key_press_flag[1]:
            mj.mj_applyFT(model, data, force, torque, np.array(pose[0:3]), 1, data.qfrc_applied)

    if (glfw.get_key(viewer.window,glfw.KEY_B) == 1):
        # Record key state
        key_press_flag[0] = 1
        force = np.array([-force_x, 0.0, 0.0])
        torque = np.array([0.0, 0.0, 0.0])
        # Key press state lock
        if key_press_flag[0] != key_press_flag[1]:
            mj.mj_applyFT(model, data, force, torque, np.array(pose[0:3]), 1, data.qfrc_applied)
    
    if (glfw.get_key(viewer.window,glfw.KEY_Z) == 1):
        # Record key state
        key_press_flag[0] = 1
        force = np.array([0.0, force_y, 0.0])
        torque = np.array([0.0, 0.0, 0.0])
        # Key press state lock
        if key_press_flag[0] != key_press_flag[1]:
            mj.mj_applyFT(model, data, force, torque, np.array(pose[0:3]), 1, data.qfrc_applied)

    if (glfw.get_key(viewer.window,glfw.KEY_X) == 1):
        # Record key state
        key_press_flag[0] = 1
        force = np.array([0.0, -force_y, 0.0])
        torque = np.array([0.0, 0.0, 0.0])
        # Key press state lock
        if key_press_flag[0] != key_press_flag[1]:
            mj.mj_applyFT(model, data, force, torque, np.array(pose[0:3]), 1, data.qfrc_applied)

    if (glfw.get_key(viewer.window,glfw.KEY_N) == glfw.RELEASE and glfw.get_key(viewer.window,glfw.KEY_B) == glfw.RELEASE and glfw.get_key(viewer.window,glfw.KEY_Z) == glfw.RELEASE and glfw.get_key(viewer.window,glfw.KEY_X) == glfw.RELEASE):
        # Record key state
        key_press_flag[0] = 0
    
    # Record key state history
    key_press_flag[1]=key_press_flag[0]
    

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
