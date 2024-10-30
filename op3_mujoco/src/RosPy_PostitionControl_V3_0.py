#!/usr/bin/env python3

import mujoco as mj
import mujoco_viewer
import numpy as np
import os
import rospy
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Pose
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header, Float64, String
import glfw
#import datetime
import time

simend = 100

class mujoco_ros:

    def callback(self, data, joint): 
        
        self.joint_angle[self.actuator_names.index(joint)] = data.data
        self.trajectory.append(self.joint_angle)
        #self.now = datetime.datetime.now()
        #print("Ctrl Cycle, Input time duration: ", self.now.time())  

    def callback__(self,data):
        
        if str(data).find("play") != -1:
            print("Getting Trajectory!!")
            self.log = True       

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
        
        # Var for Trajectory Queue
        self.trajectory = []

        # Save desired joint states in the order defined in actuator_names
        self.joint_angle  = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
                             0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]  
        
        self.joint_vel    = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
                             0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        
        self.joint_torque = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
                             0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
                
        self.publisher_Joint = rospy.Publisher('/robotis_op3/joint_states', JointState, queue_size=1)
        self.publisher_Imu = rospy.Publisher('/robotis_op3/Sim_IMU', Imu, queue_size=1)
        self.publisher_Pose = rospy.Publisher('/robotis_op3/torso_orientation', Pose, queue_size=1)
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
                    
        for joint in self.actuator_names:
            topic = "/robotis_op3/"+joint+"_position/command" 
            rospy.Subscriber(topic, Float64, self.callback, (joint)) 
        
        # Data Log
        topic = "/adol/lifting/command"
        rospy.Subscriber(topic, String, self.callback__)
        self.log = False
 

    def publisher_fn(self, joint_pos_feedback, joint_vel_feedback, joint_torque_feedback, imu_msg_data, pose, sim_time):  
        
        # Create a Clock message
        clock_msg = Clock()
        clock_msg.clock = rospy.Time.from_sec(sim_time)

        # Publish the Clock message
        self.clock_pub.publish(clock_msg)

        # Initialize JointState msg    
        self.joint_states = JointState()
        self.joint_states.header = Header()
        self.joint_states.header.stamp = rospy.Time.now()
        self.joint_states.name = self.actuator_names
        self.joint_states.position = joint_pos_feedback
        self.joint_states.velocity = joint_vel_feedback
        self.joint_states.effort = joint_torque_feedback

        # Publish JointState msg
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
        
        # Initializing Quaternion msg
        self.torso_pose = Pose()
        # Position
        self.torso_pose.position.x, self.torso_pose.position.y, self.torso_pose.position.z = pose
        
        # Orienataion
        self.torso_pose.orientation.x, self.torso_pose.orientation.y, self.torso_pose.orientation.z, self.torso_pose.orientation.w = imu_msg_data[2]

        self.publisher_Pose.publish(self.torso_pose)


def controller(model, data):

    global node, key_press_flag

    # Reset applied force to 0 at each iteration
    data.qfrc_applied.fill(0)   

    # Pull first value from trajectory queue 
    if len(node.trajectory) != 0:
        joints = node.trajectory[0]
        del node.trajectory[0]
    else:
        joints = node.joint_angle

    # Assign joint values to joints in mujoco
    for i in node.actuator_names:
        data.ctrl[node.actuator_names.index(i)] = joints[node.actuator_names.index(i)]
    
###################################################Sensor Feedback###########################################################

    # Extract sensor values required for IMU   
    imu_msg_data =[]        # order = Gyro(3x1), Accel(3x1), Orientation(4x1)
    for i in range(3):      # extract Pos, Gyro, Accel 
        if i == 0:
            pose = data.sensordata[int(len(node.actuator_names)+(3*i)):int(len(node.actuator_names)+(3*(i+1)))] 
        else:   
            imu_msg_data.append(data.sensordata[int(len(node.actuator_names)+(3*i)):int(len(node.actuator_names)+(3*(i+1)))])
    imu_msg_data.append(data.sensordata[-(4+6):-6])

    #print(data.sensordata[0:3]-data.sensordata[-6:-3]) 

    # Clear feedback values
    joint_pos_feedback = []
    joint_vel_feedback = []
    joint_torque_feedback = []

    # Read the joint angles
    for i in range(len(node.actuator_names)):
        joint_pos_feedback.append(data.qpos[i+7])       # data.qpos len=27; 0-6 pose of model centre coordinate
        joint_vel_feedback.append(data.qvel[i])
        joint_torque_feedback.append(data.qfrc_actuator[i])

    sim_time = data.time
    node.publisher_fn(joint_pos_feedback, joint_vel_feedback, joint_torque_feedback, imu_msg_data, pose, sim_time)
    #print (len(model.body_mass), len(data.xipos), (mj.mj_getTotalmass(model))) #data.xipos
    
    # Calculate COM
    sum_mi=0
    for i in range(len(data.xipos)):
        sum_mi += data.xipos[i]* model.body_mass[i]
    com_pos = sum_mi/mj.mj_getTotalmass(model)
    #print (com_pos)

    if node.log==True: 
        f.write(str(sim_time)+"\t")
        f.write(str(com_pos)+"\n")

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


f = open("op3_com_log.txt", "a")

# Start listner
rospy.init_node("MuJoCo_Listner")

key_press_flag = [0, 0]

## Load MuJoCo Model and data structures
current_folder = os.path.dirname(os.path.abspath(__file__))
#xml_path = os.path.join(current_folder, "robotis_op3", "scene.xml")
xml_path = os.path.join(current_folder, "robotis_op3", "scene_lifting.xml")

model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)

# create the viewer object
viewer = mujoco_viewer.MujocoViewer(model, data, title="OP3_Simulator")

#print(actuator_names)
node = mujoco_ros()


# Initialize ROS
run_mujoco_sim()


