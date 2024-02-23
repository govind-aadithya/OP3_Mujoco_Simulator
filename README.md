# **OP3 SIMULATOR**

This ROS package is a simulation platform for OP3 Robot. It is developed to directly work with the ROS interface developed by Robotis making the transition from simulation to implementation easier.

## **Dependencies**

1. MuJoCo (>2.1.2)

Install the latest version with this command:

'''sh
$ pip install mujoco
'''

2. Viewer for MuJoCo in Python [repo](https://github.com/rohanpsingh/mujoco-python-viewer)

Single line install

'''sh
$ pip install mujoco-python-viewer
'''

## **Package Installation**

Clone the repository to your workspace.

'''
$ git clone https://github.com/govind-aadithya/OP3_Mujoco_Simulator.git
'''

'''catkin_make op3_mujoco''' or '''catkin build op3_mujoco'''


## **Run Simulator**

Use ros run to run the node.

'''sh
$  rosrun op3_mujoco RosPy_PostitionControl_V2_1.py
'''
