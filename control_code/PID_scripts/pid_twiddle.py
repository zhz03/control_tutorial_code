import numpy as np
from robot import Robot,show

def build_robot():
    """
    Create a robot car
    Initial pose is (0,-1)
    Initial rotation angle is 10
    """
    robot = Robot()
    robot.set(0,-1,0)
    robot.set_steering_drift(10/180*np.pi)
    return robot 

