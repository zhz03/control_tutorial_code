"""
pd controller
"""
from robot import Robot, show
import numpy as np
from car_model import do_simulation,calculate_steering_angle

def run(robot, k_p, k_d, n=100, speed=1.0):
    """
    Run multiple times and record trajectories
    :param robot:   car
    :param n:       iteration times 
    :param speed:   car speed
    :param k_p:     Gain P parameters
    :param k_d:     Gain D parameters
    """
    x_trajectory = []
    y_trajectory = []
    steering = []
    p_arr = []
    d_arr = []
    prev_cte = robot.y # previous cte
    for i in range(n):
        # ---------------------------- start
        cte = 0.0 - robot.y # error
        p = k_p * cte                    # p

        d = k_d * (cte - prev_cte) / 1.0 # d
        prev_cte = cte

        steer = p + d

        p_arr.append(p)
        d_arr.append(d)
        # ---------------------------- end
        # Take steer as the deflection angle and speed as the speed to perform a movement
        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        last_x = robot.x
        last_y = robot.y
        # calculate steer based on the x,y
        if x_trajectory == []:
            steer = 0
        else:
            steering_angle = calculate_steering_angle(robot.x,robot.y,last_x,last_y)
        steering.append(steering_angle)

        print(robot)
    return x_trajectory, y_trajectory, steering, p_arr, d_arr


if __name__ == '__main__':
    # create robot
    robot = Robot()
    # initial pose x=0, y=-1, orient=0
    robot.set(0, -1, 0)
    # Set 10 degrees of wheel systematic deviation
    robot.set_steering_drift(10. / 180. * np.pi)

    # run and collect all x,y
    x_trajectory, y_trajectory, steering, p_arr, d_arr = run(robot, k_p = 0.2, k_d = 3.0)
    do_simulation(x_trajectory, y_trajectory,steering)
    # visualize running results
    show(x_trajectory, y_trajectory, p_array=p_arr, d_array=d_arr, label='PD')