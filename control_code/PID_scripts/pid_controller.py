"""
PID controller
"""
from robot import Robot, show
import numpy as np
from car_model import do_simulation,calculate_steering_angle

def run(robot, k_p, k_d, k_i, n=100, speed=1.0):
    """
    Run multiple times and record trajectories
    :param robot:   car
    :param n:       iteration times 
    :param speed:   car speed
    :param k_p:     Gain P parameters
    :param k_d:     Gain D parameters
    :param k_i:     Gain I parameters
    """
    x_trajectory = []
    y_trajectory = []
    steering = []
    p_arr = []
    d_arr = []
    i_arr = []
    prev_cte = robot.y # previous cte
    cte_sum = 0
    for i in range(n):
        # ---------------------------- start
        cte = 0.0 - robot.y # error
        p = k_p * cte                    # p

        d = k_d * (cte - prev_cte) / 1.0 # d
        prev_cte = cte

        cte_sum += cte
        i = k_i * cte_sum                # i

        steer = p + d + i

        p_arr.append(p)
        d_arr.append(d)
        i_arr.append(i)
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
    return x_trajectory, y_trajectory, steering,p_arr, d_arr, i_arr

def pid_main(robot,k_p,k_d,k_i):
    robot.reset()
    # run and collect all x,y
    x_trajectory, y_trajectory, steering,p_arr, d_arr, i_arr = run(robot, k_p=k_p, k_d=k_d, k_i=k_i)
    # simulation
    do_simulation(x_trajectory, y_trajectory,steering)
    # visualize running results
    show(x_trajectory, y_trajectory, p_array=p_arr, d_array=d_arr, i_array=i_arr, label='PID')

if __name__ == '__main__':
    # create robot
    robot = Robot()
    # initial pose x=0, y=-1, orient=0
    robot.set(0, -1, 0)
    # Set 10 degrees of wheel systematic deviation
    robot.set_steering_drift(10. / 180. * np.pi)

    k_p, k_d, k_i = [0.2, 3.0, 0.01]
    # run and collect all x,y
    x_trajectory, y_trajectory, steering,p_arr, d_arr, i_arr = run(robot, k_p=k_p, k_d=k_d, k_i=k_i)
    # simulation 
    do_simulation(x_trajectory, y_trajectory,steering)
    # visualize running results
    show(x_trajectory, y_trajectory, p_array=p_arr, d_array=d_arr, i_array=i_arr, label='PID')
    
    # try another set of value
    # k_p, k_d, k_i = [10.716018504541431, 18.68325573584582, 0.020275559590445292]
    # robot.reset()
    # # run and collect all x,y
    # x_trajectory, y_trajectory, steering,p_arr, d_arr, i_arr = run(robot, k_p=k_p, k_d=k_d, k_i=k_i)
    
    # # visualize running results
    # show(x_trajectory, y_trajectory, p_array=p_arr, d_array=d_arr, i_array=i_arr, label='PID')

