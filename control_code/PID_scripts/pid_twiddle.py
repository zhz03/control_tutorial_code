"""
PID parameters tuning
twiddle technique allows us to tune the parameters based on the mean error
This example is just to show the initial situation, no tuning 
"""
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

def run(robot, params, n = 100, speed = 1.0):
    """
    run robot 
    :param robot: running robot
    :param params: include parameters list: k_p, k_d, k_i 
    """
    x_trajectory = []
    y_trajectory = []

    # record total error 
    err = 0

    k_p, k_d, k_i = params
    prev_cte = 0 - robot.y # last error 
    cte_sum = 0 # cumulative error 
    for i in range( 2 * n ):
        cte = 0 - robot.y
        diff_cte = cte - prev_cte
        prev_cte = cte
        cte_sum += cte

        steer = k_p * cte + k_d * diff_cte + k_i * cte_sum

        robot.move(steer, speed)

        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)

        if i >= n:
            # From the begining of half of the whole process to the end, 
            # the error is accumulated
            err += cte ** 2

    return x_trajectory, y_trajectory, err / n


def twiddle(tol = 0.1):
    """
    :param tolerance: Minimum tolerance of coefficient of variation
    :return: params: include parameters list: k_p, k_d, k_i 
    """
    k_pid = [0, 0, 0]  # initial k_p value 
    dp = 1.0  # initial parameter variation coefficient

    robot = build_robot()

    # run the robot to get the initial error mean
    x_trajectory, y_trajectory, best_err = run(robot, k_pid)

    it = 0
    # Looping to continuously modify k_p changes by dp less than tol threshold , and the modification range is determined by dp 
    # while dp > tol:
    #     # Looping to continuously modify k_p changes by dp less than tol threshold 
    #     it += 1
    #     print("Iteration: {}, best_err: {} dp: {}".format(it, best_err, dp))

    return k_pid, best_err


if __name__ == '__main__':
    params, err = twiddle()

    print("Final twiddle error: {} params: {}".format(err, params))

    robot = build_robot()
    x_trajectory, y_trajectory, best_err = run(robot, params)
    show(x_trajectory, y_trajectory, label="Twiddle PID")