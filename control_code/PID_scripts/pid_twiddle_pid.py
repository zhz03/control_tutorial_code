"""
PID parameters tuning
twiddle technique allows us to tune the parameters based on the mean error
This example is just to show the whole tuning process 
"""

from robot import Robot, show
import math


def build_robot():
    """
    Create a robot car
    Initial pose is (0,-1)
    Initial rotation angle is 10
    """
    robot = Robot()
    robot.set(0, -1, 0)
    robot.set_steering_drift(math.radians(10.))

    return robot

def run(robot, params, n = 100, speed = 1.0):
    """
    运行机器人
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


def twiddle(tol = 0.2):
    """
    :param tolerance: Minimum tolerance of coefficient of variation
    :return: params: include parameters list: k_p, k_d, k_i 
    """
    k  = [0, 0, 0]  # initial k_p, k_d, k_i
    dk = [1, 1, 1]  # initial parameter variation coefficient

    robot = build_robot()

    # run the robot to get the initial error mean
    x_trajectory, y_trajectory, best_err = run(robot, k)

    it = 0
    # Looping continuously modify k_p,k_d,k_i changes by dp less than tol threshold , and the modification range is determined by dp 
    # Start at sum(dk) == 3，until it is less than the minimum threhold tol == 0.2,the loop ends
    
    while sum(dk) > tol:
        for i in range(len(k)):
            k[i] += dk[i]

            # reset robot to get the updated mean error
            robot.reset()
            x_trajectory, y_trajectory, err = run(robot, k)
            if err < best_err:
                # it's best to reduce the error mean at this direction 
                best_err = err
                # expand the change in the gain coefficient 
                dk[i] *= 1.1
            else:
                # it's not good to reduce the mean erro at this direction,
                # thus remove the value just added, 
                # and it must be doubled in the opposite direction
                k[i] -= 2 * dk[i]

                robot.reset()
                x_trajectory, y_trajectory, err = run(robot, k)

                if err < best_err:
                    # it's best to reduce the error mean at this direction
                    best_err = err
                    dk[i] *= 1.1
                else:
                    k[i] += dk[i]  # restore the value of the gain coefficient
                    dk[i] *= 0.9  # reduce the gain coefficient change 

        it += 1

        print("Iteration: {}, best_err: {} dk: {}".format(it, best_err, dk))

    return k, best_err



if __name__ == '__main__':
    params, err = twiddle()

    print("Final twiddle error: {} params: {}".format(err, params))

    robot = build_robot()
    x_trajectory, y_trajectory, best_err = run(robot, params)
    show(x_trajectory, y_trajectory, label="Twiddle PID")

    # after we get the final pid value, we can put these value back to our previous controller