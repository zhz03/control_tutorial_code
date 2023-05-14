# pid controller
from robot import Robot, show
from car_model import do_simulation


def run(robot, n=100, speed=1.0):
    """
    Run multiple times and record trajectories
    :param robot:   car
    :param n:       iteration times 
    :param speed:   car speed
    """
    x_trajectory = []
    y_trajectory = []
    for i in range(n):
        # ---------------------------- start

        steer = 0.0

        # ---------------------------- end
        # Take steer as the deflection angle and speed as the speed to perform a movement
        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        print(robot)
    return x_trajectory, y_trajectory

def test_run(robot, n=100, speed=1.0):
    x_trajectory = []
    y_trajectory = []
    steering = []
    for i in range(n):
        # ---------------------------- start

        steer = 0.0

        # ---------------------------- end
        # Take steer as the deflection angle and speed as the speed to perform a movement
        robot.move(steer, speed)
        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)
        steering.append(steer)
        print(robot)

    return x_trajectory, y_trajectory,steering    

if __name__ == '__main__':
    # create robot
    robot = Robot()
    # initial pose x=0, y=-1, orient=0
    robot.set(0, -10, 0)

    # run and collect all x,y
    # x_trajectory, y_trajectory = run(robot)
    # print(x_trajectory)
    x_trajectory, y_trajectory,steering = test_run(robot)
    do_simulation(x_trajectory, y_trajectory,steering)
    # visualize running results
    show(x_trajectory, y_trajectory, label='Car')