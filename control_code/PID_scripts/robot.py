import random
import numpy as np
import matplotlib.pyplot as plt
from car_model import plot_car

class Robot(object):

    def __init__(self, length=4.5):
        """
        Initialize robot and its position and orientation as 0, 0, 0.
        """
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0    # Angle with the positive direction of the X-axis (in radians)
        self.length = length      # Wheelbase of front and rear wheels
        self.steering_noise = 0.0 # steering_noise
        self.distance_noise = 0.0 # distance_noise
        self.steering_drift = 0.0 # steering_drift

        self.default_state = {"x": self.x,"y": self.y,"o": self.orientation}

    def reset(self):
        self.x = self.default_state["x"]
        self.y = self.default_state["y"]
        self.orientation = self.default_state["o"]

    def set(self, x, y, orientation):
        """
        Set the coordinates and direction of the robot
        """
        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)
        self.default_state = {"x": self.x,"y": self.y,"o": self.orientation}

    def set_noise(self, steering_noise, distance_noise):
        """
        Set noise parameters 
        :param steering_noise
        :param distance_noise
        """
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, drift):
        """
        Set the steering drift parameters of the system
        """
        self.steering_drift = drift

    def move(self, steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):
        """
        Car move function

        :param steering: The steering angle of the front wheels, the maximum value is max_steering_angle
        :param distance: >= 0 
        :param tolerance: The minimum difference (threshold) of steering, 
                        when it is less than this threshold, let the car go straight, 
                        the unit is radian
        :param max_steering_angle: Maximum steering angle, defaults to 180 / 4.0 = 45°
        """
        # if steering > max_steering_angle:
        #     steering = max_steering_angle
        # if steering < -max_steering_angle:
        #     steering = -max_steering_angle
        steering = np.clip(steering, -max_steering_angle, max_steering_angle)
        if distance < 0.0:
            distance = 0.0

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # apply steering drift
        steering2 += self.steering_drift

        # angular velocity = linear velocity / turning radius
        # Execute motion
        turn = np.tan(steering2) * distance2 / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion 
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)


def show(x_trajectory, y_trajectory, p_array=[], i_array=[], d_array=[], label = 'PID'):
    n = len(x_trajectory)
    # fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
    # fig, ax1 = plt.subplots(figsize=(8, 4))
    fig = plt.figure()
    ax1 = fig.add_subplot(211)
    ax1.plot(x_trajectory, np.zeros(n), 'pink', label='reference')
    ax1.plot(x_trajectory, y_trajectory, 'black', label= label + ' controller')
    ax1.set_xlabel('x')  # Add an x-label to the axes.
    ax1.set_ylabel('y')  # Add a y-label to the axes.
    ax1.set_title('Car-Position')
    h, l = ax1.get_legend_handles_labels()
    ax1.legend(h, l)  # h is a list of line objects, l is a list of text descriptions

    ax2 = fig.add_subplot(212)
    if len(p_array) > 0:
        ax2.plot(x_trajectory, p_array, color='r', label='p')
    if len(i_array) > 0:
        ax2.plot(x_trajectory, i_array, color='g', label='i')
    if len(d_array) > 0:
        ax2.plot(x_trajectory, d_array, color='b', label='d')

    ax2.set_title('PID-Value')
    h, l = ax2.get_legend_handles_labels()
    ax2.legend(h, l)  # h is a list of line objects, l is a list of text descriptions

    plt.ylim((-0.5, 0.5))
    plt.tight_layout()
    plt.show()