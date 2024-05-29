import matplotlib.pyplot as plt

class CloseLoopControl:
    def __init__(self, setpoint, initial_value, kp, iterations):
        self.setpoint = setpoint
        self.value = initial_value
        self.kp = kp
        self.iterations = iterations
        self.history = []

    def control(self):
        for _ in range(self.iterations):
            error = self.setpoint - self.value
            adjustment = self.kp * error
            self.value += adjustment
            self.history.append(self.value)

    def plot(self):
        plt.plot(self.history, label='Process Value')
        plt.axhline(y=self.setpoint, color='r', linestyle='--', label='Setpoint')
        plt.xlabel('Time')
        plt.ylabel('Temperature')
        plt.title('Close Loop Control')
        plt.legend()
        plt.show()

# use case
close_loop = CloseLoopControl(setpoint=50, initial_value=0, kp=0.1, iterations=50)
close_loop.control()
close_loop.plot()
