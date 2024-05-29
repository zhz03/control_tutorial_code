import matplotlib.pyplot as plt

class OpenLoopControl:
    def __init__(self, input_signal, initial_value, iterations):
        self.input_signal = input_signal
        self.value = initial_value
        self.iterations = iterations
        self.history = []

    def control(self):
        for i in range(self.iterations):
            self.value += self.input_signal[i]
            self.history.append(self.value)

    def plot(self):
        plt.plot(self.history, label='Process Value')
        plt.xlabel('Iteration')
        plt.ylabel('Value')
        plt.title('Open Loop Control')
        plt.legend()
        plt.show()

# use case 
input_signal = [1 for _ in range(50)]  
open_loop = OpenLoopControl(input_signal=input_signal, initial_value=0, iterations=50)
open_loop.control()
open_loop.plot()
