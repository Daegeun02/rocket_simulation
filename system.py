from visualize import Visualizer
from rocket_dynamic import Rocket, Translation, Rotation
import numpy as np
import matplotlib.pyplot as plt
import time

class System:
	def __init__(self, linear_state, angular_state, dt, rockets_shape):
		self.rocket = Rocket(linear_state, angular_state, dt, rockets_shape)
		
		self.translation = Translation(self.rocket)
		self.rotation = Rotation(self.rocket)
		# self.visualizer = Visualizer(self.rocket)

		self.t = 0

	def run(self):
		self.rocket.update(self.t)
		self.translation.momentum(self.t)
		self.rotation.momentum()
		# self.visualizer.run()

		self.t += self.rocket.dt
		
		# print(np.round(self.rocket.linear_state[:3], 4))

if __name__ == "__main__":
	linear_state = np.zeros(6)
	angular_state = np.zeros(6)
	angular_state[0] = 90
	angular_state[1] = 2
	angular_state[2] = 2 
		
	dt = 0.1
	rockets_shape = np.array([3, 0.4, 1.5, 0.5, 0.25, 3, 0.11, 100])

	system = System(linear_state, angular_state, dt, rockets_shape)
	
	while system.t < 10:
		system.run()
		print(system.rocket.angular_state[:3])
		time.sleep(0.1)	
	# plt.show()