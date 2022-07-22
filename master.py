from visualizer import Visualizer
from rocket_dynamic import Rocket, Translation, Rotation
from transformer import Transformer
import numpy as np
import matplotlib.pyplot as plt
import time

class Master:
	def __init__(self, linear_state, angular_state, dt, rockets_shape):
		self.rocket = Rocket(linear_state, angular_state, dt, rockets_shape)
		
		self.translation = Translation(self.rocket)
		self.rotation = Rotation(self.rocket)
		self.visualizer = Visualizer(self.rocket, mode="flight_data")

		self.t = 0

	def run(self):
		self.rocket.update(self.t)
		self.translation.momentum(self.t)
		self.rotation.momentum()
		self.visualizer.run(self.t)
		

		self.t += self.rocket.dt
		
		## position
		# print(f"position: {np.round(self.rocket.linear_state[3:], 3)}")

		## orientation
		# print(f"orientation: {np.round(Transformer.euler_to_orientation(self.rocket.angular_state[:3]), 3)}")

		print("========================================")
		print("\n")



if __name__ == "__main__":
	linear_state = np.zeros(6)
	angular_state = np.zeros(6)
	angular_state[0] = 0	# roll
	angular_state[1] = 0	# pitch
	angular_state[2] = np.deg2rad(10)	# yaw 
		
	dt = 0.1
	
	## ["structure_mass", "fuel_mass", "length", "structure_CG", "fuel_CG", "burnout_time", "diameter", "thrust"]
	rockets_shape = np.array([3, 0.4, 1.5, 0.5, 0.25, 3, 0.11, 100])

	master = Master(linear_state, angular_state, dt, rockets_shape)
	
	while master.t < 10:

		master.run()

		time.sleep(0.01)

	# plt.show()