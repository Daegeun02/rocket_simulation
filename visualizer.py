import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import numpy as np
from transformer import Transformer

class Visualizer:
	def __init__(self, rocket, mode=None):
		self.rocket = rocket
		self.mode = mode 

		if self.mode == None:
			plt.subplot(1,1,1, projection="3d")

			plt.gca().set_xlim(-15,15)
			plt.gca().set_ylim(-15,15)
			plt.gca().set_zlim(-10,10)

			plt.quiver(0,0,0,1,0,0,length=5,color="red")
			plt.quiver(0,0,0,0,1,0,length=5,color="green")
			plt.quiver(0,0,0,0,0,1,length=5,color="blue")
			
		elif self.mode == "flight_data":
			plt.figure(figsize=(7,10))

		else:
			return NotImplementedError()

	def run(self, t):
		
		if self.mode == None:
			self.flight_monitering()

		elif self.mode == "flight_data":
			self.flight_data(t)

		else:
			return NotImplementedError()

	def flight_monitering(self):

		position = self.rocket.linear_state[:3]
		orientation = Transformer.euler_to_orientation(self.rocket.angular_state[:3])

		length_bot = self.rocket.length - self.rocket.central_gravity
		length_top = -self.rocket.central_gravity

		thrust_vec = self.rocket.thrust
		thrust = np.linalg.norm(thrust_vec)	
		
		plt.cla()
		plt.gca().set_xlim(-100,100)
		plt.gca().set_ylim(-100,100)
		plt.gca().set_zlim(0,200)
		

		plt.quiver(position[0], position[1], position[2],\
				   orientation[0], orientation[1], orientation[2],\
				   length=length_top*20, linewidth=2.0, arrow_length_ratio=0.0, color="black")

		plt.quiver(position[0], position[1], position[2],\
				   orientation[0], orientation[1], orientation[2],\
				   length=length_bot*20, linewidth=2.0, arrow_length_ratio=0.0, color="black")
		
		plt.quiver(position[0], position[1], position[2],\
				   -thrust_vec[0], -thrust_vec[1], -thrust_vec[2],\
				   length=thrust*1e-3, linewidth=1.5, arrow_length_ratio=0.0, color="red")	

		plt.pause(0.01)
		
	def flight_data(self, t):

		position = self.rocket.linear_state[:3]
		velocity = self.rocket.linear_state[3:]
		orientation = Transformer.euler_to_orientation(self.rocket.angular_state[:3])

		plt.subplot(2,3,1)
		plt.scatter(t/self.rocket.dt, velocity[0])
		
		plt.subplot(2,3,2)
		plt.scatter(t/self.rocket.dt, velocity[1])

		plt.subplot(2,3,3)
		plt.scatter(t/self.rocket.dt, velocity[2])

		plt.subplot(2,3,4)
		plt.scatter(t/self.rocket.dt, orientation[0])

		plt.subplot(2,3,5)
		plt.scatter(t/self.rocket.dt, orientation[1])

		plt.subplot(2,3,6)
		plt.scatter(t/self.rocket.dt, orientation[2])
