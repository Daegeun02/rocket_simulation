import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import numpy as np
from transformer import Transformer

class Visualizer:
	def __init__(self, rocket):
		self.rocket = rocket

		plt.subplot(1,1,1, projection="3d")

		plt.gca().set_xlim(-15,15)
		plt.gca().set_ylim(-15,15)
		plt.gca().set_zlim(-10,10)

		plt.quiver(0,0,0,1,0,0,length=5,color="red")
		plt.quiver(0,0,0,0,1,0,length=5,color="green")
		plt.quiver(0,0,0,0,0,1,length=5,color="blue")

	def run(self):
		pitch = self.rocket.angular_state[1]
		yaw   = self.rocket.angular_state[2]

		position = self.rocket.linear_state[:3]
		orientation_top = Transformer.body_to_earth(Transformer.spherical_coordinate(pitch,yaw))
		orientation_bot = Transformer.body_to_earth(Transformer.spherical_coordinate(pitch,yaw))

		length_bot = self.rocket.length - self.rocket.central_gravity
		length_top = -self.rocket.central_gravity

		thrust_vec = self.rocket.thrust
		thrust = np.linalg.norm(thrust_vec)	
		
		plt.cla()
		plt.gca().set_xlim(-100,100)
		plt.gca().set_ylim(-100,100)
		plt.gca().set_zlim(0,300)
		

		plt.quiver(position[0], position[1], position[2],\
				   orientation_top[0], orientation_top[1], orientation_top[2],\
				   length=length_top*20, linewidth=2.0, arrow_length_ratio=0.0, color="black")

		plt.quiver(position[0], position[1], position[2],\
				   orientation_bot[0], orientation_bot[1], orientation_bot[2],\
				   length=length_bot*20, linewidth=2.0, arrow_length_ratio=0.0, color="black")
		
		plt.quiver(position[0], position[1], position[2],\
				   -thrust_vec[0], -thrust_vec[1], -thrust_vec[2],\
				   length=thrust*0.01, linewidth=1.5, arrow_length_ratio=0.0, color="red")	
		
		plt.pause(0.1)
