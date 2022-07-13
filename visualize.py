import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import numpy as np
from transformer import Transformer

class Visualizer:
	def __init__(self, rocket):
		self.rocket = rocket
		self.transformer = Transformer()

		plt.subplot(1,1,1, projection="3d")

		plt.gca().set_xlim(-15,15)
		plt.gca().set_ylim(-15,15)
		plt.gca().set_zlim(-10,10)

		plt.quiver(0,0,0,1,0,0,length=5,color="red")
		plt.quiver(0,0,0,0,1,0,length=5,color="green")
		plt.quiver(0,0,0,0,0,1,length=5,color="blue")

	def run(self):
		pose = np.zeros(6)

		pose[:3] = self.rocket.linear_state[:3]
		pose[3:] = self.rocket.angular_state[:3]
		pose[3:] = pose[3:] * self.rocket.length / np.linalg.norm(pose[3:])
		pose[3:] = np.dot(pose[3:], self.transformer.body_to_earth(np.array([0,0,90])))
		
		thrust = self.rocket.thrust / (np.linalg.norm(self.rocket.thrust) + 1e-4)
		
		plt.cla()
		plt.gca().set_xlim(-100,100)
		plt.gca().set_ylim(-100,100)
		plt.gca().set_zlim(0,300)
		

		plt.quiver(pose[0], pose[1], pose[2],\
				   pose[3], pose[4], pose[5],\
				   length=20, linewidth=2.0, arrow_length_ratio=0.0, color="black")

		
		plt.quiver(pose[0], pose[1], pose[2],\
				   -thrust[0], -thrust[1], -thrust[2],\
				   length=10, linewidth=1.5, arrow_length_ratio=0.0, color="red")	
		
		plt.pause(0.05)
