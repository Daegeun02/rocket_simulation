import numpy as np
from transformer import Transformer

class Rocket:
	def __init__(self, linear_state, angular_state, dt, rockets_shape):
		## rockets_data
		## ["structure_mass", "fuel_mass", "length", "structure_CG", "fuel_CG", "burnout_time", "diameter", "thrust"]
		self.linear_state = linear_state
		self.angular_state = angular_state
		self.dt = dt

		## unpack rockets_data
		self.structure_mass = rockets_shape[0]
		self.fuel_mass 		= rockets_shape[1]
		self.length 		= rockets_shape[2]
		self.structure_CG   = rockets_shape[3]
		self.fuel_CG 		= rockets_shape[4]
		self.burnout_time 	= rockets_shape[5]
		self.diameter 		= rockets_shape[6]

		## inertia matrix
		self.inertia = np.zeros(3)

		self.central_gravity = None

		## drag force
		self.drag =None
		self.Cd = 0.5
		
		## Angle of Attack
		self.aoa = None
		
		## Thrust
		pitch = self.angular_state[1]
		yaw   = self.angular_state[2]
		self.thrust = Transformer.spherical_coordinate(pitch, yaw) * rockets_shape[7]
		 
	def update(self, t):
		self.delta_mass(t)
		self.center_of_gravity(t)
		self.angle_of_attack()
		self.moment_of_inertia()
		self.drag_force()

	def delta_mass(self, t):
		if t > self.burnout_time:
			pass

		else:
			burning_ratio = t / self.burnout_time
			self.fuel_mass *= (1 - burning_ratio)
			self.mass = self.structure_mass + self.fuel_mass

	def center_of_gravity(self, t):
		if t > self.burnout_time:
			pass
		else:
			structure = self.structure_CG * self.structure_mass
			fuel = self.fuel_CG * self.fuel_mass
			self.central_gravity = (structure + fuel) / (self.mass)

	def angle_of_attack(self):
		velocity = self.linear_state[3:]
		speed = np.linalg.norm(velocity)
		
		direction = self.angular_state[:3]
		heading = np.linalg.norm(direction)

		self.aoa = np.arccos(np.dot(velocity, direction) / (speed * heading + 1e-4))

	def moment_of_inertia(self):
		length2top = self.length - self.central_gravity
		length2bot = -self.central_gravity

		self.inertia[0] = (1/4) * 0.5 * self.mass * self.diameter ** 2
		self.inertia[1] = (1/3) * self.mass * (length2top ** 3 + length2bot ** 3) / self.length
		self.inertia[2] = self.inertia[1]

	def drag_force(self):
		disk_area = (1/4) * np.pi * self.diameter ** 2
		body_area = self.diameter * self.length
		rho = 1.225

		self.drag = 0.5 * rho * self.Cd * (disk_area * np.sin(self.aoa) + body_area * np.cos(self.aoa)) * \
					np.linalg.norm(self.linear_state[3:])		



class Dynamic:
	def __init__(self, rocket):
		## initialize
		self.rocket = rocket

		## dynamic matrices
		self.momentum_matrix = None
		self.external_matrix = None

		## gravity
		self.gravity = np.array([0,0,-9.8])

	def momentum(self):
		return NotImplementedError()

	def external_force(self):
		return NotImplementedError()

	def dynamic_matrix(self):
		## dynamic system, it is equal in Linear and Angular
		## normal dynamic system, use 6 dof...

		self.external_matrix = np.array([[0.5*self.rocket.dt**2,0,0],
										 [0,0.5*self.rocket.dt**2,0],
										 [0,0,0.5*self.rocket.dt**2],
										 [self.rocket.dt,0,0],
										 [0,self.rocket.dt,0],
										 [0,0,self.rocket.dt]])



class Translation(Dynamic):
	def __init__(self, rocket):
		super().__init__(rocket)

	def momentum(self, t):
		self.dynamic_matrix()

		acceleration = self.external_force(t)

		# print(acceleration)

		new_state = np.dot(self.momentum_matrix, self.rocket.linear_state) + \
					np.dot(self.external_matrix, acceleration) 	

		self.rocket.linear_state = new_state

	def external_force(self, t):
		if t > self.rocket.burnout_time:
			self.rocket.thrust = np.array([0,0,0])

		else:
			pass

		return self.rocket.thrust / self.rocket.mass + self.gravity

	def dynamic_matrix(self):

		self.momentum_matrix = np.array([[1,0,0,self.rocket.dt*(1-0.5*self.rocket.drag*self.rocket.dt),0,0],
										 [0,1,0,0,self.rocket.dt*(1-0.5*self.rocket.drag*self.rocket.dt),0],
										 [0,0,1,0,0,self.rocket.dt*(1-0.5*self.rocket.drag*self.rocket.dt)],
										 [0,0,0,1-self.rocket.drag*self.rocket.dt,0,0],
										 [0,0,0,0,1-self.rocket.drag*self.rocket.dt,0],
										 [0,0,0,0,0,1-self.rocket.drag*self.rocket.dt]])

		super().dynamic_matrix()



class Rotation(Dynamic):
	def __init__(self, rocket):
		super().__init__(rocket)

	def momentum(self):
		self.dynamic_matrix()

		acceleration = self.external_force()

		# print(acceleration)

		new_state = np.dot(self.momentum_matrix, self.rocket.angular_state) + \
					np.dot(self.external_matrix, acceleration)

		self.rocket.angular_state = new_state

	def external_force(self):
		pitch = self.rocket.angular_state[1]
		yaw   = self.rocket.angular_state[2]
		
		length2top = Transformer.spherical_coordinate(pitch, yaw) * (self.rocket.length-self.rocket.central_gravity)
		length2bot = Transformer.spherical_coordinate(pitch, yaw) * (-self.rocket.central_gravity)

		moment = self.rocket.drag * np.cross(length2top, self.rocket.linear_state[3:]) + \
				 self.rocket.drag * np.cross(length2bot, self.rocket.linear_state[3:])

		Eular = np.array([(self.rocket.inertia[1]-self.rocket.inertia[2])*self.rocket.angular_state[4]*self.rocket.angular_state[5],
						  (self.rocket.inertia[2]-self.rocket.inertia[0])*self.rocket.angular_state[5]*self.rocket.angular_state[3],
						  (self.rocket.inertia[0]-self.rocket.inertia[1])*self.rocket.angular_state[3]*self.rocket.angular_state[4]])	
		
		force = moment + Eular

		return force / self.rocket.inertia

	def dynamic_matrix(self):

		self.momentum_matrix = np.array([[1,0,0,self.rocket.dt,0,0],
										 [0,1,0,0,self.rocket.dt,0],
										 [0,0,1,0,0,self.rocket.dt],
										 [0,0,0,1-self.rocket.drag*self.rocket.dt,0,0],
										 [0,0,0,0,1-self.rocket.drag*self.rocket.dt,0],
										 [0,0,0,0,0,1-self.rocket.drag*self.rocket.dt]])

		super().dynamic_matrix()



class RCSActuator:
	def __init__(self, rocket):
		self.rocket = rocket

	def run(self):
		pass
