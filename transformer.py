import numpy as np
import math

class Transformer():

    @staticmethod
    def euler_to_orientation(rocket_attitude):
        
        ## rocket_attitude: body axis
        roll  = rocket_attitude[0]
        pitch = rocket_attitude[1]
        yaw   = rocket_attitude[2]

        rotation_roll  = np.array([[1,             0,              0],
                                   [0,np.cos(roll),-np.sin(roll)],
                                   [0,np.sin(roll), np.cos(roll)]])
                
        rotation_pitch = np.array([[ np.cos(pitch),0,np.sin(pitch)],
                                   [               0,1,              0],
                                   [-np.sin(pitch),0,np.cos(pitch)]])

        rotation_yaw   = np.array([[np.cos(yaw),-np.sin(yaw),0],
                                   [np.sin(yaw), np.cos(yaw),0],
                                   [            0,             0,1]])

        body2ground    = np.array([[0, 1, 0],
                                   [0, 0,-1],
                                   [1, 0, 0]])

        orientation = np.dot(np.dot(body2ground, np.dot(rotation_yaw, np.dot(rotation_pitch, rotation_roll))), np.array([1,0,0]))

        return orientation
