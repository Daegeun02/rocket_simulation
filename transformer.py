import numpy as np

class Transformer():

    @staticmethod
    def body_to_earth(rocket_attitude):
        
        ## rocket_attitude: body axis

        psi = np.pi / 2
                
        y = np.array([[np.cos(psi), -np.sin(psi), 0],       
                      [np.sin(psi),  np.cos(psi), 0],
                      [          0,            0, 1]])
 
        transformation_mtx = y

        earth_attitude = np.dot(transformation_mtx, rocket_attitude)                    

        return earth_attitude

    @staticmethod
    def spherical_coordinate(pitch, yaw):

        pitch_ = np.deg2rad(pitch)
        yaw_   = np.deg2rad(yaw)

        x = np.sin(pitch_) * np.cos(yaw_)
        y = np.sin(pitch_) * np.sin(yaw_)
        z = np.cos(pitch_)

        return np.array([x,y,z])