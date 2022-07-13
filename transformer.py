import numpy as np

class Transformer():
    def __init___(self):
        pass
    
    @staticmethod
    def body_to_earth(rocket_attitude):

        psi = rocket_attitude[0] * np.pi/180
        theta = rocket_attitude[1] * np.pi/180
        phi = rocket_attitude[2] * np.pi/180
        
        r = np.array([[np.cos(psi), -np.sin(psi), 0],       
                      [np.sin(psi),  np.cos(psi), 0],
                      [          0,            0, 1]])
 
        y= np.array([[ np.cos(phi), 0, np.sin(phi)],
                      [             0, 1,             0],
                      [-np.sin(phi), 0, np.cos(phi)]])

        p = np.array([[1,           0,            0],
                      [0, np.cos(theta), -np.sin(theta)],
                      [0, np.sin(theta),  np.cos(theta)]])

                     
        PP = np.array([[0, 0, -1],
                       [0, 1,  0],
                       [1, 0,  0]])
             
        transformation_mtx = np.dot(np.dot(r, y), p)

                    
        return transformation_mtx

    @staticmethod
    def spherical_coordinate(pitch, yaw):

        pitch_ = np.deg2rad(pitch)
        yaw_   = np.deg2rad(yaw)

        x = np.sin(yaw_) * np.cos(pitch_)
        y = np.sin(yaw_) * np.sin(pitch_)
        z = np.cos(yaw_)

        return np.array([x,y,z])