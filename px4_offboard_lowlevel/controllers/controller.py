import numpy as np

class Controller():
    def __init__(self):
        self.name = 'controller'
		
    def set_k_position_gain(self, position_gain):
        self.position_gain_ = position_gain
        
    def set_k_velocity_gain(self, velocity_gain):
        self.velocity_gain_ = velocity_gain

    def set_k_attitude_gain(self, attitude_gain):
        self.attitude_gain_ = attitude_gain


    def set_k_angularRate_gain(self, angular_rate_gain):
        self.angular_rate_gain_ = angular_rate_gain


    def set_uav_mass(self,  uav_mass):
        self._uav_mass = uav_mass

    def set_inertia_matrix(self, inerti_matrix):
        self._inertia_matrix = inerti_matrix

    def set_gravity(self, gravity):
        self._gravity = gravity
