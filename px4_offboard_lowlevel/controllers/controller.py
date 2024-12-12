import numpy as np
from scipy.spatial.transform import Rotation as R

class Controller():
    def __init__(self):
        self.name = 'controller'
        self.position_W_ = np.zeros(3)
        self.velocity_W_ = np.zeros(3)
        
        self.r_position_W_ = np.zeros(3)
        self.r_velocity_W_ = np.zeros(3)
        self.r_acceleration_W_ = np.zeros(3)
        
        self.position_gain_ = np.zeros(3)
        self.velocity_gain_ = np.zeros(3)
        self.attitude_gain_ = np.zeros(3)
        self.angular_rate_gain_ = np.zeros(3)
        
        self._inertia_matrix = np.zeros(3)
        
        self.angular_velocity_B_ = np.zeros(3)
        
        self.R_B_W_ = np.zeros((3,3))
        
        self.r_yaw = 0.0
        self.r_yaw_rate = 0.0
  
    def set_k_position_gain(self, position_gain):
        self.position_gain_ = position_gain
        
    def set_k_velocity_gain(self, velocity_gain):
        self.velocity_gain_ = velocity_gain

    def set_k_attitude_gain(self, attitude_gain):
        self.attitude_gain_ = attitude_gain


    def set_k_angularRate_gain(self, angular_rate_gain):
        self.angular_rate_gain_ = angular_rate_gain

    def set_uav_mass(self, uav_mass):
        self._uav_mass = uav_mass

    def set_inertia_matrix(self, inerti_matrix):
        self._inertia_matrix = inerti_matrix

    def set_gravity(self, gravity):
        self._gravity = gravity
        
    def set_trajectoryPoint(self, position_W, orientation_W):
        self.r_position_W_ = position_W
        self.r_velocity_W_ = np.zeros(3)
        self.r_acceleration_W_ = np.zeros(3)
        # Convert quaternion to rotation matrix
        orientation_W = np.roll(orientation_W, -1)    
        rot = R.from_quat(orientation_W)  # [x, y, z, w]
        self.r_R_B_W_ = rot.as_matrix()
        self.r_yaw = rot.as_euler('xyz', degrees=False)[2]
        self.r_yaw_rate = 0.0
        
        return self.r_R_B_W_, self.r_yaw
        
    # def set_trajectoryPoint(self, position_W, velocity_W, acceleration_W, orientation_W, angular_velocity_B):
    #     self.r_position_W_ = position_W
    #     self.r_velocity_W_ = velocity_W
    #     self.r_acceleration_W_ = acceleration_W
    #     # Convert quaternion to rotation matrix
    #     rot = R.from_quat(orientation_W)  # [x, y, z, w]
    #     self.r_R_B_W_ = rot.as_matrix()
    #     self.r_yaw = rot.as_euler('xyz', degrees=False)[2]
    #     self.r_yaw_rate = angular_velocity_B[2]
        
    # nhận từ phía drone
    def set_odometry(self, position_W, orientation_B_W, velocity_B, angular_velocity_B):
        rot = R.from_quat(orientation_B_W)  # [x, y, z, w]
        self.R_B_W_ = rot.as_matrix()
        self.position_W_ = position_W
        self.velocity_W_ = self.R_B_W_ @ velocity_B
        self.angular_velocity_B_ = angular_velocity_B
        
        
    # def calculate_controller_output(Eigen::VectorXd *controller_torque_thrust, Eigen::Quaterniond *desired_quaternion)
    def calculate_controller_output(self):
        # kiểm tra controller_torque_thrust có null không ?
        # assert(controller_torque_thrust)
        controller_torque_thrust = np.zeros(4) # 3 total moment and 1 total thrust

        # Geometric controller based on:
        # T. Lee, M. Leok and N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3),
        # " 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, USA, 2010.

        # Trajectory tracking.

        # 1. Compute translational tracking errors.
        # 	1.1 The tracking errors for the position
        # 	position_W_: vị trí thực tế, r_position_W_ : vị trí mong muốn
        e_p = self.position_W_ - self.r_position_W_
        # 	1.2 The tracking errors for the velocity
        # 	velocity_W_: vận tốc thực tế, r_velocity_W_ : vận tốc mong muốn
        e_v = self.velocity_W_ - self.r_velocity_W_

        # 2. caculate Iad: gia tốc mong muốn
        unit_z = np.array([0, 0, 1])
        I_a_d = (self._uav_mass * self._gravity * unit_z 
                 - np.multiply(self.position_gain_, e_p) 
                 - np.multiply(self.velocity_gain_, e_v)
                 + self._uav_mass * self.r_acceleration_W_)

        # 3. lực đẩy T
        thrust = I_a_d @ self.R_B_W_[:, 2]

        # 4. Bzd
        B_z_d = I_a_d / np.linalg.norm(I_a_d)

        # 5. Calculate Desired Rotational Matrix
        #  5.1 Bxd (do Bdx và Bdz ta cần chọn cho hai thằng này không song song)
        B_x_d = np.array([np.cos(self.r_yaw), np.sin(self.r_yaw), 0.0])
        #  5.1 Byd (từ Bzd and Bxd)
        B_y_d = np.cross(B_z_d, B_x_d)  # Compute the cross product of B_z_d and B_x_d
        B_y_d = B_y_d / np.linalg.norm(B_y_d)  # Normalize B_y_d
        #  5.2 Rdw
        # Assuming B_y_d and B_z_d are already defined
        R_d_w = np.zeros((3, 3))  # Initialize a 3x3 matrix

        # Fill columns of R_d_w
        R_d_w[:, 0] = np.cross(B_y_d, B_z_d)  # First column
        R_d_w[:, 1] = B_y_d                  # Second column
        R_d_w[:, 2] = B_z_d                  # Third column

        desired_quaternion = R.from_matrix(R_d_w).as_quat()

        # 	6. the attitude tracking error eR
        e_R_matrix = 0.5 * (R_d_w.T @ self.R_B_W_ - self.R_B_W_.T @ R_d_w)
        
        e_R = np.array([ e_R_matrix[2, 1], e_R_matrix[0, 2], e_R_matrix[1, 0]])
        

        # 	7.  the tracking error for the angular velocity e_omega
        omega_ref = self.r_yaw_rate * unit_z
        e_omega = self.angular_velocity_B_ - self.R_B_W_.T @ R_d_w @ omega_ref

        # 8. Moment T (Attitude tracking)
        tau = ( - np.multiply(self.attitude_gain_, e_R)  # Element-wise product
                - np.multiply(self.angular_rate_gain_, e_omega)  # Element-wise product
                + np.cross(self.angular_velocity_B_, self._inertia_matrix * self.angular_velocity_B_)
            )
        # Output the wrench
        controller_torque_thrust = np.hstack((tau, thrust))
        
        return controller_torque_thrust, desired_quaternion
        
        
