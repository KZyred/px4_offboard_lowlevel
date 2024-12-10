import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import VehicleThrustSetpoint
from px4_msgs.msg import VehicleTorqueSetpoint

from controllers.controller import Controller


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
    
    # self.declare_parameter("config_file", rclpy.Parameter.Type.STRING)
    def loadParams(self):
        # UAV Parameters
        self.declare_parameter("uav_parameters.mass", 0.0)
        self.declare_parameter("uav_parameters.arm_length", 0.0)
        self.declare_parameter("uav_parameters.num_of_arms", 4)
        self.declare_parameter("uav_parameters.moment_constant", 0.0)
        self.declare_parameter("uav_parameters.thrust_constant", 0.0)
        self.declare_parameter("uav_parameters.max_rotor_speed", 0)
        self.declare_parameter("uav_parameters.gravity", 0.0)
        self.declare_parameter("uav_parameters.PWM_MIN", 0)
        self.declare_parameter("uav_parameters.PWM_MAX", 0)
        self.declare_parameter("uav_parameters.input_scaling", 0)
        self.declare_parameter("uav_parameters.zero_position_armed", 0)
        self.declare_parameter("uav_parameters.inertia.x", 0.0)
        self.declare_parameter("uav_parameters.inertia.y", 0.0)
        self.declare_parameter("uav_parameters.inertia.z", 0.0)
        self.declare_parameter("uav_parameters.omega_to_pwm_coefficient.x_2", 0.0)
        self.declare_parameter("uav_parameters.omega_to_pwm_coefficient.x_1", 0.0)
        self.declare_parameter("uav_parameters.omega_to_pwm_coefficient.x_0", 0.0)
        
        _uav_mass = self.get_parameter("uav_parameters.mass").value
        self._arm_length = self.get_parameter("uav_parameters.arm_length").value
        self._num_of_arms = self.get_parameter("uav_parameters.num_of_arms").value
        self._moment_constant = self.get_parameter("uav_parameters.moment_constant").value
        self._thrust_constant = self.get_parameter("uav_parameters.thrust_constant").value
        self._max_rotor_speed = self.get_parameter("uav_parameters.max_rotor_speed").value
        _gravity = self.get_parameter("uav_parameters.gravity").value
        _PWM_MIN = self.get_parameter("uav_parameters.PWM_MIN").value
        _PWM_MAX = self.get_parameter("uav_parameters.PWM_MAX").value
        _input_scaling = self.get_parameter("uav_parameters.input_scaling").value
        _zero_position_armed = self.get_parameter("uav_parameters.zero_position_armed").value
        _inertia_x = self.get_parameter("uav_parameters.inertia.x").value
        _inertia_y = self.get_parameter("uav_parameters.inertia.y").value
        _inertia_z = self.get_parameter("uav_parameters.inertia.z").value
        _omega_to_pwm_coefficient_x_2 = self.get_parameter("uav_parameters.omega_to_pwm_coefficient.x_2").value
        _omega_to_pwm_coefficient_x_1 = self.get_parameter("uav_parameters.omega_to_pwm_coefficient.x_1").value
        _omega_to_pwm_coefficient_x_0 = self.get_parameter("uav_parameters.omega_to_pwm_coefficient.x_0").value
        
        _inertia_matrix = np.array([_inertia_x, 
                                    _inertia_y, 
                                    _inertia_z])
        self._omega_to_pwm_coefficients = np.array([_omega_to_pwm_coefficient_x_2, 
                                                    _omega_to_pwm_coefficient_x_1, 
                                                    _omega_to_pwm_coefficient_x_0])
        
	    # Topics Names
        self.declare_parameter("topics_names.command_pose_topic", "default")
        self.declare_parameter("topics_names.command_traj_topic", "default")
        self.declare_parameter("topics_names.odometry_topic", "default")
        self.declare_parameter("topics_names.status_topic", "default")
        self.declare_parameter("topics_names.battery_status_topic", "default")
        self.declare_parameter("topics_names.actuator_status_topic", "default")
        self.declare_parameter("topics_names.offboard_control_topic", "default")
        self.declare_parameter("topics_names.vehicle_command_topic", "default")
        self.declare_parameter("topics_names.attitude_setpoint_topic", "default")
        self.declare_parameter("topics_names.thrust_setpoints_topic", "default")
        self.declare_parameter("topics_names.torque_setpoints_topic", "default")
        self.declare_parameter("topics_names.actuator_control_topic", "default")
        
        self.command_pose_topic_ = self.get_parameter("topics_names.command_pose_topic").value
        self.command_traj_topic_ = self.get_parameter("topics_names.command_traj_topic").value
        self.odometry_topic_ = self.get_parameter("topics_names.odometry_topic").value
        self.status_topic_ = self.get_parameter("topics_names.status_topic").value
        self.battery_status_topic_ = self.get_parameter("topics_names.battery_status_topic").value
        self.actuator_status_topic = self.get_parameter("topics_names.actuator_status_topic").value
        self.offboard_control_topic_ = self.get_parameter("topics_names.offboard_control_topic").value
        self.vehicle_command_topic_ = self.get_parameter("topics_names.vehicle_command_topic").value
        self.attitude_setpoint_topic_ = self.get_parameter("topics_names.attitude_setpoint_topic").value
        self.thrust_setpoint_topic_ = self.get_parameter("topics_names.thrust_setpoints_topic").value
        self.torque_setpoint_topic_ = self.get_parameter("topics_names.torque_setpoints_topic").value
        self.actuator_control_topic_ = self.get_parameter("topics_names.actuator_control_topic").value
        
        # Load logic switches
        self.declare_parameter("sitl_mode", True)
        self.declare_parameter("control_mode", 1)

        self.in_sitl_mode_ = self.get_parameter("sitl_mode").value
        self.control_mode_ = self.get_parameter("control_mode").value
        
        # Controller gains
        self.declare_parameter("control_gains.K_p_x", 0.0)
        self.declare_parameter("control_gains.K_p_y", 0.0)
        self.declare_parameter("control_gains.K_p_z", 0.0)
        self.declare_parameter("control_gains.K_v_x", 0.0)
        self.declare_parameter("control_gains.K_v_y", 0.0)
        self.declare_parameter("control_gains.K_v_z", 0.0)
        self.declare_parameter("control_gains.K_R_x", 0.0)
        self.declare_parameter("control_gains.K_R_y", 0.0)
        self.declare_parameter("control_gains.K_R_z", 0.0)
        self.declare_parameter("control_gains.K_w_x", 0.0)
        self.declare_parameter("control_gains.K_w_y", 0.0)
        self.declare_parameter("control_gains.K_w_z", 0.0)

        position_gain_ = np.array([self.get_parameter("control_gains.K_p_x"), 
                                    self.get_parameter("control_gains.K_p_y"), 
                                    self.get_parameter("control_gains.K_p_z")])

        velocity_gain_ = np.array([self.get_parameter("control_gains.K_v_x"), 
                                    self.get_parameter("control_gains.K_v_y"), 
                                    self.get_parameter("control_gains.K_v_z")])
        
        attitude_gain_ = np.array([self.get_parameter("control_gains.K_R_x"), 
                                    self.get_parameter("control_gains.K_R_y"), 
                                    self.get_parameter("control_gains.K_R_z")])
        
        ang_vel_gain_ = np.array([self.get_parameter("control_gains.K_w_x"), 
                                    self.get_parameter("control_gains.K_w_y"), 
                                    self.get_parameter("control_gains.K_w_z")])

        # pass the UAV Parameters and controller gains to the controller
        Controller.set_uav_mass(_uav_mass)
        Controller.set_inertia_matrix(_inertia_matrix)
        Controller.set_gravity(_gravity)
        Controller.set_k_position_gain(position_gain_)
        Controller.set_k_velocity_gain(velocity_gain_)
        Controller.set_k_attitude_gain(attitude_gain_)
        Controller.set_k_angularRate_gain(ang_vel_gain_)
