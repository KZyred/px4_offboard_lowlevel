import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from scipy.spatial.transform import Rotation as R

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

from px4_offboard_lowlevel.controllers.controller import Controller
import px4_offboard_lowlevel.convert_unit as Convert


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.current_status_ = None
    
        self.load_params()
        # tính toán một số ma trận chuyển đổi tốc độ quay rotor <-> torque, thrust
        self.compute_ControlAllocation_and_ActuatorEffect_matrices()
        
        # Defining the compatible ROS 2 predefined QoS for PX4 topics
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.vehicle_odometry_sub_ = self.create_subscription(
            VehicleOdometry,
            self.odometry_topic_,
            self.vehicle_odometryCallback,
            qos_profile)
        
        self.vehicle_status_sub_ = self.create_subscription(
            VehicleStatus,
            self.status_topic_,
            self.vehicleStatusCallback,
            qos_profile)
        
        self.command_pose_sub_ = self.create_subscription(
            PoseStamped,
            self.command_pose_topic_,
            self.commandPoseCallback,
            10)
        	
        # Publishers
        self.attitude_setpoint_publisher_ = self.create_publisher(VehicleAttitudeSetpoint, self.attitude_setpoint_topic_, 10)
        self.actuator_motors_publisher_ = self.create_publisher(ActuatorMotors, self.actuator_control_topic_, 10)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, self.offboard_control_topic_, 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, self.vehicle_command_topic_, 10)
        self.thrust_setpoint_publisher_ = self.create_publisher(VehicleThrustSetpoint, self.thrust_setpoint_topic_, 10)
        self.torque_setpoint_publisher_ = self.create_publisher(VehicleTorqueSetpoint, self.torque_setpoint_topic_, 10)

        # Parameters subscriber
    	# callback_handle_ = this->add_on_set_parameters_callback(
		# std::bind(&ControllerNode::parametersCallback, this, std::placeholders::_1))
     
        # self.callback_handle_ = self.create_service(SetPose, '/set_pose', self.add_on_set_parameters_callback)
        
        # Timers
        # 1. Chỉnh off board control model: (hiện tại đang thao tác với 3 model 1.attitude 2.thrust_and_torque	3.direct_actuator)        
        offboard_period = 0.33  # seconds        
        offboardTimer = self.create_timer(offboard_period, self.publish_offboardControlMode_msg)
        
        
        # 2. Điều khiển chính
        controller_period = 0.01
        controllerTimer = self.create_timer(controller_period, self.update_controller_output)
        
    # # Service: xác nhận đã chuyển control_mode
    # rcl_interfaces::msg::SetParametersResult ControllerNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    #     rcl_interfaces::msg::SetParametersResult result
    #     result.successful = true
    #     result.reason = "success"
    #     // print info about the changed parameter
    #     for (const auto &param : parameters)
    #     {
    #         RCLCPP_INFO(this->get_logger(), "Parameter %s has changed to [%s]", param.get_name().c_str(), param.value_to_string().c_str())
    #         if (param.get_name() == "control_mode")
    #         {
    #             control_mode_ = param.as_int()
    #         }
    #     }
    #     return result    

    # def add_set_pos_callback(self, request, response):
    #     self.setpoint_position[0] = request.pose.position.x
    #     self.setpoint_position[1] = request.pose.position.y
    #     self.setpoint_position[2] = request.pose.position.z

    #     return response
    
    # self.declare_parameter("config_file", rclpy.Parameter.Type.STRING)
    def load_params(self):
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
        self._PWM_MIN = self.get_parameter("uav_parameters.PWM_MIN").value
        self._PWM_MAX = self.get_parameter("uav_parameters.PWM_MAX").value
        self._input_scaling = self.get_parameter("uav_parameters.input_scaling").value
        self._zero_position_armed = self.get_parameter("uav_parameters.zero_position_armed").value
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

        position_gain_ = np.array([self.get_parameter("control_gains.K_p_x").value, 
                                    self.get_parameter("control_gains.K_p_y").value, 
                                    self.get_parameter("control_gains.K_p_z").value])

        velocity_gain_ = np.array([self.get_parameter("control_gains.K_v_x").value, 
                                    self.get_parameter("control_gains.K_v_y").value, 
                                    self.get_parameter("control_gains.K_v_z").value])
        
        attitude_gain_ = np.array([self.get_parameter("control_gains.K_R_x").value, 
                                    self.get_parameter("control_gains.K_R_y").value, 
                                    self.get_parameter("control_gains.K_R_z").value])
        
        ang_vel_gain_ = np.array([self.get_parameter("control_gains.K_w_x").value, 
                                    self.get_parameter("control_gains.K_w_y").value, 
                                    self.get_parameter("control_gains.K_w_z").value])

        # pass the UAV Parameters and controller gains to the controller
        self.controller = Controller()
        self.controller.set_uav_mass(_uav_mass)
        self.controller.set_inertia_matrix(_inertia_matrix)
        self.controller.set_gravity(_gravity)
        self.controller.set_k_position_gain(position_gain_)
        self.controller.set_k_velocity_gain(velocity_gain_)
        self.controller.set_k_attitude_gain(attitude_gain_)
        self.controller.set_k_angularRate_gain(ang_vel_gain_)

    def compute_ControlAllocation_and_ActuatorEffect_matrices(self):
        '''
        1.1 rotor_velocities_to_torques_and_thrust: lực/... nhân ma trận này -> momen or lực đẩy tổng
        # Ma trận này xác định mối quan hệ giữa tốc độ quay của rotor và lực đẩy cùng với mô men mà nó tạo ra.
        # 3 hàng đầu: momen quanh roll, pitch, yaw
        # hàng cuối: biểu diễn tổng lực đẩy.
        #
        # 1.2 torques_and_thrust_to_rotor_velocities_: chuyển ngược lại thôi
        #
        # 2. mixing_matrix: Một ma trận cố định định nghĩa sự phân phối của lực đẩy và mô men cho các động cơ,
        # chuyển các lực và mô men mong muốn thành các lệnh cụ thể cho mỗi động cơ.
        #
        # 3. throttles_to_normalized_torques_and_thrust_:
        # Ma trận này ánh xạ các đầu vào điều khiển đã chuẩn hóa (throttle) thành các mô men và lực đẩy tương ứng,
        # được thiết lập cứng cho một cấu hình cụ thể để tránh sai số từ tính toán nghịch đảo giả.
        '''
        k_deg_to_rad = np.pi / 180.0
        
        # TODO: for other _num_of_arms
        rotor_velocities_to_torques_and_thrust = 0
        # mixing_matrix = 0
        
        if (self._num_of_arms == 4):
            # lượng động cơ bằng 4
            # động cơ 4 cánh -> cấu hình chữ X
            kS = np.sin(k_deg_to_rad * 45)
            #
            rotor_velocities_to_torques_and_thrust = np.array([[-kS, kS, kS, -kS],
                                                              [-kS, kS, -kS, kS],
                                                              [-1, -1, 1, 1],
                                                              [1, 1, 1, 1]])
            # tính bằng tay thay vì code (cái này là tiêu chuẩn nên ko lo)
            # -> (cái này là tiêu chuẩn nên ko lo, à không phụ thuộc vào k thurst và k moment nữa)
            # mixing_matrix = np.array([[-0.495384, -0.707107, -0.765306, 1.0],
            #                         [0.495384, 0.707107, -1.0, 1.0],
            #                         [0.495384, -0.707107, 0.765306, 1.0],
            #                         [-0.495384, 0.707107, 1.0, 1.0]])
            #
            self.torques_and_thrust_to_rotor_velocities_ = np.zeros((4,4))
            # Hardcoded because the calculation of pesudo-inverse is not accurate
            # việc nghịch đảo để tính ra có vẻ không ổn -> họ tính bằng tay
            # -> (cái này là tiêu chuẩn nên ko lo, à không phụ thuộc vào k thurst và k moment nữa)
            self.throttles_to_normalized_torques_and_thrust_ = np.array([[-0.5718, 0.4376, 0.5718, -0.4376],
                                                              [-0.3536, 0.3536, -0.3536, 0.3536],
                                                              [-0.2832, -0.2832, 0.2832, 0.2832],
                                                              [0.2500, 0.2500, 0.2500, 0.2500]])
        else:
            self.get_logger().info("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices\n")
        # Calculate Control allocation matrix: Wrench to Rotational velocities
        # Helper diagonal matrix.   
        k = np.array([self._thrust_constant * self._arm_length, 
                     self._thrust_constant * self._arm_length, 
                     self._moment_constant * self._thrust_constant, 
                     self._thrust_constant])
        
        rotor_velocities_to_torques_and_thrust = np.diag(k) @ rotor_velocities_to_torques_and_thrust
        self.get_logger().info("rotor_velocities_to_torques_and_thrust = %s \n" % (rotor_velocities_to_torques_and_thrust))
        #
        self.torques_and_thrust_to_rotor_velocities_[:,:] = 0
        self.torques_and_thrust_to_rotor_velocities_ = np.linalg.pinv(rotor_velocities_to_torques_and_thrust)
        
        self.get_logger().info("rotor_velocities_to_torques_and_thrust = %s \n" % (rotor_velocities_to_torques_and_thrust))
        self.get_logger().info("torques_and_thrust_to_rotor_velocities = %s \n" % self.torques_and_thrust_to_rotor_velocities_)
        self.get_logger().info("throttles_to_normalized_torques_and_thrust = %s \n" % (self.throttles_to_normalized_torques_and_thrust_))
        
    # def px4_inverse(self, normalized_torque_and_thrust, Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench)
    def px4_inverse(self, normalized_torque_and_thrust, throttles, wrench):
        omega = 0
        pwm = 0
        ones_temp = 0
        normalized_torque_and_thrust = np.zeros(4)
        
        if (self._num_of_arms == 4):
            omega = np.zeros(4)
            pwm = np.zeros(4)
            throttles = np.zeros(4)
            ones_temp = np.ones(4)
        else:
            self.get_logger().info("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices\n")
            
        
        # Control allocation: Wrench to Rotational velocities (omega)
        omega = self.torques_and_thrust_to_rotor_velocities_ @ wrench
        omega[omega <= 0] = 0.0
        omega = np.sqrt(omega)
        
        pwm = (self._omega_to_pwm_coefficients(0) * omega @ omega) + (self._omega_to_pwm_coefficients(1) * omega) + (self._omega_to_pwm_coefficients(2) * ones_temp)
        throttles = (pwm - (self._PWM_MIN * ones_temp))
        throttles /= (self._PWM_MAX - self._PWM_MIN)
        # Inverse Mixing: throttles to normalized torques and thrust
        normalized_torque_and_thrust = self.throttles_to_normalized_torques_and_thrust_ @ throttles
        
        return normalized_torque_and_thrust, throttles
    
    
    def px4_inverse_sitl(self, normalized_torque_and_thrust, throttles, wrench):
        omega = 0
        normalized_torque_and_thrust = np.zeros(4)
        ones_temp = 0
        if self._num_of_arms == 6:
            omega = np.zeros(6)
            throttles = np.zeros(6)
            ones_temp = np.ones(6)
        elif self._num_of_arms == 4:
            omega = np.zeros(4)
            throttles = np.zeros(4)
            ones_temp = np.ones(4)
        elif self._num_of_arms == 44:
            omega = np.zeros(8)
            throttles = np.zeros(8)
            ones_temp = np.ones(8)
        else:
            self.get_logger().info("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices\n")
            
        # Control allocation: Wrench to Rotational velocities (omega)
        omega = np.dot(self.torques_and_thrust_to_rotor_velocities_, wrench)
        omega = np.sqrt(omega)
        throttles = (omega - (self._zero_position_armed * ones_temp))
        throttles /= (self._input_scaling)
        # Inverse Mixing: throttles to normalized torques and thrust
        normalized_torque_and_thrust = self.throttles_to_normalized_torques_and_thrust_ @ throttles
        
        return normalized_torque_and_thrust, throttles
    
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")
    
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command send")
        
    def publish_vehicle_command(self, command, param1 = 0.0, param2 = 0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        # self.vehicle_command_publisher_.publish(msg)
    
    def publish_offboardControlMode_msg(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.body_rate = False
        match self.control_mode_:
            case 1:
                offboard_msg.attitude = True
                offboard_msg.thrust_and_torque = False
                offboard_msg.direct_actuator = False
            case 2:
                offboard_msg.attitude = False
                offboard_msg.thrust_and_torque = True
                offboard_msg.direct_actuator = False
            case 3:
                offboard_msg.attitude = False
                offboard_msg.thrust_and_torque = False
                offboard_msg.direct_actuator = True
            case _:
                offboard_msg.attitude = True
                offboard_msg.thrust_and_torque = False
                offboard_msg.direct_actuator = False
                
                
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(offboard_msg)
        self.get_logger().info("Offboard enabled", once=True)
        
    def commandPoseCallback(self, pose_msg):
        # When a command is received
        # initialize vectors
        
        position, orientation = Convert.eigen_trajectoryPoint_from_poseMsg(pose_msg)
        self.get_logger().info("Controller got first command message.", once=True)
        r_R_B_W_, r_yaw = self.controller.set_trajectoryPoint(position, orientation); # Send the command to controller_ obj
        
        
    # def commandTrajectoryCallback(self, traj_msg):
    #     # When a command is received
    #     # initialize vectors
    #     position, orientation, velocity, angular_velocity, acceleration = Convert.eigen_trajectoryPoint_from_msg(traj_msg, position, orientation, velocity, angular_velocity, acceleration)
    #     self.controller.set_trajectoryPoint(position, velocity, acceleration, orientation, angular_velocity)
    #     self.get_logger().info("Controller got first command message.", once=True)
        
    def vehicle_odometryCallback(self, odom_msg):
        #  Debug message
        self.get_logger().info("Controller got first odometry message.", once=True)
        
        position, orientation, velocity, angular_velocity = Convert.eigen_odometry_from_PX4Msg(odom_msg)
        self.controller.set_odometry(position, orientation, velocity, angular_velocity)

    # check trạng thái
    def vehicleStatusCallback(self, status_msg):
        self.current_status_ = status_msg
        
        if self.current_status_.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info("ARMED - vehicle_status_msg.", once=True)
        else:
            self.get_logger().info("NOT ARMED - vehicle_status_msg.")
        
        if self.current_status_.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info("OFFBOARD - vehicle_status_msg.", once=True)
        else:
            self.get_logger().info("NOT OFFBOARD - vehicle_status_msg.")
    
    def publish_actuatorMotors_msg(self, throttles):
        # Lockstep should be disabled from PX4 and from the model.sdf file
        # direct motor throttles control
        # Prepare msg
        actuator_motors_msg = ActuatorMotors()
        
        thrust_command = np.zeros(12, dtype=np.float32)
        
        thrust_command[0] = throttles[0]
        thrust_command[1] = throttles[1]

        thrust_command[2] = throttles[2]
        thrust_command[3] = throttles[3]

        actuator_motors_msg.control = thrust_command.flatten()
        
        actuator_motors_msg.reversible_flags = 0
        actuator_motors_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        actuator_motors_msg.timestamp_sample = actuator_motors_msg.timestamp
        self.actuator_motors_publisher_.publish(actuator_motors_msg)
        
    def publish_thrust_torque_msg(self, controller_output):
        # Lockstep should be disabled from PX4 and from the model.sdf file
        # Prepare msgs
        thrust_sp_msg = VehicleThrustSetpoint()
        thrust_sp_msg.timestamp_sample = int(Clock().now().nanoseconds / 1000)
        thrust_sp_msg.timestamp = thrust_sp_msg.timestamp_sample
        
        torque_sp_msg = VehicleTorqueSetpoint()
        torque_sp_msg.timestamp_sample = thrust_sp_msg.timestamp_sample
        torque_sp_msg.timestamp = thrust_sp_msg.timestamp_sample
        
        # Fill thrust setpoint msg
        thrust_sp_msg.xyz[0] = 0.0
        thrust_sp_msg.xyz[1] = 0.0
        if controller_output[3] > 0.1:
            thrust_sp_msg.xyz[2] = -controller_output[3] # DO NOT FORGET THE MINUS SIGN (body NED frame)
        else:
            thrust_sp_msg.xyz[2] = -0.1
            
        # Rotate torque setpoints from FLU to FRD and fill the msg
        rotated_torque_sp = Convert.rotate_vector_fromTo_FRD_FLU(np.array[controller_output[0], controller_output[1], controller_output[2]])
        torque_sp_msg.xyz[0] = rotated_torque_sp[0]
        torque_sp_msg.xyz[1] = rotated_torque_sp[1]
        torque_sp_msg.xyz[2] = rotated_torque_sp[2]

        # Publish msgs
        self.thrust_setpoint_publisher_.publish(thrust_sp_msg)
        self.torque_setpoint_publisher_.publish(torque_sp_msg)



    def publish_attitude_setpoint_msg(self, controller_output, desired_quaternion):
        attitude_setpoint_msg = VehicleAttitudeSetpoint()
        attitude_setpoint_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        
        desired_quaternion = np.roll(desired_quaternion, 1)
        rotated_quat = Convert.rotate_quaternion_fromTo_ENU_NED(desired_quaternion)
  
        # [w x y z] PX4 ~ [x y z w] np.array
        attitude_setpoint_msg.q_d[0] = rotated_quat[3]
        attitude_setpoint_msg.q_d[1] = rotated_quat[0]
        attitude_setpoint_msg.q_d[2] = rotated_quat[1]
        attitude_setpoint_msg.q_d[3] = rotated_quat[2]

        if controller_output[3] > 0.1:
            attitude_setpoint_msg.thrust_body[0] = 0.0
            attitude_setpoint_msg.thrust_body[1] = 0.0
            attitude_setpoint_msg.thrust_body[2] = -controller_output[3]; # DO NOT FORGET THE MINUS SIGN (body NED frame)
        else:
            attitude_setpoint_msg.thrust_body[2] = -0.1

        self.attitude_setpoint_publisher_.publish(attitude_setpoint_msg)
        

    def update_controller_output(self):
        if self.current_status_ is None:
            return
        
        #  calculate controller output
        controller_output, desired_quaternion = self.controller.calculate_controller_output()

        # Normalize the controller output
        normalized_torque_thrust = np.zeros(4)
        throttles = np.zeros(3)
        if self.in_sitl_mode_:
            normalized_torque_thrust, throttles = self.px4_inverse_sitl(normalized_torque_thrust, throttles, controller_output)
        else:
            normalized_torque_thrust, throttles = self.px4_inverse(normalized_torque_thrust, throttles, controller_output)
        
        # Publish the controller output
        if self.current_status_.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            match self.control_mode_:
                case 1:
                    self.publish_attitude_setpoint_msg(normalized_torque_thrust, desired_quaternion)
                case 2:
                    self.publish_thrust_torque_msg(normalized_torque_thrust)
                case 3:
                    self.publish_actuatorMotors_msg(throttles)
                case _:
                    self.publish_attitude_setpoint_msg(normalized_torque_thrust, desired_quaternion)
                    
                    
def main(args=None):
    # rclpy.init(args=sys.argv)
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
