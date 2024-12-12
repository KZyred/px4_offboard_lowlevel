import numpy as np
from scipy.spatial.transform import Rotation as R

def rotate_vector_fromTo_ENU_NED(vec_in):
    # TODO: handle NED->ENU transformation 
    # NED (X North, Y East, Z Down) & ENU (X East, Y North, Z Up)
    vec_out = np.array([vec_in[1], vec_in[0], -vec_in[2]])
    return vec_out

def rotate_vector_fromTo_FRD_FLU(vec_in):
    # FRD (X Forward, Y Right, Z Down) & FLU (X Forward, Y Left, Z Up)
    vec_out = np.array([vec_in[0], -vec_in[1], -vec_in[2]])
    return vec_out

def rotate_quaternion_fromTo_ENU_NED(quat_in):
    """
    Rotate a quaternion between ENU and NED frames and also between aircraft and base_link frames.

    Parameters:
        quat_in: Input quaternion as a numpy array [w, x, y, z]

    Returns:
        Rotated quaternion as a numpy array [x, y, z, w]
    """
    # Static quaternion needed for rotating between ENU and NED frames
    euler_1 = np.array([np.pi, 0.0, np.pi/2])  # Roll, Pitch, Yaw in radians
    NED_ENU_Q = R.from_euler('xyz', euler_1).as_quat()  # Convert to quaternion [x, y, z, w]


    # Static quaternion needed for rotating between aircraft and base_link frames
    euler_2 = np.array([np.pi, 0.0, 0.0])  # Roll, Pitch, Yaw in radians
    AIRCRAFT_BASELINK_Q = R.from_euler('xyz', euler_2).as_quat()  # Convert to quaternion [x, y, z, w]

    # Convert input quaternion to scipy Rotation
    quat_in = np.roll(quat_in, -1)
    quat_in_rot = R.from_quat(quat_in)  # Input quaternion as [x, y, z, w]

    # Apply the rotations
    ned_enu_rot = R.from_quat(NED_ENU_Q)
    aircraft_baselink_rot = R.from_quat(AIRCRAFT_BASELINK_Q)

    # Final quaternion
    rotated_quat = ((ned_enu_rot * quat_in_rot) * aircraft_baselink_rot).as_quat()
    return rotated_quat

def eigen_odometry_from_PX4Msg(msg):
        # position_W: Vector vị trí trong hệ quy chiếu World
        position_W = rotate_vector_fromTo_ENU_NED(np.array([msg.position[0], msg.position[1], msg.position[2]]))
        # orientation_B_W: Quaternion biểu diễn hướng của thân (Body) đối với hệ quy chiếu World
        orientation_B_W = rotate_quaternion_fromTo_ENU_NED(np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]]))
        # velocity_B: Vector vận tốc trong hệ quy chiếu của thân (Body)
        velocity_B = rotate_vector_fromTo_ENU_NED(np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]]))
        # angular_velocity_B: Vector vận tốc góc trong hệ quy chiếu của thân (Body)
        angular_velocity_B = rotate_vector_fromTo_FRD_FLU(np.array([msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]]))
        return position_W, orientation_B_W, velocity_B, angular_velocity_B

def eigen_trajectoryPoint_from_msg(msg, position_W, orientation_W_B, velocity_W, angular_velocity_W, acceleration_W):
    if not msg.transforms : #.empty()) #???
        return
    
    # position_W: Vector vị trí trong hệ quy chiếu World
    position_W = np.array([msg.transforms[0].translation.x, msg.transforms[0].translation.y, msg.transforms[0].translation.z])
    # orientation_W_B
    orientation_W_B = np.array([msg.transforms[0].rotation.w, msg.transforms[0].rotation.x, msg.transforms[0].rotation.y, msg.transforms[0].rotation.z])
    
    # velocity_W
    # angular_velocity_W
    if np.size(msg.velocities) > 0:
        velocity_W = np.array([msg.velocities[0].linear.x, msg.velocities[0].linear.y, msg.velocities[0].linear.z])
        angular_velocity_W = np.array([msg.velocities[0].angular.x, msg.velocities[0].angular.y, msg.velocities[0].angular.z])
    else:
    # empty data received
        velocity_W = np.zeros(3)
        angular_velocity_W[:] = np.zeros(3)
        
    # acceleration_W
    if np.size(msg.accelerations) > 0:
        acceleration_W = np.array([msg.accelerations[0].linear.x, msg.accelerations[0].linear.y, msg.accelerations[0].linear.z])
    else:
    # empty data received
        acceleration_W = np.zeros(3)
        
    return position_W, orientation_W_B, velocity_W, angular_velocity_W, acceleration_W

# chuyển đổi các thông tin từ thông điệp ROS (geometry_msgs::msg::PoseStamped) sang các kiểu dữ liệu Eigen.
def eigen_trajectoryPoint_from_poseMsg( msg):
    # position_W: Vector vị trí trong hệ quy chiếu World
    position_W = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    # orientation_W_B
    orientation_W_B = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
    return position_W, orientation_W_B