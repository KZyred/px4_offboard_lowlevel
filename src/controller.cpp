/****************************************************************************
 *
 *   Copyright (c) 2023, SMART Research Group, Saxion University of
 *   Applied Sciences.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "../include/px4_offboard_lowlevel/controller.h"

controller::controller()
{
}
/**
 * tính toán điều khiển đầu ra
 * controller_torque_thrust: phần điều khiển đầu ra sau khi xử lý bám quỹ đạo
 */
void controller::calculateControllerOutput(
	Eigen::VectorXd *controller_torque_thrust, Eigen::Quaterniond *desired_quaternion)
{
	// kiểm tra controller_torque_thrust có null không ?
	assert(controller_torque_thrust);

	controller_torque_thrust->resize(4); // 3 total moment and 1 total thrust

	// Geometric controller based on:
	// T. Lee, M. Leok and N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3),
	// " 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, USA, 2010.

	// Trajectory tracking.
	double thrust;
	Eigen::Matrix3d R_d_w;

	// 1. Compute translational tracking errors.
	// 	1.1 The tracking errors for the position
	// 	position_W_: vị trí thực tế, r_position_W_ : vị trí mong muốn
	const Eigen::Vector3d e_p =
		position_W_ - r_position_W_;
	// 	1.2 The tracking errors for the velocity
	// 	velocity_W_: vận tốc thực tế, r_velocity_W_ : vận tốc mong muốn
	const Eigen::Vector3d e_v =
		velocity_W_ - r_velocity_W_;

	// 2. caculate Iad: gia tốc mong muốn
	const Eigen::Vector3d I_a_d = -position_gain_.cwiseProduct(e_p) - velocity_gain_.cwiseProduct(e_v) + _uav_mass * _gravity * Eigen::Vector3d::UnitZ() + _uav_mass * r_acceleration_W_;

	// 3. lực đẩy T
	thrust = I_a_d.dot(R_B_W_.col(2));

	// 4. Bzd
	Eigen::Vector3d B_z_d;
	B_z_d = I_a_d;
	B_z_d.normalize();

	// 5. Calculate Desired Rotational Matrix
	//  5.1 Bxd (do Bdx và Bdz ta cần chọn cho hai thằng này không song song)
	const Eigen::Vector3d B_x_d(std::cos(r_yaw), std::sin(r_yaw), 0.0);
	//  5.1 Byd (từ Bzd and Bxd)
	Eigen::Vector3d B_y_d = B_z_d.cross(B_x_d);
	B_y_d.normalize();
	//  5.2 Rdw
	R_d_w.col(0) = B_y_d.cross(B_z_d);
	R_d_w.col(1) = B_y_d;
	R_d_w.col(2) = B_z_d;

	Eigen::Quaterniond q_temp(R_d_w);
	*desired_quaternion = q_temp;

	// Attitude tracking.
	Eigen::Vector3d tau;

	// 	6. the attitude tracking error eR
	const Eigen::Matrix3d e_R_matrix =
		0.5 * (R_d_w.transpose() * R_B_W_ - R_B_W_.transpose() * R_d_w);
	Eigen::Vector3d e_R;
	e_R << e_R_matrix(2, 1), e_R_matrix(0, 2), e_R_matrix(1, 0);

	// 	7.  the tracking error for the angular velocity e_omega
	const Eigen::Vector3d omega_ref =
		r_yaw_rate * Eigen::Vector3d::UnitZ();
	const Eigen::Vector3d e_omega = angular_velocity_B_ - R_B_W_.transpose() * R_d_w * omega_ref;

	// 8. Moment T (Attitude tracking)
	Eigen::Vector3d tau;
	tau = -attitude_gain_.cwiseProduct(e_R) - angular_rate_gain_.cwiseProduct(e_omega) + angular_velocity_B_.cross(_inertia_matrix.asDiagonal() * angular_velocity_B_);

	// Output the wrench
	*controller_torque_thrust << tau, thrust;
}
