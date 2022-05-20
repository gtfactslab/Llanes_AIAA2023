#pragma once

#include "Eigen/Dense"
#include "OsqpEigen/OsqpEigen.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <queue>
#include "mav_util.h"

namespace asif
{

//typedef struct {
//	double x;
//	double y;
//	double z;
//	double vx;
//	double vy;
//	double vz;
//	double phi;
//	double theta;
//	double psi;
//} quad_states_s;
//
//typedef struct {
//	double thrust;
//	double roll_rate;
//} control_s;

enum {
	SOLVE_SUCCESSFUL = 1,
	SOLVE_ERROR = 2,
	INITIALIZE_PROBLEM_ERROR = 5,
	SET_DATA_ERROR = 10,
};


class MavAsif: public rclcpp::Node {
public:
	MavAsif()
			: Node("mav_asif_node")
	{
		osqp_solver_.settings()->setWarmStart(true);
		osqp_solver_.data()->setNumberOfVariables(NUM_CONTROL_INPUTS);
		Eigen::SparseMatrix<double> control_size_identity_matrix(NUM_CONTROL_INPUTS,NUM_CONTROL_INPUTS);
		control_size_identity_matrix.setIdentity();
		osqp_solver_.data()->setHessianMatrix(control_size_identity_matrix);
		try {
			this->declare_parameter("mass");
			this->declare_parameter("gravity");
			this->declare_parameter("backup_controller.kp");
			this->declare_parameter("backup_controller.kv");
			this->declare_parameter("backup_controller.roll_kp");
			this->declare_parameter("backup_controller.pitch_kp");
			this->declare_parameter("backup_controller.yaw_kp");
			this->declare_parameter("thrust_model.mav_max_thrust");
			this->declare_parameter("thrust_model.mav_min_thrust");
			this->declare_parameter("asif_alpha");

			this->get_parameter("mass", mass_);
			this->get_parameter("gravity", gravity_);
			this->get_parameter("backup_controller.kp", kp_);
			this->get_parameter("backup_controller.kv", kv_);
			this->get_parameter("backup_controller.roll_kp", roll_kp_);
			this->get_parameter("backup_controller.pitch_kp", pitch_kp_);
			this->get_parameter("backup_controller.yaw_kp", yaw_kp_);
			this->get_parameter("asif_alpha", asif_alpha_);

		} catch (rclcpp::ParameterTypeException &excp) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Parameter type exception caught");
			rclcpp::shutdown(nullptr, "ASIF ERROR: Parameter type exception caught on initialization");
		}
	};

	~MavAsif() override = default;

	int solve(const px4_msgs::msg::VehicleOdometry &mav,
		      mavControl &mav_control,
		      const Vector3dMap& p_obs,
		      const Eigen::VectorXd& r_obs,
		      const size_t& num_obs,
		      std::chrono::microseconds &computation_time);

private:
	double gravity_;
	double mass_;
	double kp_;
	double kv_;
	double roll_kp_;
	double pitch_kp_;
	double yaw_kp_;
	double asif_alpha_;

	double quad_max_thrust_body_;
    double quad_min_thrust_body_;

	Eigen::Matrix<double, NUM_STATES, 1> fx_;
	Eigen::Matrix<double, NUM_STATES, NUM_CONTROL_INPUTS> gx_;
	OsqpEigen::Solver osqp_solver_;
	Eigen::MatrixXd DGamma_;
	Eigen::SparseMatrix<double> constraints_matrix_;
	Eigen::VectorXd upper_bound_;
	control_vector_t qp_gradient_;
	control_vector_t asif_control_;

	double dt_backup_;
	int backup_horizon_; // number of backup steps of dt_backup_ backup_time = backup_horizon_*dt_backup_

	void update_state(state_vector_t& x, const control_vector_t& u) const;
	control_vector_t backup_control(const state_vector_t& x) const;
	static Eigen::VectorXd barrier_function(const state_vector_t& x, const Vector3dMap& p_obs, const Eigen::VectorXd& r_obs, const size_t& num_obs);
	static VectorStateMap gradient_barrier_function(const VectorStateMap& x, const Vector3dMap& p_obs, const size_t& num_obs);
	state_jacobian_t jacobian(const state_vector_t& x, const control_vector_t& u, const position_t& p_des) const;

}; //class MavAsif

} //namespace asif