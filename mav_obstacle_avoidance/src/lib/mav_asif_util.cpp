#include "mav_asif_util.hpp"
#include <chrono>
#include <iostream>
#include "mav_util.h"

using namespace asif;
using namespace Eigen;
using namespace std;

int MavAsif::solve(const px4_msgs::msg::VehicleOdometry &mav,
                   mavControl &mav_control,
                   const Vector3dMap& p_obs,
                   const VectorXd& r_obs,
                   const size_t& num_obs,
                   std::chrono::microseconds &computation_time) {

    const std::chrono::time_point<std::chrono::steady_clock> start =
            std::chrono::steady_clock::now();



    Eigen::Vector3d orientation_rpy = quaternion_to_rpy_wrap(mav.q[0],mav.q[1],mav.q[2],mav.q[3]);

	double phi = orientation_rpy(0);
	double theta = orientation_rpy(1);
	double psi = orientation_rpy(2);

    double cphi = cos(phi);
	double sphi = sin(phi);
	double ctheta = cos(theta);
	double stheta = sin(theta);
	double cpsi = cos(psi);
	double spsi = sin(psi);
	double tantheta = tan(theta);

	fx_.setZero();
	fx_(0) = mav.vx;
	fx_(1) = mav.vy;
	fx_(2) = mav.vz;
	fx_(5) = -gravity_;

    gx_.setZero();
    gx_(3, 0) = (sphi*spsi + cphi*cpsi*stheta);
    gx_(4, 0) = -(cpsi*sphi - cphi*spsi*stheta);
    gx_(5, 0) = cphi*ctheta;
    gx_(6,1) = 1.0;
	gx_(6,2) = sphi*tantheta;
	gx_(6,3) = cphi*tantheta;
	gx_(7,2) = cphi;
	gx_(7,3) = -sphi;
	gx_(8,2) = sphi/ctheta;
	gx_(8,3) = cphi/ctheta;

    state_vector_t x;
    x(0) = mav.x;
    x(1) = mav.y;
    x(2) = mav.z;
    x(3) = mav.vx;
    x(4) = mav.vy;
    x(5) = mav.vz;
    x(6) = phi;
    x(7) = theta;
    x(8) = psi;

	position_t p_des(mav.x,mav.y,mav.z);
	control_vector_t control;

    // Backup set max of min
	VectorXd hx_min = VectorXd::Constant(num_obs,1,INFINITY);
    VectorXd hx;
	VectorStateMap worst_state;
	state_sensitivity_t sensitivity_matrix;
	sensitivity_matrix.setIdentity();

	MatrixStateSensitivityMap sensitivity_matrix_obs_map;
	for(auto & itr : sensitivity_matrix_obs_map){
		itr.second = sensitivity_matrix;
	}

    for (int i = 0; i <= backup_horizon_; i++) {

		std::cout << i << ", ";
	    hx = barrier_function(x,p_obs,r_obs,num_obs);
		for (size_t j = 0; j < num_obs; j++) {
			if (hx(j) < hx_min(j)) {
				hx_min(j) = hx(j);
				worst_state.insert({j, x});
				sensitivity_matrix_obs_map.at(j) = sensitivity_matrix;
			}
		}

        if (i != backup_horizon_) {
        	control = backup_control(x);
	        sensitivity_matrix = jacobian(x,control,p_des) * sensitivity_matrix * dt_backup_ + sensitivity_matrix;
	        update_state(x,control);
        }
    }

	VectorStateMap dhdx_map = gradient_barrier_function(worst_state, p_obs, num_obs);

	DGamma_.resize(num_obs, NUM_STATES);

	for (size_t j = 0; j < num_obs; j++) {
		DGamma_.row(j) = (dhdx_map.at(j).transpose() * sensitivity_matrix_obs_map.at(j)).sparseView();
	}

	constraints_matrix_.resize(num_obs, NUM_CONTROL_INPUTS);
	constraints_matrix_ = (-DGamma_ * gx_).sparseView();

	upper_bound_.resize(num_obs);
	upper_bound_ = DGamma_ * fx_ + asif_alpha_ * hx;

	osqp_solver_.data()->setNumberOfConstraints(num_obs);

	qp_gradient_ << -mav_control.thrust,
					-mav_control.roll_rate,
					-mav_control.pitch_rate,
					-mav_control.yaw_rate;

	if(!osqp_solver_.data()->setGradient(qp_gradient_)) return SET_DATA_ERROR;
	if(!osqp_solver_.data()->setLinearConstraintsMatrix(constraints_matrix_)) return SET_DATA_ERROR;
	if(!osqp_solver_.data()->setUpperBound(upper_bound_)) return SET_DATA_ERROR;

	if(!osqp_solver_.initSolver()) return INITIALIZE_PROBLEM_ERROR;

	if(osqp_solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return SOLVE_ERROR;

	asif_control_ = osqp_solver_.getSolution();
	mav_control.thrust = asif_control_(0) * mass_;
	mav_control.roll_rate = asif_control_(1);
	mav_control.pitch_rate = asif_control_(2);
	mav_control.yaw_rate = asif_control_(3);

    const auto end = std::chrono::steady_clock::now();

	computation_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    return SOLVE_SUCCESSFUL;
}

void MavAsif::update_state(state_vector_t& x, const control_vector_t& u) const {
	double vx = x(3);
	double vy = x(4);
	double vz = x(5);
	double phi = x(6);
	double theta = x(7);
	double psi = x(8);

	state_vector_t dxdt;
	dxdt <<  vx,
             vy,
             vz,
			 u(0)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)),
             -u(0)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)),
			 u(0)*cos(phi)*cos(theta) - gravity_,
			 u(1) + u(3)*cos(phi)*tan(theta) + u(2)*sin(phi)*tan(theta),
			 u(2)*cos(phi) - u(3)*sin(phi),
			 (u(3)*cos(phi))/cos(theta) + (u(2)*sin(phi))/cos(theta);

	x = x + dxdt*dt_backup_;
}

control_vector_t MavAsif::backup_control(const state_vector_t& x) const{
	double px = x(0);
	double py = x(1);
	double pz = x(2);
	double vx = x(3);
	double vy = x(4);
	double vz = x(5);
	double phi = x(6);
	double theta = x(7);
	double psi = x(8);

	position_t p_des(px,py,pz);

	double ax = kp_ * (p_des.x() - px) - kv_ * vx;
	double ay = kp_ * (p_des.y() - py) - kv_ * vy;
	double az = kp_ * (p_des.z() - pz) + gravity_ - kv_ * vz;

	Vector3d acc_vec(ax,ay,az);
	double acc_mag = acc_vec.norm();
	Vector3d acc_dir = acc_vec.normalized();

	Vector3d interm_force_frame1 = dcm(0,0,psi) * acc_dir;
	double theta_cmd = atan(interm_force_frame1(1)/(interm_force_frame1(3)));

	Vector3d interm_force_frame2 = dcm(0,theta_cmd,0) * interm_force_frame1;
	double phi_cmd = atan(-interm_force_frame2(2)/(interm_force_frame2(3)));

	double wx = -roll_kp_ * (phi - phi_cmd);
	double wy = -pitch_kp_ * (theta - theta_cmd);
	double wz = -yaw_kp_ * psi;

	control_vector_t control(acc_mag, wx, wy, wz);
	return control;
}

VectorXd MavAsif::barrier_function(const state_vector_t& x, const Vector3dMap& p_obs, const Eigen::VectorXd& r_obs, const size_t& num_obs) {
	VectorXd hx(num_obs);
	Vector3d p_tild;
	Vector3d p_quad(x(0), x(1), x(2)) ;

	for (size_t i = 0; i < num_obs; i++) {
		p_tild = p_quad - p_obs.at(i);
		hx(i) = p_tild.transpose() * Matrix3d::Identity() * p_tild - r_obs(i);
	}
	return hx;
}

VectorStateMap MavAsif::gradient_barrier_function(const VectorStateMap& x, const Vector3dMap& p_obs, const size_t& num_obs) {
	VectorStateMap dhdx_map;
	Vector3d p_tild;
	Vector3d p_quad;
	state_vector_t dhdx;

	for (size_t i = 0; i < num_obs; i++) {
		p_quad << x.at(i).x(), x.at(i).y(), x.at(i).z();
		p_tild = p_quad - p_obs.at(i);
		dhdx << 2.0 *  p_tild.transpose() * Matrix3d::Identity(), 0., 0., 0., 0., 0., 0.;
		dhdx_map.insert({i, dhdx});
	}
	return dhdx_map;
}

state_jacobian_t MavAsif::jacobian(const state_vector_t& x, const control_vector_t& u, const position_t& p_des) const {
	//x = px, py, pz, vx, vy, vz, phi, theta, psi
    state_jacobian_t J;
	Matrix<double, NUM_STATES, NUM_STATES> dfdx;
	Matrix<double, NUM_STATES, NUM_CONTROL_INPUTS> dfdu;
	Matrix<double, NUM_CONTROL_INPUTS, NUM_STATES> dudx;

	double px = x(0);
	double py = x(1);
	double pz = x(2);
	double vx = x(3);
	double vy = x(4);
	double vz = x(5);
	double phi = x(6);
	double theta = x(7);
	double psi = x(8);

	double coll_acc = u(0);
//	double wx = u(1);
	double wy = u(2);
	double wz = u(3);
	double cphi = cos(phi);
	double sphi = sin(phi);
	double ctheta = cos(theta);
	double stheta = sin(theta);
	double cpsi = cos(psi);
	double spsi = sin(psi);
	double c2psi = cos(2*psi);
	double s2psi = sin(2*psi);
	double tantheta = tan(theta);
	double secant_theta_squared = 1.0/(ctheta*ctheta);

	double ax = kp_ * (p_des.x() - px) - kv_ * vx;
	double ay = kp_ * (p_des.y() - py) - kv_ * vy;
	double az = kp_ * (p_des.z() - pz) + gravity_ - kv_ * vz;

	dfdx.setZero();
	dfdx(0,3) = 1.0;
	dfdx(1,4) = 1.0;
	dfdx(2, 5) = 1.0;
	dfdx(3,6) = coll_acc*(cphi*spsi - cpsi*sphi*stheta);
	dfdx(3,7) = coll_acc*cphi*cpsi*ctheta;
	dfdx(3,8) = coll_acc*(cpsi*sphi - cphi*spsi*stheta);
	dfdx(4,6) = -coll_acc*(cphi*cpsi + sphi*spsi*stheta);
	dfdx(4,7) = coll_acc*cphi*ctheta*spsi;
	dfdx(4,8) = coll_acc*(sphi*spsi + cphi*cpsi*stheta);
	dfdx(5,6) = -coll_acc*ctheta*sphi;
	dfdx(5,7) = -coll_acc*cphi*stheta;
	dfdx(6,6) = wy*cphi*tantheta - wz*sphi*tantheta;
	dfdx(6,7) = wz*cphi*secant_theta_squared + wy*sphi*secant_theta_squared;
	dfdx(7,6) = -wz*cphi - wy*sphi;
	dfdx(8,6) = (wy*cphi)/ctheta - (wz*sphi)/ctheta;
	dfdx(8,7) = (wz*cphi*stheta)*secant_theta_squared + (wy*sphi*stheta)*secant_theta_squared;

	dfdu.setZero();
	dfdu(3,0) = sphi*spsi + cphi*cpsi*stheta;
	dfdu(4,0) = cphi*spsi*stheta - cpsi*sphi;
	dfdu(5,0) = cphi*ctheta;
	dfdu(6,1) = 1.0;
	dfdu(6,2) = sphi*tantheta;
	dfdu(6,3) = cphi*tantheta;
	dfdu(7,2) = cphi;
	dfdu(7,3) = -sphi;
	dfdu(8,2) = sphi/ctheta;
	dfdu(8,3) = cphi/ctheta;

	dudx.setZero();
	dudx(0,0) = -kp_*ax/coll_acc;
	dudx(0,1) = -kp_*ay/coll_acc;
	dudx(0,2) = -kp_*az/coll_acc;
	dudx(0,3) = -kv_*ax/coll_acc;
	dudx(0,4) = -kv_*ay/coll_acc;
	dudx(0,5) = -kv_*az/coll_acc;
	dudx(1,0) = (2*kp_*roll_kp_*abs(az)*(az*az*az*az*spsi - ay*ay*ay*ay*spsi + ay*ay*ay*ay*cpsi*cpsi*spsi + 3*ax*ay*ay*ay*cpsi - 3*ax*ay*ay*ay*cpsi*cpsi*cpsi + ax*ax*ax*ay*cpsi*cpsi*cpsi - 3*ax*ay*az*az*cpsi - 3*ax*ax*ay*ay*cpsi*cpsi*spsi + 3*ax*ax*az*az*cpsi*cpsi*spsi - 3*ay*ay*az*az*cpsi*cpsi*spsi + 6*ax*ay*az*az*cpsi*cpsi*cpsi))/(az*sqrt(ax*ax*cpsi*cpsi + ay*ay*spsi*spsi + az*az - 2*ax*ay*cpsi*spsi)*(ax*ax*ax*ax*c2psi - ay*ay*ay*ay*c2psi + ax*ax*ax*ax + ay*ay*ay*ay + 2*az*az*az*az + 2*ax*ax*ay*ay - ax*ax*az*az - ay*ay*az*az - 2*ax*ay*ay*ay*s2psi - 2*ax*ax*ax*ay*s2psi - 3*ax*ax*az*az*c2psi + 3*ay*ay*az*az*c2psi + 6*ax*ay*az*az*s2psi));
	dudx(1,1) = (2*kp_*roll_kp_*abs(az)*(az*az*az*az*cpsi - ax*ax*ax*ax*cpsi*cpsi*cpsi + ax*ay*ay*ay*spsi - 3*ax*ax*ay*ay*cpsi - 3*ax*ax*az*az*cpsi + 3*ay*ay*az*az*cpsi + 3*ax*ax*ay*ay*cpsi*cpsi*cpsi + 3*ax*ax*az*az*cpsi*cpsi*cpsi - 3*ay*ay*az*az*cpsi*cpsi*cpsi - ax*ay*ay*ay*cpsi*cpsi*spsi + 3*ax*ax*ax*ay*cpsi*cpsi*spsi + 3*ax*ay*az*az*spsi - 6*ax*ay*az*az*cpsi*cpsi*spsi))/(az*sqrt(ax*ax*cpsi*cpsi + ay*ay*spsi*spsi + az*az - 2*ax*ay*cpsi*spsi)*(ax*ax*ax*ax*c2psi - ay*ay*ay*ay*c2psi + ax*ax*ax*ax + ay*ay*ay*ay + 2*az*az*az*az + 2*ax*ax*ay*ay - ax*ax*az*az - ay*ay*az*az - 2*ax*ay*ay*ay*s2psi - 2*ax*ax*ax*ay*s2psi - 3*ax*ax*az*az*c2psi + 3*ay*ay*az*az*c2psi + 6*ax*ay*az*az*s2psi));
	dudx(1,2) = -(2*kp_*roll_kp_*abs(az)*(ay*cpsi + ax*spsi)*(3*ax*ax*cpsi*cpsi + 3*ay*ay*spsi*spsi + az*az - 6*ax*ay*cpsi*spsi))/(sqrt(ax*ax*cpsi*cpsi + ay*ay*spsi*spsi + az*az - 2*ax*ay*cpsi*spsi)*(ax*ax*ax*ax*c2psi - ay*ay*ay*ay*c2psi + ax*ax*ax*ax + ay*ay*ay*ay + 2*az*az*az*az + 2*ax*ax*ay*ay - ax*ax*az*az - ay*ay*az*az - 2*ax*ay*ay*ay*s2psi - 2*ax*ax*ax*ay*s2psi - 3*ax*ax*az*az*c2psi + 3*ay*ay*az*az*c2psi + 6*ax*ay*az*az*s2psi));
	dudx(1,3) = (2*kv_*roll_kp_*abs(az)*(az*az*az*az*spsi - ay*ay*ay*ay*spsi + ay*ay*ay*ay*cpsi*cpsi*spsi + 3*ax*ay*ay*ay*cpsi - 3*ax*ay*ay*ay*cpsi*cpsi*cpsi + ax*ax*ax*ay*cpsi*cpsi*cpsi - 3*ax*ay*az*az*cpsi - 3*ax*ax*ay*ay*cpsi*cpsi*spsi + 3*ax*ax*az*az*cpsi*cpsi*spsi - 3*ay*ay*az*az*cpsi*cpsi*spsi + 6*ax*ay*az*az*cpsi*cpsi*cpsi))/(az*sqrt(ax*ax*cpsi*cpsi + ay*ay*spsi*spsi + az*az - 2*ax*ay*cpsi*spsi)*(ax*ax*ax*ax*c2psi - ay*ay*ay*ay*c2psi + ax*ax*ax*ax + ay*ay*ay*ay + 2*az*az*az*az + 2*ax*ax*ay*ay - ax*ax*az*az - ay*ay*az*az - 2*ax*ay*ay*ay*s2psi - 2*ax*ax*ax*ay*s2psi - 3*ax*ax*az*az*c2psi + 3*ay*ay*az*az*c2psi + 6*ax*ay*az*az*s2psi));
	dudx(1,4) = (2*kv_*roll_kp_*abs(az)*(az*az*az*az*cpsi - ax*ax*ax*ax*cpsi*cpsi*cpsi + ax*ay*ay*ay*spsi - 3*ax*ax*ay*ay*cpsi - 3*ax*ax*az*az*cpsi + 3*ay*ay*az*az*cpsi + 3*ax*ax*ay*ay*cpsi*cpsi*cpsi + 3*ax*ax*az*az*cpsi*cpsi*cpsi - 3*ay*ay*az*az*cpsi*cpsi*cpsi - ax*ay*ay*ay*cpsi*cpsi*spsi + 3*ax*ax*ax*ay*cpsi*cpsi*spsi + 3*ax*ay*az*az*spsi - 6*ax*ay*az*az*cpsi*cpsi*spsi))/(az*sqrt(ax*ax*cpsi*cpsi + ay*ay*spsi*spsi + az*az - 2*ax*ay*cpsi*spsi)*(ax*ax*ax*ax*c2psi - ay*ay*ay*ay*c2psi + ax*ax*ax*ax + ay*ay*ay*ay + 2*az*az*az*az + 2*ax*ax*ay*ay - ax*ax*az*az - ay*ay*az*az - 2*ax*ay*ay*ay*s2psi - 2*ax*ax*ax*ay*s2psi - 3*ax*ax*az*az*c2psi + 3*ay*ay*az*az*c2psi + 6*ax*ay*az*az*s2psi));
	dudx(1,5) = -(2*kv_*roll_kp_*abs(az)*(ay*cpsi + ax*spsi)*(3*ax*ax*cpsi*cpsi + 3*ay*ay*spsi*spsi + az*az - 6*ax*ay*cpsi*spsi))/(sqrt(ax*ax*cpsi*cpsi + ay*ay*spsi*spsi + az*az - 2*ax*ay*cpsi*spsi)*(ax*ax*ax*ax*c2psi - ay*ay*ay*ay*c2psi + ax*ax*ax*ax + ay*ay*ay*ay + 2*az*az*az*az + 2*ax*ax*ay*ay - ax*ax*az*az - ay*ay*az*az - 2*ax*ay*ay*ay*s2psi - 2*ax*ax*ax*ay*s2psi - 3*ax*ax*az*az*c2psi + 3*ay*ay*az*az*c2psi + 6*ax*ay*az*az*s2psi));
	dudx(1,6) = -roll_kp_;
	dudx(1,8) = (2*roll_kp_*abs(az)*(ax*cpsi - ay*spsi)*(ax*ax*ax*ax*cpsi*cpsi - ay*ay*ay*ay*cpsi*cpsi + ay*ay*ay*ay - az*az*az*az + ax*ax*ay*ay + 3*ax*ax*az*az - ax*ay*ay*ay*s2psi - ax*ax*ax*ay*s2psi - 3*ax*ax*az*az*cpsi*cpsi + 3*ay*ay*az*az*cpsi*cpsi + 3*ax*ay*az*az*s2psi))/(az*sqrt(ax*ax*cpsi*cpsi + ay*ay*spsi*spsi + az*az - 2*ax*ay*cpsi*spsi)*(ax*ax*ax*ax*c2psi - ay*ay*ay*ay*c2psi + ax*ax*ax*ax + ay*ay*ay*ay + 2*az*az*az*az + 2*ax*ax*ay*ay - ax*ax*az*az - ay*ay*az*az - 2*ax*ay*ay*ay*s2psi - 2*ax*ax*ax*ay*s2psi - 3*ax*ax*az*az*c2psi + 3*ay*ay*az*az*c2psi + 6*ax*ay*az*az*s2psi));
	dudx(2,0) = -(az*kp_*pitch_kp_*cpsi)/(ax*ax*cpsi*cpsi + ay*ay*spsi*spsi + az*az - 2*ax*ay*cpsi*spsi);
	dudx(2,1) = (az*kp_*pitch_kp_*spsi)/(ax*ax*cpsi*cpsi + ay*ay*spsi*spsi + az*az - 2*ax*ay*cpsi*spsi);
	dudx(2,2) = (kp_*pitch_kp_*(ax*cpsi - ay*spsi))/(ax*ax*cpsi*cpsi + ay*ay*spsi*spsi + az*az - 2*ax*ay*cpsi*spsi);
	dudx(2,3) = -(az*kv_*pitch_kp_*cpsi)/(ax*ax*cpsi*cpsi + ay*ay*spsi*spsi + az*az - 2*ax*ay*cpsi*spsi);
	dudx(2,4) = (az*kv_*pitch_kp_*spsi)/(ax*ax*cpsi*cpsi + ay*ay*spsi*spsi + az*az - 2*ax*ay*cpsi*spsi);
	dudx(2,5) = (kv_*pitch_kp_*(ax*cpsi - ay*spsi))/(ax*ax*cpsi*cpsi + ay*ay*spsi*spsi + az*az - 2*ax*ay*cpsi*spsi);
	dudx(2,7) = -pitch_kp_;
	dudx(2,8) = -(az*pitch_kp_*(ay*cpsi + ax*spsi))/(ax*ax*cpsi*cpsi + ay*ay*spsi*spsi + az*az - 2*ax*ay*cpsi*spsi);
	dudx(3,8) = -yaw_kp_;

	J = dfdx + dfdu*dudx;
    return J;
}




