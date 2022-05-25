#pragma once

#include "Eigen/Dense"

namespace asif
{
#define NUM_STATES 9
#define NUM_CONTROL_INPUTS 4
#define POWER_OF_TWO(EXPONENT) (1 << (EXPONENT))

typedef Eigen::Matrix<double, NUM_STATES, 1> state_vector_t;
typedef Eigen::Matrix<double, NUM_CONTROL_INPUTS, 1> control_vector_t;
typedef Eigen::Matrix<double, NUM_STATES, NUM_STATES> state_jacobian_t;
typedef Eigen::Matrix<double, NUM_STATES, NUM_STATES> state_sensitivity_t;
typedef Eigen::Vector3d position_t;

typedef std::map<size_t, Eigen::Vector3d, std::less<int>,Eigen::aligned_allocator<std::pair<const size_t, Eigen::Vector3d> > > Vector3dMap;
typedef std::map<size_t, state_vector_t, std::less<int>,Eigen::aligned_allocator<std::pair<const size_t, state_vector_t> > > VectorStateMap;
typedef std::map<size_t, state_sensitivity_t, std::less<int>,Eigen::aligned_allocator<std::pair<const size_t, state_sensitivity_t> > > MatrixStateSensitivityMap;

static constexpr uint8_t OFFBOARD_ENABLE_CHANNEL = 7;
static constexpr uint8_t POSITION_SETPOINT_CHANNEL = 5;
static constexpr uint8_t ASIF_ENABLE_CHANNEL = 6;

typedef struct mavControl {
	double roll_rate; // rad/s
	double pitch_rate; // rad/s
	double yaw_rate; // rad/s
	double thrust; // Newtons
#if defined(__cplusplus)
	mavControl()
			: roll_rate(0)
			, pitch_rate(0)
			, yaw_rate(0)
			, thrust(0) {};
#endif
} mavControl;


Eigen::Vector3d quaternion_to_rpy_wrap(double qw,double qx,double qy,double qz)
{
	Eigen::Vector3d rpy;
	double roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (pow(qx, 2) + pow(qy, 2)));
	double pitch = asin(2 * (qw * qy - qz * qx));
	double yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (pow(qy, 2) + pow(qz, 2)));

	rpy << roll,
	       pitch,
	       yaw;

	return rpy;
}

Eigen::Matrix3d dcm(double phi,double theta,double psi) {
	double cphi = cos(phi);
	double sphi = sin(phi);
	double ctheta = cos(theta);
	double stheta = sin(theta);
	double cpsi = cos(psi);
	double spsi = sin(psi);
	Eigen::Matrix3d dcm;
	dcm << ctheta*cpsi, -cphi*spsi+sphi*stheta*cpsi,  sphi*spsi+cphi*stheta*cpsi,
	     ctheta*spsi,  cphi*cpsi+sphi*stheta*spsi, -sphi*cpsi+cphi*stheta*spsi,
	         -stheta,                 sphi*ctheta,                 cphi*ctheta;
	return dcm;
}

}