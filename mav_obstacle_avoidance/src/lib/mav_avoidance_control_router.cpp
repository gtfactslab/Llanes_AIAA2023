#include "mav_avoidance_control_router.hpp"
#include <utility>
#include "Eigen/Dense"
#include <px4_ros_com/frame_transforms.h>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_ros_com::frame_transforms::utils::quaternion;
using namespace px4_ros_com::frame_transforms;
using namespace mav_asif_msgs::msg;
using namespace px4_msgs::msg;
using namespace asif;

MavControlRouter::MavControlRouter()
        : Node("mav_avoidance_control_router") {
    // ----------------------- Parameter Initialization ------------------
    this->declare_parameter("mav_id");
	this->declare_parameter("mass");
	this->declare_parameter("gravity");
	this->declare_parameter("obstacle_info.p_obs");
	this->declare_parameter("obstacle_info.r_obs");
	this->declare_parameter("obstacle_info.num_obs");
	this->declare_parameter("position_controller.kp");
	this->declare_parameter("position_controller.kv");
	this->declare_parameter("position_controller.roll_kp");
	this->declare_parameter("position_controller.pitch_kp");
	this->declare_parameter("position_controller.yaw_kp");
	this->declare_parameter("thrust_model.mav_max_thrust");
	this->declare_parameter("thrust_model.mav_min_thrust");
    try {
	    this->get_parameter("mav_id", mav_id_);
	    this->get_parameter("mass", mass_);
	    this->get_parameter("gravity", gravity_);
	    rclcpp::Parameter p_obs_param = this->get_parameter("obstacle_info.p_obs");
	    rclcpp::Parameter r_obs_param = this->get_parameter("obstacle_info.r_obs");
	    this->get_parameter("obstacle_info.num_obs", num_obs_);
	    this->get_parameter("position_controller.kp", kp_);
	    this->get_parameter("position_controller.kv", kv_);
	    this->get_parameter("position_controller.roll_kp", roll_kp_);
	    this->get_parameter("position_controller.pitch_kp", pitch_kp_);
	    this->get_parameter("position_controller.yaw_kp", yaw_kp_);
	    this->get_parameter("thrust_model.mav_max_thrust", mav_max_thrust_);
	    this->get_parameter("thrust_model.mav_min_thrust", mav_min_thrust_);

	    std::vector<double> p_obs_tmp = p_obs_param.as_double_array();
	    std::vector<double> r_obs_tmp = r_obs_param.as_double_array();
	    r_obs_.resize(num_obs_);

	    for(size_t i = 0; i < num_obs_; i++) {
		    p_obs_.insert({i,position_t( p_obs_tmp[3*i], p_obs_tmp[3*i+1], p_obs_tmp[3*i+2])});
		    r_obs_(i) = r_obs_tmp[i];
	    }

    } catch (rclcpp::ParameterTypeException &excp) {
        RCLCPP_ERROR(get_logger(), "Parameter type exception caught");
        rclcpp::shutdown(nullptr, "Parameter type exception caught on initialization");
    }

    // ----------------------- Publishers --------------------------
    vehicle_command_pub_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>(
                    "mav" + std::to_string(mav_id_) + "/fmu/vehicle_command/in", 10);
    offboard_control_mode_pub_ =
            this->create_publisher<px4_msgs::msg::OffboardControlMode>(
                    "mav" + std::to_string(mav_id_) + "/fmu/offboard_control_mode/in", 10);
    asif_status_pub_ =
            this->create_publisher<AsifStatus>("mav" + std::to_string(mav_id_) + "/asif_status", 10);
    vehicle_rates_setpoint_pub_ =
            this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
                    "mav" + std::to_string(mav_id_) + "/fmu/vehicle_rates_setpoint/in", 10);
    // ----------------------- Subscribers --------------------------
    mav_battery_status_sub_ =
            this->create_subscription<px4_msgs::msg::BatteryStatus>(
                    "mav" + std::to_string(mav_id_) + "/fmu/battery_status/out", 10,
                    [this](const BatteryStatus::UniquePtr msg) {
                        mav_battery_status_ = *msg;
                    });
    timesync_sub_ =
            this->create_subscription<px4_msgs::msg::Timesync>("mav" + std::to_string(mav_id_) + "/fmu/timesync/out",
                                                               10,
                                                               [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
                                                                   timestamp_.store(msg->timestamp);
                                                               });
    mav_vehicle_status_sub_ =
            this->create_subscription<px4_msgs::msg::VehicleStatus>(
                    "mav" + std::to_string(mav_id_) + "/fmu/vehicle_status/out", 10,
                    [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
                        mav_vehicle_status_ = *msg;
                    });
    mav_channels_sub_ =
            this->create_subscription<px4_msgs::msg::RcChannels>(
                    "mav" + std::to_string(mav_id_) + "/fmu/rc_channels/out", 10,
                    [this](const px4_msgs::msg::RcChannels::UniquePtr msg) {
                        mav_channels_ = *msg;
                        if (msg->channels[ASIF_ENABLE_CHANNEL - 1] >= 0.75) {
                            asif_enabled_ = true;
                        } else {
                            asif_enabled_ = false;
                        }
                        if (mav_channels_.channels[POSITION_SETPOINT_CHANNEL - 1] >= 0.75) {
	                        is_static_reference_ = true;
                        } else {
                        	if(is_static_reference_) {
		                        reference_time_ = rclcpp::Time(1000*timestamp_.load());
                        	}
	                        is_static_reference_ = false;
                        }
                    });
    mav_estimator_odometry_sub_ =
            this->create_subscription<VehicleOdometry>("mav" + std::to_string(mav_id_) + "/fmu/vehicle_odometry/out", 10,
                                                       [this](const VehicleOdometry::UniquePtr msg) {
                                                           mav_odom_ = *msg;
                                                       });
#ifdef RUN_SITL
	mav_channels_.channels[OFFBOARD_ENABLE_CHANNEL - 1] = 1.0;
	mav_channels_.channels[POSITION_SETPOINT_CHANNEL - 1] = 1.0;
#endif
    auto timer_callback = [this]() -> void
    {
	    if(is_static_reference_) {
		    p_des_ << 0.5, 0.0, -0.5;
	    } else {
		    double time = (rclcpp::Time(1000*timestamp_.load()) - reference_time_).seconds();
		    p_des_ << 0.5*cos(0.2*time), 0.5*sin(0.2*time), -0.5;
	    }

        position_controller(p_des_);
        set_control();
    };

    timer_ = create_wall_timer(10ms, timer_callback);
#ifdef RUN_SITL
	auto asif_activate_callback = [this]() -> void
		{
//            mav_channels_.channels[ASIF_ENABLE_CHANNEL - 1] = 1.0;
//			mav_channels_.channels[POSITION_SETPOINT_CHANNEL - 1] = 0.0;
			is_static_reference_ = false;
			asif_enabled_ = true;
		    reference_time_ = rclcpp::Time(1000*timestamp_.load());
			asif_activate_timer_->cancel();
			asif_activate_timer_->reset();
		};
    asif_activate_timer_ = create_wall_timer(20s, asif_activate_callback);
#endif
}

/**
* @brief Publish the offboard control mode.
*        For this example, only position and altitude controls are active.
*/
void MavControlRouter::publish_offboard_control_mode() const {
    OffboardControlMode msg{};
    msg.timestamp = timestamp_.load();
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = true;

    offboard_control_mode_pub_->publish(msg);
}


/**
 * @brief Send a command to Arm the vehicle
 */
void MavControlRouter::arm() const {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void MavControlRouter::disarm() const {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void MavControlRouter::publish_vehicle_command(uint16_t command, float param1,
                                               float param2) const {
    VehicleCommand msg{};
    msg.timestamp = timestamp_.load();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_pub_->publish(msg);
}

void MavControlRouter::set_control() {
    mav_asif_msgs::msg::AsifStatus asif_status;
    asif_status.position_controller[0].roll_rate = mav_control_.roll_rate;
    asif_status.position_controller[0].pitch_rate = mav_control_.pitch_rate;
    asif_status.position_controller[0].yaw_rate = mav_control_.yaw_rate;
    asif_status.position_controller[0].thrust = mav_control_.thrust;

    if (asif_enabled_) {
        asif_solver_.solve(mav_odom_,
                           mav_control_,
                           p_obs_,
                           r_obs_,
                           num_obs_,
                           computation_time);
        asif_status.active_set_invariance_filter[0].roll_rate = mav_control_.roll_rate;
        asif_status.active_set_invariance_filter[0].pitch_rate = mav_control_.pitch_rate;
        asif_status.active_set_invariance_filter[0].yaw_rate = mav_control_.yaw_rate;
        asif_status.active_set_invariance_filter[0].thrust = mav_control_.thrust;
        asif_status.asif_computation_time = computation_time.count();
    }
    asif_status.asif_active = asif_enabled_;
    asif_status.timestamp = this->now().nanoseconds();
    asif_status_pub_->publish(asif_status);

    publish_control();
}

void MavControlRouter::publish_control() {
    VehicleRatesSetpoint vehicle_rates;
    if ((mav_channels_.channels[OFFBOARD_ENABLE_CHANNEL - 1] >= 0.75) &&
        (mav_vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)) {
        if ((mav_vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) &&
            (offboard_counter_ == 10)) {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        } else if (offboard_counter_ < 11) {
            offboard_counter_++;
        }

        vehicle_rates.timestamp = timestamp_.load();
        vehicle_rates.roll = mav_control_.roll_rate;
        vehicle_rates.pitch = mav_control_.pitch_rate;
        vehicle_rates.yaw = mav_control_.yaw_rate;
        vehicle_rates.thrust_body[2] = -compute_relative_thrust(mav_control_.thrust * mass_);

        publish_offboard_control_mode();
        vehicle_rates_setpoint_pub_->publish(vehicle_rates);
    } else {
        offboard_counter_ = 0;
    }
}

void MavControlRouter::position_controller(const position_t& p_des) {
	Eigen::Vector3d orientation_rpy = quaternion_to_rpy_wrap(mav_odom_.q[0],
	                                                         mav_odom_.q[1],
	                                                         mav_odom_.q[2],
	                                                         mav_odom_.q[3]);
	double px = mav_odom_.x;
	double py = mav_odom_.y;
	double pz = mav_odom_.z;
	double vx = mav_odom_.vx;
	double vy = mav_odom_.vy;
	double vz = mav_odom_.vz;
	double phi = orientation_rpy(0);
	double theta = orientation_rpy(1);
	double psi = orientation_rpy(2);

	double ax = kp_ * (p_des.x() - px) - kv_ * vx;
	double ay = kp_ * (p_des.y() - py) - kv_ * vy;
	double az = kp_ * (p_des.z() - pz) - gravity_ - kv_ * vz;

	Eigen::Vector3d acc_vec(ax,ay,az);
	double acc_mag = acc_vec.norm();
	Eigen::Vector3d acc_dir = acc_vec.normalized();

	Eigen::Vector3d interm_force_frame1 = dcm(0, 0, psi) * acc_dir;
	double theta_cmd = atan2(-interm_force_frame1.x(),-interm_force_frame1.z());

	Eigen::Vector3d interm_force_frame2 = dcm(0, theta_cmd, 0) * interm_force_frame1;
	double phi_cmd = atan2(interm_force_frame2.y(),-interm_force_frame2.z());

	mav_control_.thrust = acc_mag;
	mav_control_.roll_rate = -roll_kp_ * (phi - phi_cmd);
	mav_control_.pitch_rate = -pitch_kp_ * (theta - theta_cmd);
	mav_control_.yaw_rate = -yaw_kp_ * psi;
}

double MavControlRouter::compute_relative_thrust(const double &collective_thrust) const {
#ifdef RUN_SITL
	double rel_thrust = (collective_thrust - mav_min_thrust_) / (mav_max_thrust_ - mav_min_thrust_);
	return (0.54358075 * rel_thrust + 0.25020242 * sqrt(3.6484 * rel_thrust + 0.00772641) - 0.021992793);
#else
	if (mav_battery_status_.voltage_filtered_v > 14.0) {
        double rel_thrust = (collective_thrust - mav_min_thrust_) / (mav_max_thrust_ - mav_min_thrust_);
        return (0.54358075 * rel_thrust + 0.25020242 * sqrt(3.6484 * rel_thrust + 0.00772641) - 0.021992793) *
               (1 - 0.0779 * (mav_battery_status_.voltage_filtered_v - 16.0));
    } else {
        double rel_thrust = (collective_thrust - mav_min_thrust_) / (mav_max_thrust_ - mav_min_thrust_);
        return (0.54358075 * rel_thrust + 0.25020242 * sqrt(3.6484 * rel_thrust + 0.00772641) - 0.021992793);
    }
#endif
}
