#include "mav_control_router.hpp"

using namespace asif;

int main(int argc, char *argv[]) {
    std::cout << "Starting MAV Control example..." << std::endl;
    Eigen::initParallel();
    Eigen::setNbThreads(2);
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavControlRouter>());

    rclcpp::shutdown();
    return 0;
}