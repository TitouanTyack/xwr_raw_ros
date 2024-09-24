#include <cstdio>
#include "xwr_raw_ros/recver.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Recver>());
    rclcpp::shutdown();
    return 0;
}
