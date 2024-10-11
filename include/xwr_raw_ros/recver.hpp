#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <string>
#include <cctype>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h> 
#include <signal.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "xwr_raw_ros_msgs/msg/radar_frame_full.hpp"
#include "xwr_raw_ros_msgs/msg/radar_frame_stamped.hpp"
#include "xwr_raw_ros_msgs/msg/radar_frame.hpp"

class FrameBuffer{
    public:
        FrameBuffer(rclcpp::Logger _logger, size_t _capacity, size_t _frame_size){
            buffer = (unsigned char*)calloc(_capacity, sizeof(char));
            capacity   = _capacity;
            frame_size = _frame_size;
            curr_idx   = 0;
            last_seqn  = 0;
            logger = std::make_shared<rclcpp::Logger>(_logger); 
        }

        void pad_zeros(size_t n_msgs, size_t msg_size){
            size_t total_size = n_msgs*msg_size;

            if (curr_idx + total_size > capacity){
                memset(buffer+curr_idx, 0, (capacity-curr_idx));
                memset(buffer, 0, ((curr_idx + total_size) % capacity));
                curr_idx = (curr_idx + total_size) % capacity;
            }
        }

        unsigned char* add_msg(int seqn, unsigned char* msg, size_t msg_len){
            unsigned char* new_frame = NULL;

            if (seqn > last_seqn + 1){
                RCLCPP_INFO(*logger,"Packet drop.");
                pad_zeros((seqn - last_seqn - 1), msg_len);
            }

            last_seqn = seqn;
            
            if (curr_idx + msg_len > capacity){
                memcpy(buffer+curr_idx, msg, (capacity - curr_idx));
                memcpy(buffer, msg+(capacity-curr_idx), (msg_len - (capacity - curr_idx)));
            }
            else{
                memcpy(buffer+curr_idx, msg, msg_len);
            }

            size_t old_frame_idx = curr_idx / frame_size;
            curr_idx = (curr_idx + msg_len) % capacity;
            size_t new_frame_idx = curr_idx / frame_size;

            if (old_frame_idx != new_frame_idx){
                return buffer + old_frame_idx*frame_size;
            }
            else{
                return NULL;
            }
        }

        unsigned char* buffer;
        size_t capacity;
        size_t frame_size;
        size_t curr_idx;
        size_t last_seqn;
        std::shared_ptr<rclcpp::Logger> logger;
};


/**
 * @brief The Recver class provides a ROS interface for the recver code
 */
class Recver: public rclcpp::Node
{
public:
  Recver()
  : rclcpp::Node("xwr_radar_recver")
  {
    // Parse args.
    host_ip        = this->declare_parameter("host_ip"       , "192.168.33.30");
    host_data_port = this->declare_parameter("host_data_port", 4098);

    parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "xwr_radar");
    parameters_client->wait_for_service();
    auto parameters_future = parameters_client->get_parameters(
        {"radar_config/Platform",
         "radar_params/adc_output_fmt",
         "radar_params/range_bias",
         "radar_params/rx_phase_bias",
         "radar_params/frame_size",
         "radar_params/chirp_time",
         "radar_params/chirp_slope",
         "radar_params/frame_time",
         "radar_params/velocity_max",
         "radar_params/velocity_res",
         "radar_params/sample_rate",
         "radar_params/range_max",
         "radar_params/range_res",
         "radar_params/rx",
         "radar_params/tx",
         "radar_params/n_chirps",
         "radar_params/n_rx",
         "radar_params/n_samples"},
        std::bind(&Recver::callbackGlobalParam, this, std::placeholders::_1));

    parameter_flag = true;

    pub_radar_frame = this->create_publisher<xwr_raw_ros_msgs::msg::RadarFrameFull>("radar_data", 1);
	
    timer_ =  this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Recver::update, this));
  }
  private:
    // Parameters
    std::string host_ip;
    int host_data_port;

    std::string         platform;
    int                 adc_output_fmt;
    double              range_bias;
    std::vector<double> rx_phase_bias;
    int                 frame_size;
    std::shared_future<int>    frame_size_;
    double              chirp_time;
    double              chirp_slope;
    double              frame_time;
    double              velocity_max;
    double              velocity_res;
    int                 sample_rate;
    double              range_max;
    double              range_res;
    std::vector<int>    rx;
    std::vector<int>    tx;
    int                 n_chirps;
    int                 n_rx;
    int                 n_samples;
    std::vector<int>    shape;
    bool parameter_flag;
    rclcpp::Clock clocker;
    rclcpp::Time current_stamp;
    bool first_stamp;
    int msg_counter;

    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client;
    void callbackGlobalParam(std::shared_future<std::vector<rclcpp::Parameter>> future);

    rclcpp::Publisher<xwr_raw_ros_msgs::msg::RadarFrameFull>::SharedPtr pub_radar_frame;
    rclcpp::TimerBase::SharedPtr timer_;
    void update(void);
};