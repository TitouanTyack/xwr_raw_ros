#include "xwr_raw_ros/recver.hpp"

void Recver::callbackGlobalParam(std::shared_future<std::vector<rclcpp::Parameter>> future)
{
    auto parameters = future.get();
    parameter_flag = false;
    for (const auto &param: parameters)
    {
        try{
        RCLCPP_INFO(this->get_logger(),"Param received");
        {
            if (param.get_name() == "radar_config/Platform")       platform = param.as_string();        
            if (param.get_name() == "radar_params/adc_output_fmt") adc_output_fmt = param.as_int();                
            if (param.get_name() == "radar_params/range_bias")     range_bias = param.as_double();             
            if (param.get_name() == "radar_params/rx_phase_bias")  {auto rx_phase_bias_tmp = param.as_double_array();
                                                                    rx_phase_bias.assign(rx_phase_bias_tmp.begin(),rx_phase_bias_tmp.end());}        
            if (param.get_name() == "radar_params/frame_size")     frame_size = param.as_int();
            if (param.get_name() == "radar_params/chirp_time")     chirp_time = param.as_double();
            if (param.get_name() == "radar_params/chirp_slope")    chirp_slope = param.as_double();
            if (param.get_name() == "radar_params/frame_time")     frame_time = param.as_double();
            if (param.get_name() == "radar_params/velocity_max")   velocity_max = param.as_double();
            if (param.get_name() == "radar_params/velocity_res")   velocity_res = param.as_double();
            if (param.get_name() == "radar_params/sample_rate")    sample_rate = param.as_int();
            if (param.get_name() == "radar_params/range_max")      range_max = param.as_double();
            if (param.get_name() == "radar_params/range_res")      range_res = param.as_double();
            if (param.get_name() == "radar_params/rx")             {auto rx_tmp = param.as_integer_array();
                                                                    rx.assign(rx_tmp.begin(),rx_tmp.end());}
            if (param.get_name() == "radar_params/tx")             {auto tx_tmp = param.as_integer_array();
                                                                    tx.assign(tx_tmp.begin(),tx_tmp.end());}
            if (param.get_name() == "radar_params/n_chirps")       n_chirps = param.as_int();
            if (param.get_name() == "radar_params/n_rx")           n_rx = param.as_int();
            if (param.get_name() == "radar_params/n_samples")      n_samples = param.as_int();
        }
        if ((param.get_name() == "radar_params/n_chirps") ||
            (param.get_name() == "radar_params/n_rx") ||
            (param.get_name() == "radar_params/n_samples")) {auto shape_tmp = std::vector<int>({n_chirps, n_rx, n_samples});
                                                            shape.assign(shape_tmp.begin(),shape_tmp.end());}
        }
    catch(std::exception e)
    {
        RCLCPP_INFO(this->get_logger(),"Issue with param: %s",param.get_name().c_str());
    }
    }
}

void Recver::update(void)
{
    
    if(!parameter_flag)
    {
        RCLCPP_INFO(this->get_logger(), "Node Started");
        // socket address used for the server
        struct sockaddr_in server_address;
        memset(&server_address, 0, sizeof(server_address));
        server_address.sin_family = AF_INET;
        server_address.sin_addr.s_addr = inet_addr(host_ip.c_str());
        server_address.sin_port = htons((unsigned short)host_data_port);

        // create a UDP socket, creation returns -1 on failure
        int sock;
        if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            RCLCPP_INFO(this->get_logger(), "could not create socket");
        }

        // bind it to listen to the incoming connections on the created server
        // address, will return -1 on error
        if ((bind(sock, (struct sockaddr *)&server_address,
                sizeof(server_address))) < 0) {
            RCLCPP_INFO(this->get_logger(), "could not bind socket");
        }

        int socksize = 131071;
        if ((setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &socksize, sizeof(int))) < 0)
        {
            RCLCPP_INFO(this->get_logger(), "error setting sock opts");
        }
        // allocate frame buffer
        FrameBuffer frame_buffer(this->get_logger(),2*frame_size, frame_size);
    
        // run indefinitely
        xwr_raw_ros_msgs::msg::RadarFrameFull frame;
        frame.platform       = platform;
        frame.adc_output_fmt = adc_output_fmt;
        frame.range_bias     = range_bias;
        frame.rx_phase_bias.assign(rx_phase_bias.begin(),rx_phase_bias.end());

        frame.chirp_time     = chirp_time;
        frame.chirp_slope    = chirp_slope;
        frame.frame_time     = frame_time;
        frame.velocity_max   = velocity_max;
        frame.velocity_res   = velocity_res;

        frame.sample_rate    = sample_rate;
        frame.range_max      = range_max;
        frame.range_res      = range_res;

        frame.tx.assign(tx.begin(),tx.end());
        frame.rx.assign(tx.begin(),tx.end());
        frame.shape.assign(shape.begin(),shape.end());
        
        unsigned int seqn = 0;
        unsigned int bytec = 0;
        while(rclcpp::ok())
        {
        // if(!parameter_flag)
        unsigned char buffer[2048];

        // read content into buffer from an incoming client
        /* auto t1 = chrono::high_resolution_clock::now(); */

        int len = recvfrom(sock, buffer, sizeof(buffer), 0, NULL, NULL);

        seqn = (unsigned int) *buffer;
        bytec = (unsigned int) *(buffer + 4);

        unsigned char* frame_data = frame_buffer.add_msg(seqn, buffer+10, len-10);

        /* auto t2 = chrono::high_resolution_clock::now(); */
        /* chrono::duration<double, std::milli> ms_double = t2 - t1; */
        /* cout << ms_double.count() << "ms\n"; */

        if (frame_data){
            frame.data.assign((int16_t *)frame_data, (int16_t *)(frame_data + frame_buffer.frame_size));
            pub_radar_frame->publish(frame);
        } 
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for params");
    }
}


