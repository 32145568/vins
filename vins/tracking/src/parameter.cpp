#include "parameter.hpp"

tracking_parameter::tracking_parameter() {
    ;
}

void tracking_parameter::read_ros_parameter(ros::NodeHandle &n) {
    string name = "config_file";
    
    if(n.getParam(name, config_file)) {
        ROS_INFO_STREAM("name: " << name <<"is: "<< config_file);
    } 
    else {
        ROS_ERROR_STREAM("no such name");
        n.shutdown();
    }
}

void tracking_parameter::read_parameter() {

    cv::FileStorage read_config(config_file, cv::FileStorage::READ);

    if(!read_config.isOpened()) {
        cout<<"open file failed"<<endl;
    }
    else {
        read_config["image_topic"] >> image_topic;
        image_row = read_config["image_height"];
        image_col = read_config["image_width"];
        f_threshold = read_config["F_threshold"];
        freq = read_config["freq"];
        max_kps_n = read_config["max_cnt"];
        min_dis = read_config["min_dist"];
        equalize = read_config["equalize"];
    }

    read_config.release();
}