#include "parameter.hpp"

back_end_parameter::back_end_parameter() {
    ;
}

void back_end_parameter::read_ros_parameter(ros::NodeHandle &n) {
    string name = "config_file";
    if(n.getParam(name, config_file)) {
        ROS_INFO("read ros parameter successfully");
    } else {
        ROS_INFO("read ros parameter failed");
        n.shutdown();
    }
}

void back_end_parameter::read_back_end_parameter() {
    cv::FileStorage read_config(config_file, cv::FileStorage::READ);
    if(!read_config.isOpened()) {
        ROS_INFO("can not open the config file");
        return;
    } 

    read_config["image_topic"] >> image_topic;
    read_config["imu_topic"] >> imu_topic;
    
    cv::Mat cv_rotation_ex;
    cv::Mat cv_translation_ex;
    read_config["extrinsicRotation"] >> cv_rotation_ex;
    read_config["extrinsicTranslation"] >> cv_translation_ex;
    
    cv::cv2eigen(cv_rotation_ex, rotation_ex);
    cv::cv2eigen(cv_translation_ex, translation_ex);
    
    num_interations = read_config["max_num_iterations"];
    estimated_ed = read_config["estimate_td"];
    image_col = read_config["image_width"];
    image_row = read_config["image_height"];

    acc_n = read_config["acc_n"];
    gyr_n  = read_config["gyr_n"];
    acc_bn = read_config["acc_w"];
    gyr_bn = read_config["gyr_w"];
    td = read_config["td"];
    keyframe_parallax = read_config["keyframe_parallax"];
    keyframe_parallax = keyframe_parallax / focal_length;
    max_solver_time = read_config["keyframe_parallax"];
}