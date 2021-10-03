#include <iostream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>

using namespace std;

class back_end_parameter {

public:
    back_end_parameter();
    void read_ros_parameter(ros::NodeHandle &n);
    void read_back_end_parameter();

    string image_topic;
    string imu_topic;
    string config_file;
    
    Eigen::Matrix3d rotation_ex;
    Eigen::Matrix3d translation_ex;
    Eigen::Vector3d g{0, 0, 9.81};
    
    int window_size = 10;
    int num_interations;
    int estimated_ed;
    int image_col;
    int image_row;

    double acc_n;
    double gyr_n;
    double acc_bn;
    double gyr_bn;
    double td;
    double keyframe_parallax;
    double max_solver_time;
    double focal_length = 460.0;

    int o_p = 0; 
    int o_r = 3;
    int o_v = 6;
    int o_ba = 9;
    int o_bg = 12;

    int o_an = 0;
    int o_gn = 3;
    int o_a_bn = 6;
    int o_g_bn = 9;
};