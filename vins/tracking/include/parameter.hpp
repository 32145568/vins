#include <iostream>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

class tracking_parameter {

public:
    int image_col;
    int image_row;
    int focal_length = 460;
    int max_kps_n; 
    int min_dis;
    int freq;
    int equalize;
    double f_threshold;
    
    string image_topic;
    string config_file;
    
    tracking_parameter();
    void read_ros_parameter(ros::NodeHandle &n);
    void read_parameter();
};

