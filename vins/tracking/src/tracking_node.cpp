#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include "tracking.hpp"
#include "tic_toc.h"

using namespace std;

bool first_frame = true;
bool if_pub = false;
bool if_restart = false;
int pub_frame_n = 0;
double cur_frame_time = -1;
double last_frame_time = -1;
double first_frame_time = -1;
tracking_parameter* tp = new tracking_parameter();
tracking* tracking_node;
ros::Publisher pub_image;
ros::Publisher pub_feature;
ros::Publisher pub_restart;

void image_callback(const sensor_msgs::ImageConstPtr &image_msgs) {

    if(first_frame) {
        first_frame = false;
        cur_frame_time = image_msgs->header.stamp.toSec();
        first_frame_time = image_msgs->header.stamp.toSec();
        last_frame_time = first_frame_time;
        pub_frame_n = 1;
        cv_bridge::CvImageConstPtr ptr;
        ptr = cv_bridge::toCvCopy(image_msgs, sensor_msgs::image_encodings::MONO8);
        cv::Mat image = ptr->image;
        tracking_node = new tracking(image, tp, 50, cur_frame_time);
        return;
    } else {
        last_frame_time = cur_frame_time;
        cur_frame_time = image_msgs->header.stamp.toSec();

        if(cur_frame_time - last_frame_time > 1 || cur_frame_time < last_frame_time) {
            ROS_INFO("there is a falure in feature tracking");
            if_restart = true;
            first_frame = true;
            pub_frame_n = 0;
            std_msgs::Bool restart_flag;
            restart_flag.data = true;
            pub_restart.publish(restart_flag);
        } else {
            cv_bridge::CvImageConstPtr ptr;
            ptr = cv_bridge::toCvCopy(image_msgs, sensor_msgs::image_encodings::MONO8);
            cv::Mat image = ptr->image;
            TicToc t_tracking;
            if(round(1.0 * pub_frame_n / (cur_frame_time - first_frame_time)) <= tp->freq) {
                sensor_msgs::PointCloudPtr kps_pcl(new sensor_msgs::PointCloud);
                sensor_msgs::ChannelFloat32 id_of_kps;
                sensor_msgs::ChannelFloat32 x_of_kps;
                sensor_msgs::ChannelFloat32 y_of_kps;
                sensor_msgs::ChannelFloat32 vx_of_kps;
                sensor_msgs::ChannelFloat32 vy_of_kps;

                kps_pcl->header = image_msgs->header;
                kps_pcl->header.frame_id = "world";
                
                pub_frame_n ++;
                if_pub = true;
                TicToc t_f;
                tracking_node->process(image, cur_frame_time, if_pub);
                ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
                cv::cvtColor(image, ptr->image, CV_GRAY2RGB);

                for (size_t j = 0; j < tracking_node->temp_kps.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * tracking_node->temp_tracked_n[j] / 20.0);
                    cv::circle(ptr->image, tracking_node->temp_kps[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    geometry_msgs::Point32 p;
                    p.x = tracking_node->temp_un_kps[j].x;
                    p.y = tracking_node->temp_un_kps[j].y;
                    p.z = 1;
                    
                    kps_pcl->points.push_back(p);
                    id_of_kps.values.push_back(tracking_node->temp_ids[j]);
                    x_of_kps.values.push_back(tracking_node->temp_kps[j].x);
                    y_of_kps.values.push_back(tracking_node->temp_kps[j].y);
                    vx_of_kps.values.push_back(tracking_node->temp_kps_v[j].x);
                    vy_of_kps.values.push_back(tracking_node->temp_kps_v[j].y);
                }

                kps_pcl->channels.push_back(id_of_kps);
                kps_pcl->channels.push_back(x_of_kps);
                kps_pcl->channels.push_back(y_of_kps);
                kps_pcl->channels.push_back(vx_of_kps);
                kps_pcl->channels.push_back(vy_of_kps);

                pub_image.publish(ptr->toImageMsg());
                pub_feature.publish(kps_pcl);

                if_pub = false;

            } else {
                tracking_node->process(image, cur_frame_time, if_pub);
            }

            ROS_INFO("tracking cost: %fms", t_tracking.toc());
        }
    }
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "tracking");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    tp->read_ros_parameter(n);
    tp->read_parameter();

    ros::Subscriber sub_image = n.subscribe(tp->image_topic, 100, image_callback);
    pub_image = n.advertise<sensor_msgs::Image>("image", 1000);
    pub_feature = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart", 1000);

    ros::spin();
    return 0;
}