#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <mutex>
#include <thread>
#include <map>
#include <queue>
#include <cv_bridge/cv_bridge.h>
#include <condition_variable>
#include "parameter.hpp"
#include "back_end/kf.h"
#include "back_end/up_kf.h"
#include "optimization.hpp"

using namespace std;

back_end_parameter *b_p = new back_end_parameter();
optimization *op = new optimization();

ros::Publisher pub_kp_msgs;
ros::Publisher pub_up_kf_msgs;

queue<sensor_msgs::ImuConstPtr> imu_q;
queue<sensor_msgs::PointCloudConstPtr> feature_q;
queue<sensor_msgs::ImageConstPtr> raw_image_q; 

mutex queue_lock;
mutex optimization_lock;
mutex state_lock;
std::condition_variable con;

bool if_first_image = true;

struct measurement {
    sensor_msgs::PointCloudConstPtr feature_msgs;
    queue<sensor_msgs::ImuConstPtr> imu_msgs;
    sensor_msgs::ImageConstPtr image_msgs;
};

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msgs) {
    queue_lock.lock();
    feature_q.push(feature_msgs);
    queue_lock.unlock();
    con.notify_one();
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msgs) {
    queue_lock.lock();
    imu_q.push(imu_msgs);
    queue_lock.unlock();
    con.notify_one();
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msgs) {
    if(restart_msgs->data) {
        ROS_INFO("back end restart");
    }
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msgs) {
    queue_lock.lock();
    raw_image_q.push(image_msgs);
    queue_lock.unlock();
    con.notify_one();
}

bool get_measurement(measurement &m) {
    if(imu_q.empty() || feature_q.empty() || raw_image_q.empty()) {
        return false;
    } 

    if(imu_q.front()->header.stamp.toSec() >= feature_q.front()->header.stamp.toSec() + op->td) {
        feature_q.pop();
        while(true) {
            if(raw_image_q.empty()) {
                break;
            }
            
            if(raw_image_q.front()->header.stamp.toSec() <= feature_q.front()->header.stamp.toSec()) {
                raw_image_q.pop();
            } else {
                break;
            }
        }

        return false;
    }

    if(imu_q.back()->header.stamp.toSec() < feature_q.front()->header.stamp.toSec()) {
        return false;
    }

    m.feature_msgs = feature_q.front();
    feature_q.pop();

    while(true) {
        if(raw_image_q.front()->header.stamp.toSec() < m.feature_msgs->header.stamp.toSec()) {
            raw_image_q.pop();
        } else {
            m.image_msgs = raw_image_q.front();
            raw_image_q.pop();
            break;
        }
    }

    while(true) {
        if(!(m.imu_msgs.empty())) {
            queue<sensor_msgs::ImuConstPtr> empty_q;
            swap(empty_q, m.imu_msgs);
        }
            
        if(imu_q.front()->header.stamp.toSec() <= m.feature_msgs->header.stamp.toSec()) {
            m.imu_msgs.push(imu_q.front());
            imu_q.pop();
        } else {
            m.imu_msgs.push(imu_q.front());
            break;
        }
    }

    if(if_first_image) {
        ROS_INFO("first frame is obtained");
        if_first_image = false;
        return false;
    }

    return true;
}

void optimization() {        
    double frame_time = 0;
    while(true) {        
        measurement m;
        
        if(frame_time == 0) {
            frame_time = m.feature_msgs->header.stamp.toSec();
        }
        cout<<"image freq: "<<m.feature_msgs->header.stamp.toSec()-frame_time<<endl;
        frame_time = m.feature_msgs->header.stamp.toSec();

        std::unique_lock<std::mutex> lock(queue_lock);
        con.wait(lock, [&] {return get_measurement(m);});
        lock.unlock();

        optimization_lock.lock();
        cout<<"imu: "<<m.imu_msgs.back()->header.stamp.toSec() - m.imu_msgs.front()->header.stamp.toSec()<<endl;
        cout<<"image-feature: "<<m.feature_msgs->header.stamp.toSec() - m.image_msgs->header.stamp.toSec()<<endl;
        optimization_lock.unlock();

        cout<<endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "back_end");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    b_p->read_ros_parameter(n);
    b_p->read_back_end_parameter();

    ros::Subscriber sub_feature = n.subscribe("/tracking/feature", 2000, feature_callback);
    ros::Subscriber sub_imu = n.subscribe(b_p->imu_topic, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe(b_p->image_topic, 2000, image_callback, ros::TransportHints().tcpNoDelay());    
    ros::Subscriber sub_restart = n.subscribe("/tracking/restart", 2000, restart_callback);

    pub_kp_msgs = n.advertise<back_end::kf>("keyframe", 1000);
    pub_up_kf_msgs = n.advertise<back_end::up_kf>("update_keyframe", 1000);

    thread optimization_thread{optimization};

    ros::spin();
    optimization_thread.join();
}