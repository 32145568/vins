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
#include <thread>
#include <mutex>
#include <condition_variable>

using namespace std;

bool first_frame = true;
bool if_pub = false;
bool if_restart = false;
int pub_frame_n = 0;
double cur_frame_time = -1;
double last_frame_time = -1;
double first_frame_time = -1;
std::mutex m;
std::condition_variable c_v;
tracking_parameter* tp = new tracking_parameter();


void image_callback(const sensor_msgs::ImageConstPtr &image_msgs) {
    m.lock();
    if(first_frame) {
        cur_frame_time = image_msgs->header.stamp.toSec();  
        last_frame_time = cur_frame_time;      
        first_frame = false;
    } else {
        last_frame_time = cur_frame_time;
        cur_frame_time = image_msgs->header.stamp.toSec();
    }
    cout<<"time: "<<cur_frame_time-last_frame_time<<endl;
    m.unlock();
    c_v.notify_one();
}

int judge() {
    std::this_thread::sleep_for(std::chrono::seconds(1));
	return 0;
}

void process() {		

	for(size_t i=0; i<100; i++) {	
        cout<<tp->image_topic<<endl;
		std::unique_lock<std::mutex> lock(m);
		c_v.wait(lock, [&] { return judge();});		
		lock.unlock();  
	}
}


int main(int argc, char **argv) {
    
    ros::init(argc, argv, "test");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    tp->read_ros_parameter(n);
    tp->read_parameter();

    ros::Subscriber sub_image = n.subscribe(tp->image_topic, 100, image_callback);
    std::thread p(process);
    ros::spin();
    p.join();
    return 0;
}