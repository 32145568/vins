#include <iostream>
#include <vector>
#include <map>
#include <thread>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Dense>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "parameter.hpp"

using namespace std;
using namespace camodocal;
using namespace Eigen;

struct tracking_frame {
    cv::Mat image;
    vector<cv::Point2f> keypoints;
    vector<cv::Point2f> un_keypoints;
    vector<int> ids;
    vector<int> tracked_number;
    double time;
};

class tracking {

public:
    tracking_frame* pre_frame;
    tracking_parameter* t_p;
    vector<cv::Point2f> temp_kps;
    vector<cv::Point2f> temp_un_kps;
    vector<cv::Point2f> temp_kps_v;
    vector<cv::Point2f> new_kps;
    vector<int> temp_ids;
    vector<int> correspondence;
    vector<int> temp_tracked_n;
    vector<uchar> status;
    cv::Mat temp_image;
    cv::Mat mask;
    camodocal::CameraPtr m_camera;

    double temp_time;
    int global_id;

    tracking(cv::Mat &image, tracking_parameter* _t_p, int n, double time);
    vector<cv::Point2f> new_keypoints_detection(int n);
    vector<cv::Point2f> new_keypoints_detection_with_sub(int n);
    void process(cv::Mat &image, double time, bool if_pub);
    void undistorted_keypoint(cv::Point2f &point, cv::Point2f &un_point);
    void reject_with_F();
    void remove_outlier();
    void remove_outlier_with_make_pair(vector<pair<int, int>> &pairs);
};