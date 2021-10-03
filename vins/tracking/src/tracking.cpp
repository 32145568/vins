#include "tracking.hpp"

tracking::tracking(cv::Mat &image, tracking_parameter* _t_p, int n, double time) {
    t_p = _t_p;
    if (t_p->equalize)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(image, temp_image);
    } else {
        temp_image = image;
    }

    mask = cv::Mat(t_p->image_row, t_p->image_col, CV_8UC1, cv::Scalar(255));
    temp_time = time;    
    global_id = 0;

    pre_frame = new tracking_frame;
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(t_p->config_file);
    temp_kps = new_keypoints_detection_with_sub(t_p->max_kps_n);
    
    for(size_t i=0; i<temp_kps.size(); i++) {
        cv::Point2f point, un_point;
        point = temp_kps[i];
        undistorted_keypoint(point, un_point);
        temp_ids.push_back(-1);
        temp_tracked_n.push_back(1);
        temp_un_kps.push_back(un_point);
        correspondence.push_back(i);
    }

    pre_frame->image = temp_image.clone();
    pre_frame->keypoints = temp_kps;
    pre_frame->un_keypoints= temp_un_kps;
    pre_frame->ids = temp_ids;
    pre_frame->tracked_number = temp_tracked_n;
    pre_frame->time = temp_time;
}


vector<cv::Point2f> tracking::new_keypoints_detection_with_sub(int n) {
    vector<cv::Point2f> kps;
    cv::goodFeaturesToTrack(temp_image, kps, n, 0.01, t_p->min_dis, mask);
	cv::TermCriteria criteria = cv::TermCriteria(
					cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
					40,
					0.01);
	cv::cornerSubPix(temp_image, kps, cv::Size(6, 6), cv::Size(-1, -1), criteria);
    return kps;
}

vector<cv::Point2f> tracking::new_keypoints_detection(int n) {
    vector<cv::Point2f> kps;
    cv::goodFeaturesToTrack(temp_image, kps, n, 0.01, t_p->min_dis, mask);
    return kps;
}

void tracking::undistorted_keypoint(cv::Point2f &point, cv::Point2f &un_point) {
    Eigen::Vector3d p3;
    Eigen::Vector2d p2;
    p2(0) = point.x;
    p2(1) = point.y;

    m_camera->liftProjective(p2, p3);
    un_point.x = p3(0) / p3(2);
    un_point.y = p3(1) / p3(2);
}

void tracking::process(cv::Mat &image, double time, bool if_pub) {
    mask = cv::Mat(t_p->image_row, t_p->image_col, CV_8UC1, cv::Scalar(255));
    vector<float> err;
    if (t_p->equalize)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(image, temp_image);
    } else {
        temp_image = image;
    }
    temp_time = time;    

    temp_kps.clear();
    temp_un_kps.clear();
    temp_ids.clear();
    temp_tracked_n.clear();
    temp_kps_v.clear();
    
    status.clear();
    new_kps.clear();

    cv::calcOpticalFlowPyrLK(pre_frame->image, temp_image, pre_frame->keypoints, temp_kps, status, err, cv::Size(21, 21), 3);
    remove_outlier();

    if(if_pub) {
        reject_with_F();        
        vector<pair<int, int>> pairs;
        remove_outlier_with_make_pair(pairs);  
        
        if(static_cast<int>(temp_kps.size()) < t_p->max_kps_n) {  
            new_kps = new_keypoints_detection_with_sub(t_p->max_kps_n - static_cast<int>(temp_kps.size()));      
            for(size_t i = 0; i < temp_kps.size(); i++) {
                cv::Point2f p1, p2, v;
                int corr = correspondence[i];
                double delta_t;
                delta_t = temp_time - pre_frame->time;
                p1 = pre_frame->un_keypoints[corr];
                p2 = temp_un_kps[i];
                v.x = (p2.x - p1.x) / delta_t;
                v.y = (p2.y - p1.y) / delta_t;
                temp_kps_v.push_back(v);

                if(temp_ids[i] == -1) {
                    temp_ids[i] = global_id;
                    global_id ++;
                }
                correspondence[i] = i;
            }

            int k = temp_kps.size();
            for(size_t i = 0; i < new_kps.size(); i++) {
                cv::Point2f p, un_p;
                p = new_kps[i];
                undistorted_keypoint(p, un_p);
                temp_kps.push_back(p);
                temp_un_kps.push_back(un_p);
                temp_ids.push_back(-1);
                temp_tracked_n.push_back(1);
                correspondence.push_back(i + k);
            }
                       
            pre_frame->image = temp_image.clone();
            pre_frame->keypoints = temp_kps;
            pre_frame->un_keypoints= temp_un_kps;
            pre_frame->ids = temp_ids;
            pre_frame->tracked_number = temp_tracked_n;
            pre_frame->time = temp_time;
            return;
        } 
    }else {
        pre_frame->image = temp_image.clone();
        pre_frame->keypoints = temp_kps;
        pre_frame->un_keypoints = temp_un_kps;
        pre_frame->ids = temp_ids;
        pre_frame->tracked_number = temp_tracked_n;
        pre_frame->time = temp_time; 
        correspondence.clear();
    }
}

void tracking::reject_with_F() {
    vector<cv::Point2f> kps_1, kps_2;
    kps_1.resize(temp_un_kps.size());
    kps_2.resize(temp_un_kps.size());
    for(size_t i=0 ;i<temp_un_kps.size(); i++) {
        int corr = correspondence[i];
        kps_1[i] = pre_frame->un_keypoints[corr];
        kps_2[i] = temp_un_kps[i];
        kps_1[i].x = kps_1[i].x * t_p->focal_length + t_p->image_col * 0.5;
        kps_1[i].y = kps_1[i].y * t_p->focal_length + t_p->image_row * 0.5;
        kps_2[i].x = kps_2[i].x * t_p->focal_length + t_p->image_col * 0.5;
        kps_2[i].y = kps_2[i].y * t_p->focal_length + t_p->image_row * 0.5;    
    }
    cv::findFundamentalMat(kps_1, kps_2, cv::FM_RANSAC, t_p->f_threshold, 0.99, status);
}

void tracking::remove_outlier() {
    int j=0;
    
    for(size_t i=0; i<temp_kps.size(); i++) {
        if(status[i]) {
            if(temp_kps[i].x <= 2 || temp_kps[i].x >= t_p->image_col - 2) {
                status[i] = 0;
            }
            if(temp_kps[i].y <= 2 || temp_kps[i].y >= t_p->image_row - 2) {
                status[i] = 0;
            } 
        }

        if(status[i]) {
            temp_kps[j] = temp_kps[i];
            cv::Point2f un_point;
            undistorted_keypoint(temp_kps[i], un_point);
            temp_un_kps.push_back(un_point);
            temp_ids.push_back(pre_frame->ids[i]);
            temp_tracked_n.push_back(pre_frame->tracked_number[i]+1);
            correspondence[j] = correspondence[i];
            j++;
        }
    }
    
    temp_kps.resize(j);
    correspondence.resize(j);
}

void tracking::remove_outlier_with_make_pair(vector<pair<int, int>> &pairs) {
    int j=0;
    
    for(size_t i=0; i<temp_kps.size(); i++) {
        if(status[i]) {
            if(temp_kps[i].x <= 2 || temp_kps[i].x >= t_p->image_col - 2) {
                status[i] = 0;
            }
            if(temp_kps[i].y <= 2 || temp_kps[i].y >= t_p->image_row - 2) {
                status[i] = 0;
            } 
        }
        
        if(status[i]) {
            temp_kps[j] = temp_kps[i];
            temp_un_kps[j] = temp_un_kps[i];
            temp_ids[j] = temp_ids[i];
            temp_tracked_n[j] = temp_tracked_n[i];
            correspondence[j] = correspondence[i];
            pairs.push_back(make_pair(temp_tracked_n[j], j));
            j++;
        }
    }

    sort(pairs.begin(), pairs.end(), [](const pair<int, int> &a, const pair<int, int> &b)
         {
            return a.first > b.first;
         });

    vector<cv::Point2f> temp_kps_;
    vector<cv::Point2f> temp_un_kps_;
    vector<int> temp_ids_;
    vector<int> temp_tracked_n_;
    vector<int> correspondence_;

    for(int i = 0; i < j; i++) {
        int k = pairs[i].second;
        cv::Point2f p;
        p.x = cvRound(temp_kps[k].x); 
        p.y = cvRound(temp_kps[k].y); 
        if(mask.at<uchar>(p) == 255) {
            temp_kps_.push_back(temp_kps[k]);
            temp_ids_.push_back(temp_ids[k]);
            temp_un_kps_.push_back(temp_un_kps[k]);
            temp_tracked_n_.push_back(temp_tracked_n[k]);
            correspondence_.push_back(correspondence[k]);

            cv::circle(mask, p, t_p->min_dis, 0, -1);
        }
    }

    temp_ids = temp_ids_;
    temp_kps = temp_kps_;
    temp_un_kps = temp_un_kps_;
    temp_tracked_n = temp_tracked_n_;
    correspondence = correspondence_;
}

/*
void tracking::add_keypoints_in_temp_v(int n) {

}

void tracking::add_keypoints_in_frame() {
 
}

void tracking::process(cv::Mat &image, double time, bool if_pub) {

}

void tracking::reject_with_F() {

}

void tracking::remove_outlier() {

}

void tracking::remove_outlier_with_make_pair() {

}*/