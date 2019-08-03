/*

*/

#ifndef ORBSLAM2_PROJECTMAP_H_
#define ORBSLAM2_PROJECTMAP_H_

// standard
#include <vector>
#include <string>

// ROS
#include <ros/ros.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"

// ORB_SLAM
#include "System.h"
#include "MapPoint.h"
#include "Converter.h"

// thirdparty
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>


namespace ProjectMap
{
    class Pub
    {
    public:
        Pub (ORB_SLAM2::System* orb_slam_ptr, ros::NodeHandle &node_handle);
        ~Pub ();

    protected:
        ORB_SLAM2::System* slam_ptr_;

    private:
        void GetAndPublishMsgs ();
        geometry_msgs::PoseArray GetAllKfsPts ();
        geometry_msgs::PoseArray GetSingleKfPts ();
        geometry_msgs::Pose TransformFromMapPt_orb2ros (const cv::Mat translation_);
        geometry_msgs::Pose TransformFromCamera_orb2ros (const cv::Mat translation_, cv::Mat rotation_);

        ros::NodeHandle node_handle_;
        ros::Publisher all_kfs_pts_publisher_;
        ros::Publisher single_kf_pts_publisher_;
        std::string name_of_node_;

        bool pub_all_pts_ = false;
        unsigned int all_pts_pub_gap_ = 0;
        unsigned int pub_count_ = 0;

        cv::Mat orb2ros = (cv::Mat_<int>(3,3) << 0,0,1,-1,0,0,0,-1,0);
    }
};



#endif
