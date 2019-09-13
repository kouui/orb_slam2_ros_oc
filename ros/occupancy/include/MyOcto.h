/*

*/

#ifndef MYOCTO_H_
#define MYOCTO_H_

// octomap 
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
//#include <octomap/ColorOcTree.h>
//#include <octomap/math/Pose6D.h>

// standard
#include <vector>
#include <string>
#include <utility>
#include <iostream>

// ROS
#include <ros/ros.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"


// thirdparty
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>

namespace ns_myocto
{
    class myocto
    {
    public:
        myocto (ros::NodeHandle &node_handle);
        ~myocto () { delete tree_; };

    private:
        void GetROSParameter ();
        void SingleCallback (const geometry_msgs::PoseArray::ConstPtr& kf_pts_array);
        void AllCallback (const geometry_msgs::PoseArray::ConstPtr& kfs_pts_array);
        void AddPointsToTree (const geometry_msgs::PoseArray::ConstPtr& kf_pts_array, unsigned int n_pts, unsigned int start_id);


        ros::NodeHandle node_handle_;
        ros::Subscriber all_kfs_pts_subscriber_;
        ros::Subscriber single_kf_pts_subscriber_;
        std::string name_of_node_;

        octomap::OcTree* tree_;


        bool loop_closure_being_processed_ = false;
        unsigned int n_kf_received_ = 0;
        double resolution_;
        double z_min_, z_max_;

        std::string all_kfs_pts_topic_param_;
        std::string single_kf_pts_topic_param_;
        std::string frame_id_param_;
        double resolution_param_;
        double z_max_param_, z_min_param_;

    };
}

#endif
