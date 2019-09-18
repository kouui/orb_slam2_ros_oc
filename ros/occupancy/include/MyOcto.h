/*

*/

#ifndef MYOCTO_H_
#define MYOCTO_H_

// octomap 
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>
//#include <octomap/ColorOcTree.h>
//#include <octomap/math/Pose6D.h>

// standard
#include <vector>
#include <string>
#include <utility>
#include <iostream>

// self-defined message
#include "orb_slam2_ros/SaveMap.h"

// ROS
#include <ros/ros.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/OccupancyGrid.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>


// thirdparty
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui_c.h>
//#include <opencv2/highgui/highgui.hpp>

namespace ns_myocto
{
    /*
    class grid2dmap
    {
    public :
        grid2dmap () { };
        ~grid2dmap () { };

        nav_msgs::OccupancyGrid map;

        double z_min_, z_max_;
        double thresOccupancyOccupied_, thresOccupancyFree_;

        // min --> minus value; max --> positive value
        double defaultPaddedMinX_, defaultPaddedMaxX_, defaultPaddedMinY_, defaultPaddedMaxY_;
        octomap::OcTreeKey paddedMinKey_, paddedMaxKey_;
        unsigned scale_;
        double paddedMinX_, paddedMaxX_, paddedMinY_, paddedMaxY_;


    };
    */

    class myocto
    {
    public:
        myocto (ros::NodeHandle &node_handle);
        ~myocto () 
        { 
            delete tree_;
            //delete ptr_g2d_; 
        };

        void PublishAllTopics_FixedRate ();

        bool publish_in_fixed_rate_;

    private:
        void GetROSParameter ();
        void InitializeTree ();
        /*
        void InitializeGrid2dmap ();
        octomap::OcTreeKey InitPaddedMinKey();
        octomap::OcTreeKey InitPaddedMaxKey();
        void ProcessKeyToGrid2dmap (octomap::OcTreeKey key);
        */
        void UpdatePoint (const octomap::point3d &camera_point3d, const octomap::point3d &map_point3d, octomap::KeySet &free_cells, octomap::KeySet &occupied_cells, bool isOutZLimit);
        void UpdateTree (octomap::KeySet &free_cells, octomap::KeySet &occupied_cells);
        void SingleCallback (const geometry_msgs::PoseArray::ConstPtr& kf_pts_array);
        void AllCallback (const geometry_msgs::PoseArray::ConstPtr& kfs_pts_array);
        void PublishAllTopics_WhenKFsCome ();
        void PublishFullOctomap (const ros::Time& rostime);
        void PublishBinaryOctomap (const ros::Time& rostime);
        //void PublishGrid2dmap (const ros::Time& rostime);
        void CheckSpeckleNode ();
        bool HaveNeighbor(const octomap::OcTreeKey &nKey);
        bool SaveOctreeSrv (orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res);

        ros::NodeHandle node_handle_;
        ros::Subscriber all_kfs_pts_subscriber_;
        ros::Subscriber single_kf_pts_subscriber_;
        ros::Publisher  binary_map_publisher_, full_map_publisher_;//, grid2dmap_publisher_;
        ros::ServiceServer service_server_;
        std::string name_of_node_;

        octomap::OcTree* tree_;
        octomap::KeyRay keyRay_;  // temp storage for ray casting


        bool loop_closure_being_processed_ = false;
        unsigned int n_kf_received_ = 0;

        std::string all_kfs_pts_topic_name_;
        std::string single_kf_pts_topic_name_;
        std::string frame_id_;
        std::string octree_load_file_name_;
        bool load_octree_;
        int fixed_publish_rate_;
        double resolution_;
        double z_min_, z_max_;
        double probHit_, probMiss_, thresMin_, thresMax_, thresOccupancy_;
        double rangeMax_;
        int multi_free_factor_;
        bool publish_topic_when_subscribed_;
        float setFreeVal_= -2.2; // p=0.1
        unsigned maxTreeDepth_;

        // 2d projected map
        //grid2dmap* ptr_g2d_;


    };
}

#endif
