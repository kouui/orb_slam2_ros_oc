/*

*/

#ifndef MYOCMAP_H_
#define MYOCMAP_H_

// standard
#include <iostream>
#include <string>

// ROS
#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>




namespace ns_myocmap
{
    class myocmap
    {
    public:
        myocmap (ros::NodeHandle &node_handle);
        ~myocmap () { };

    private:

        void GetROSParameter ();
        void OctomapFullCallback (const octomap_msgs::OctomapConstPtr& msg);

        ros::NodeHandle node_handle_;
        ros::Subscriber octomap_full_subscriber_;
        ros::Publisher  occupancy_map_publisher_;
        std::string name_of_node_;
        nav_msgs::OccupancyGrid occupancy_map;

        std::string octomap_full_topic_name_;
        bool publish_topic_when_subscribed_;
        double thresOccupancyOccupied_, thresOccupancyFree_;
        std::string mode_;

        static const std::size_t max_octree_depth_ = sizeof(unsigned short) * 8;

    };
}

#endif