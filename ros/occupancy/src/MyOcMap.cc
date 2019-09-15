
#include "MyOcMap.h"

namespace ns_myocmap
{
    myocmap::myocmap (ros::NodeHandle &node_handle)
    {
        node_handle_ = node_handle;
        name_of_node_ = ros::this_node::getName();

        GetROSParameter ();

        // subscriber
        octomap_full_subscriber_ = node_handle_.subscribe(octomap_full_topic_name_, 5, &myocmap::OctomapFullCallback, this);
        // publisher
        occupancy_map_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(name_of_node_+"/occupancy_map", 1, true);

    }

    void myocmap::GetROSParameter ()
    {
        node_handle_.param<std::string>(name_of_node_+"/octomap_full_topic", octomap_full_topic_name_, "/myocto/octomap_full");
        node_handle_.param(name_of_node_+"/threshold_occupancy_occupied", thresOccupancyOccupied_, 0.5);
        node_handle_.param(name_of_node_+"/threshold_occupancy_free", thresOccupancyFree_, 0.5);
        node_handle_.param(name_of_node_+"/publish_topic_when_subscribed", publish_topic_when_subscribed_, true);
        node_handle_.param<std::string>(name_of_node_+"/mode", mode_, "maximum");

    }

    void myocmap::OctomapFullCallback (const octomap_msgs::OctomapConstPtr& msg)
    {
        //std::cout << "step 1 --> ";
        // creating octree
        octomap::OcTree* octomap = NULL;
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);

        //std::cout << "step 2 --> ";

        if (tree)
            octomap = dynamic_cast<octomap::OcTree*>(tree);

        //std::cout << "step 3 --> ";
        
        if (!octomap)
        {
            ROS_ERROR("failed to create octree structure");
            return;
        }

        //std::cout << "step 4 --> ";

        // get dimensions of octree
        double minX, minY, minZ, maxX, maxY, maxZ;
        octomap->getMetricMin(minX, minY, minZ);
        octomap->getMetricMax(maxX, maxY, maxZ);
        octomap::point3d minPt = octomap::point3d(minX, minY, minZ);

        unsigned int tree_depth = octomap->getTreeDepth();

        //std::cout << "step 5 --> ";

        octomap::OcTreeKey paddedMinKey = octomap->coordToKey(minPt);

        //nav_msgs::OccupancyGrid::Ptr occupancy_map (new nav_msgs::OccupancyGrid());

        unsigned int width, height;
        double res;

        unsigned int ds_shift = 0;//tree_depth - max_octree_depth_;

        occupancy_map.header = msg->header;
        occupancy_map.info.resolution = res = octomap->getNodeSize(tree_depth);//(max_octree_depth_);
        occupancy_map.info.width = width = (maxX-minX) / res + 1;
        occupancy_map.info.height = height = (maxY-minY) / res + 1;
        occupancy_map.info.origin.position.x = minX  - (res / (float)(1<<ds_shift) ) + res;
        occupancy_map.info.origin.position.y = minY  - (res / (float)(1<<ds_shift) );

        occupancy_map.data.clear();
        occupancy_map.data.resize(width*height, -1);

        //std::cout << "step 6 --> ";

        // traverse all leafs in the tree:
        //unsigned int treeDepth = std::min<unsigned int>(max_octree_depth_, octomap->getTreeDepth());
        for (octomap::OcTree::iterator it = octomap->begin(tree_depth), end = octomap->end(); it != end; ++it)
        {
            double p = it->getOccupancy();
            bool occupied = octomap->isNodeOccupied(*it);
            int intSize = 1 << (tree_depth - it.getDepth());

            octomap::OcTreeKey minKey=it.getIndexKey();

            for (int dx = 0; dx < intSize; dx++)
                for (int dy = 0; dy < intSize; dy++)
                {
                    int posX = std::max<int>(0, minKey[0] + dx - paddedMinKey[0]);
                    posX>>=ds_shift;

                    int posY = std::max<int>(0, minKey[1] + dy - paddedMinKey[1]);
                    posY>>=ds_shift;

                    int idx = width * posY + posX;
                    if (mode_=="default")
                    {
                        if (occupied)
                        occupancy_map.data[idx] = 100;
                        else if (occupancy_map.data[idx] == -1)
                        {
                        occupancy_map.data[idx] = 0;
                    }
                    }
                    if (mode_=="maximum")
                    {
                        occupancy_map.data[idx] = std::max<int>( occupancy_map.data[idx], (int) (p*100) );
                    }
                }
        }

        //std::cout << "step 7 --> ";

        delete octomap;
        //std::cout << "step 8 --> ";
        occupancy_map_publisher_.publish(occupancy_map);
        //std::cout << "step 9 --> " << std::endl;
        //delete occupancy_map;

    }


}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "myocmap");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;
    ns_myocmap::myocmap g2d (node_handle);

    ros::spin();

    ros::shutdown();

    return 0;
}

