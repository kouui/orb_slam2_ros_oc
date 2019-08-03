#include <ProjectMap.h>


namespace ProjectMap
{
    Map::Map ()
    { }
}


int main(int argc, char const** argv)
{
    ros::init(argc, argv, "GridMap");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;

    ros::spin();

    ros::shutdown();

    return 0;
}
