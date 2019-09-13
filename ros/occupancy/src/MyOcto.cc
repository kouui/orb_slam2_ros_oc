
#include "MyOcto.h"

namespace ns_myocto
{
    myocto::myocto (ros::NodeHandle &node_handle)
    {
        node_handle_ = node_handle;
        name_of_node_ = ros::this_node::getName();

        GetROSParameter ();

        tree_ = new octomap::OcTree ( resolution_ );

        // subscriber
        single_kf_pts_subscriber_ = node_handle_.subscribe(single_kf_pts_topic_param_, 1000, &myocto::SingleCallback, this);
        all_kfs_pts_subscriber_ = node_handle_.subscribe(all_kfs_pts_topic_param_, 1000, &myocto::AllCallback, this);

        // publisher
    }

    void myocto::GetROSParameter ()
    {
        //static parameters
        node_handle_.param<std::string>(name_of_node_+"/all_kfs_pts_topic", all_kfs_pts_topic_param_, "/orb_slam2_mono/all_kfs_pts");
        node_handle_.param<std::string>(name_of_node_+"/single_kf_pts_topic", single_kf_pts_topic_param_, "/orb_slam2_mono/single_kf_pts");
        node_handle_.param<std::string>(name_of_node_+"/frame_id", frame_id_param_, "/map");
        node_handle_.param(name_of_node_+"/resolution", resolution_param_, 0.05);
        node_handle_.param(name_of_node_+"/z_min", z_min_param_, -0.1);
        node_handle_.param(name_of_node_+"/z_max", z_max_param_, 0.8);

        resolution_ = (double) resolution_param_;
        z_min_      = (double) z_min_param_;
        z_max_      = (double) z_max_param_;
    }

    void myocto::SingleCallback (const geometry_msgs::PoseArray::ConstPtr& kf_pts_array)
    {
        // loop closure takes time
        if (loop_closure_being_processed_) return;

        n_kf_received_++;
        std::cout << "Received " << n_kf_received_ << " frames.\n";

        unsigned int n_pts = kf_pts_array->poses.size() - 1;
        AddPointsToTree (kf_pts_array, n_pts, 1);

    }
    
    void myocto::AllCallback (const geometry_msgs::PoseArray::ConstPtr& kfs_pts_array)
    {

    }

    void myocto::AddPointsToTree (const geometry_msgs::PoseArray::ConstPtr& kf_pts_array, unsigned int n_pts, unsigned int start_id)
    {
        std::vector<geometry_msgs::Pose> pts = kf_pts_array->poses;
        unsigned int end_id = start_id + n_pts;
        
        for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id)
        {
            octomap::point3d mappoint ( (float) pts[pt_id].position.x, (float) pts[pt_id].position.y, (float) pts[pt_id].position.z );
            
            // integrate 'occupied' measurement
            tree_->updateNode(mappoint, true);
            
            // integrate 'free' measurement using Ray-tracing
        }
        
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "myocto");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;
    ns_myocto::myocto o3d (node_handle);

    ros::spin();

    ros::shutdown();

    return 0;
}