
#include "MyOcto.h"

namespace ns_myocto
{
    myocto::myocto (ros::NodeHandle &node_handle)
    {
        node_handle_ = node_handle;
        name_of_node_ = ros::this_node::getName();

        GetROSParameter ();

        InitializeTree ();


        // subscriber
        single_kf_pts_subscriber_ = node_handle_.subscribe(single_kf_pts_topic_name_, 1000, &myocto::SingleCallback, this);
        all_kfs_pts_subscriber_ = node_handle_.subscribe(all_kfs_pts_topic_name_, 1000, &myocto::AllCallback, this);

        // publisher
        full_map_publisher_ = node_handle_.advertise<octomap_msgs::Octomap>(name_of_node_+"/octomap_full", 1, true);
        binary_map_publisher_ = node_handle_.advertise<octomap_msgs::Octomap>(name_of_node_+"/octomap_binary", 1, true);
    }

    void myocto::GetROSParameter ()
    {
        //static parameters
        node_handle_.param<std::string>(name_of_node_+"/all_kfs_pts_topic", all_kfs_pts_topic_name_, "/orb_slam2_mono/all_kfs_pts");
        node_handle_.param<std::string>(name_of_node_+"/single_kf_pts_topic", single_kf_pts_topic_name_, "/orb_slam2_mono/single_kf_pts");
        node_handle_.param<std::string>(name_of_node_+"/frame_id", frame_id_, "/map");
        node_handle_.param(name_of_node_+"/resolution", resolution_, 0.05);
        node_handle_.param(name_of_node_+"/z_min", z_min_, -0.1);
        node_handle_.param(name_of_node_+"/z_max", z_max_, 0.8);
        node_handle_.param("sensor_model/hit", probHit_, 0.7);
        node_handle_.param("sensor_model/miss", probMiss_, 0.4);
        node_handle_.param("sensor_model/min", thresMin_, 0.08);
        node_handle_.param("sensor_model/max", thresMax_, 0.97);
        node_handle_.param("rangeMax", rangeMax_, 4.0);
    }

    void myocto::InitializeTree ()
    {
        tree_ = new octomap::OcTree ( resolution_ );
        tree_->setProbHit(probHit_);
        tree_->setProbMiss(probMiss_);
        tree_->setClampingThresMin(thresMin_);
        tree_->setClampingThresMax(thresMax_);
    }

    void myocto::SingleCallback (const geometry_msgs::PoseArray::ConstPtr& kf_pts_array)
    {
        // loop closure takes time
        if (loop_closure_being_processed_) return;

        n_kf_received_++;
        std::cout << "Received " << n_kf_received_ << " frames.\n";

        const geometry_msgs::Point cam_pt = kf_pts_array->poses[0].position;
        octomap::point3d camera_point3d ( (float) cam_pt.x, (float) cam_pt.y, (float) cam_pt.z );

        unsigned int n_pts = kf_pts_array->poses.size() - 1;
        unsigned int start_id = 1;
        unsigned int end_id = start_id + n_pts;
        //const std::vector<geometry_msgs::Pose> v_mappoint ( kf_pts_array->poses.cbegin()+start_id, kf_pts_array->poses.cbegin()+end_id+1 );
        
        octomap::KeySet free_cells, occupied_cells;
        for (unsigned int pi = start_id; pi < end_id; ++pi)
        {
            geometry_msgs::Point pt = kf_pts_array->poses[pi].position;

            // check moving difference between mappoint and camera_position in z direction
            float z_dif = (float) (pt.z - cam_pt.z); 
            if ( (z_dif > z_max_) || (z_dif < z_min_) ) continue;
            
            octomap::point3d map_point3d ( (float) pt.x, (float) pt.y, (float) pt.z );

            // free on ray; occupied on endpoint;
            if ((rangeMax_ < 0.0) || ((map_point3d - camera_point3d).norm() <= rangeMax_) )
            {
                // free cells
                if (tree_->computeRayKeys(camera_point3d, map_point3d, keyRay_))
                    free_cells.insert(keyRay_.begin(), keyRay_.end());

                // occupied endpoint
                octomap::OcTreeKey key;
                if (tree_->coordToKeyChecked(map_point3d, key))
                    occupied_cells.insert(key);      
            }
            else // ray longer than maxrange
            {
                octomap::point3d newend_point3d = camera_point3d + (map_point3d - camera_point3d).normalized() * rangeMax_;

                // free cells
                if (tree_->computeRayKeys(camera_point3d, newend_point3d, keyRay_))
                    free_cells.insert(keyRay_.begin(), keyRay_.end());

                // free endpoint
                octomap::OcTreeKey key;
                if (tree_->coordToKeyChecked(newend_point3d, key))
                    free_cells.insert(key); 
            }
        }

        // mark free cells only if not seen occupied in this cloud
        for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it)
        {
            //if (occupied_cells.find(*it) == occupied_cells.end()){
            tree_->updateNode(*it, false);
            //}
        }

        // now mark all occupied cells:
        for (octomap::KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++)
            tree_->updateNode(*it, true);

        // publish
        bool is_publishFullMap = true;//( full_map_publisher.getNumSubscribers() > 0);
        bool is_publishBinaryMap = true;//( binary_map_publisher.getNumSubscribers() > 0);
        
        if (is_publishFullMap)
        {
            PublishFullOctomap ();
        }
        if (is_publishBinaryMap)
        {
            PublishBinaryOctomap ();
        }
    }
    
    void myocto::AllCallback (const geometry_msgs::PoseArray::ConstPtr& kfs_pts_array)
    {

    }

    void myocto::PublishFullOctomap ()
    {
        octomap_msgs::Octomap msg;
        msg.header.seq = n_kf_received_;
        msg.header.frame_id = frame_id_;
        msg.header.stamp = ros::Time::now();
        if (octomap_msgs::fullMapToMsg(*tree_, msg))
            full_map_publisher_.publish( msg );
        else
            ROS_ERROR("Error serializing OctoMap");
        
    }

    void myocto::PublishBinaryOctomap ()
    {
        octomap_msgs::Octomap msg;
        msg.header.seq = n_kf_received_;
        msg.header.frame_id = frame_id_;
        msg.header.stamp = ros::Time::now();
        if (octomap_msgs::binaryMapToMsg(*tree_, msg))
            binary_map_publisher_.publish( msg );
        else
            ROS_ERROR("Error serializing OctoMap");
        
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