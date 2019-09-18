
#include "MyOcto.h"

namespace ns_myocto
{
    myocto::myocto (ros::NodeHandle &node_handle)
    {
        node_handle_ = node_handle;
        name_of_node_ = ros::this_node::getName();

        GetROSParameter ();

        InitializeTree ();

        //InitializeGrid2dmap ();

        // save octree service
        service_server_ = node_handle_.advertiseService(name_of_node_+"/save_octree", &myocto::SaveOctreeSrv, this);

        
        if (load_octree_)
        {
            std::cout << "loading octree binary file from " << octree_load_file_name_ << std::endl;
            bool load_success = tree_->readBinary(octree_load_file_name_);
            if (load_success)
            {
                std::cout << "loading octree binary file --> success " << std::endl;
                
                // with preload octree, we'd better publish it in a fixed rate
                publish_in_fixed_rate_ = true;
            }
            else
            {
                std::cout << "loading octree binary file --> failed " << std::endl;
                return;
            }
            
            
        }
        else
        {
            // subscriber
            single_kf_pts_subscriber_ = node_handle_.subscribe(single_kf_pts_topic_name_, 1000, &myocto::SingleCallback, this);
            all_kfs_pts_subscriber_ = node_handle_.subscribe(all_kfs_pts_topic_name_, 1000, &myocto::AllCallback, this);
        }

        // publisher
        full_map_publisher_ = node_handle_.advertise<octomap_msgs::Octomap>(name_of_node_+"/octomap_full", 1, true);
        binary_map_publisher_ = node_handle_.advertise<octomap_msgs::Octomap>(name_of_node_+"/octomap_binary", 1, true);
        //grid2dmap_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(name_of_node_+"/grid2dmap", 1, true);
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
        node_handle_.param(name_of_node_+"/sensor_model/occupancyThres", thresOccupancy_, 0.6);
        node_handle_.param(name_of_node_+"/sensor_model/hit", probHit_, 0.6);
        node_handle_.param(name_of_node_+"/sensor_model/miss", probMiss_, 0.4);
        node_handle_.param(name_of_node_+"/sensor_model/min", thresMin_, 0.10);
        node_handle_.param(name_of_node_+"/sensor_model/max", thresMax_, 0.90);
        node_handle_.param(name_of_node_+"/rangeMax", rangeMax_, 3.0);
        node_handle_.param(name_of_node_+"/multi_free_factor", multi_free_factor_, 1);
        node_handle_.param(name_of_node_+"/publish_topic_when_subscribed", publish_topic_when_subscribed_, true);
        node_handle_.param(name_of_node_+"/load_octree", load_octree_, false);
        node_handle_.param(name_of_node_+"/publish_in_fixed_rate", publish_in_fixed_rate_, false);
        node_handle_.param(name_of_node_+"/fixed_publish_rate", fixed_publish_rate_, 2);
        node_handle_.param<std::string>(name_of_node_+"/octree_load_file", octree_load_file_name_, "/home/ubuntu/savefiles/street_s60d850_zminm002_zmaxp030.bt");
    }

    void myocto::InitializeTree ()
    {
        tree_ = new octomap::OcTree ( resolution_ );
        tree_->setOccupancyThres(thresOccupancy_);
        tree_->setProbHit(probHit_);
        tree_->setProbMiss(probMiss_);
        tree_->setClampingThresMin(thresMin_);
        tree_->setClampingThresMax(thresMax_);

        maxTreeDepth_ = tree_->getTreeDepth();
    }
    /*
    void myocto::InitializeGrid2dmap ()
    {

        ptr_g2d_ = new grid2dmap ();
        
        node_handle_.param(name_of_node_+"/grid2dmap/default_padded_minX", ptr_g2d_->defaultPaddedMinX_, -3.0);
        node_handle_.param(name_of_node_+"/grid2dmap/default_padded_maxX", ptr_g2d_->defaultPaddedMaxX_, 3.0);
        node_handle_.param(name_of_node_+"/grid2dmap/default_padded_minY", ptr_g2d_->defaultPaddedMinY_, -3.0);
        node_handle_.param(name_of_node_+"/grid2dmap/default_padded_maxY", ptr_g2d_->defaultPaddedMaxY_, 3.0);
        node_handle_.param(name_of_node_+"/grid2dmap/threshold_occupancy_occupied", ptr_g2d_->thresOccupancyOccupied_, thresOccupancy_);
        node_handle_.param(name_of_node_+"/grid2dmap/threshold_occupancy_free", ptr_g2d_->thresOccupancyFree_, thresOccupancy_);
        assert (ptr_g2d_->thresOccupancyOccupied_ < ptr_g2d_->thresOccupancyFree_);

        ptr_g2d_->map.info.resolution = tree_->getResolution();
        ptr_g2d_->map.header.frame_id = frame_id_;      
        // ptr_g2d_->map.header.stamp = ?  // define the map before map being updated

        ptr_g2d_->paddedMinKey_ = InitPaddedMinKey ();
        ptr_g2d_->paddedMaxKey_ = InitPaddedMaxKey ();

        ptr_g2d_->scale_ = 1;
        ptr_g2d_->map.info.width = (ptr_g2d_->paddedMaxKey_[0] - ptr_g2d_->paddedMinKey_[0])/ptr_g2d_->scale_  + 1;
        ptr_g2d_->map.info.height = (ptr_g2d_->paddedMaxKey_[1] - ptr_g2d_->paddedMinKey_[1])/ptr_g2d_->scale_  + 1;


        octomap::point3d origin = tree_->keyToCoord(ptr_g2d_->paddedMinKey_, maxTreeDepth_);
        double gridSize = tree_->getResolution(); //
        //if (std::abs(gridSize-ptr_g2d_->map.info.resolution) > 1e-6)
        // we will keep working under maximum resolution, so the resolution of ptr_g2d_->map will not change
        ptr_g2d_->map.info.origin.position.x = origin.x() - gridSize*0.5;
        ptr_g2d_->map.info.origin.position.y = origin.y() - gridSize*0.5;

        ptr_g2d_->map.data.resize( ptr_g2d_->map.info.width*ptr_g2d_->map.info.height , 50 );
    }

    octomap::OcTreeKey myocto::InitPaddedMinKey()
    {

        double X, Y, Z;
        tree_->getMetricMin(X, Y, Z);
        octomap::point3d Pt (X, Y, Z);
        octomap::OcTreeKey Key0 = tree_->coordToKey(Pt, maxTreeDepth_);

        // campare with the given default paddings
        X = std::min( X, ptr_g2d_->defaultPaddedMinX_ );
        Y = std::min( Y, ptr_g2d_->defaultPaddedMinY_ );

        ptr_g2d_->paddedMinX_ = X;
        ptr_g2d_->paddedMinY_ = Y;
        

        Pt = octomap::point3d(X, Y, Z);
        octomap::OcTreeKey paddedKey;
        if ( !tree_->coordToKeyChecked(Pt, maxTreeDepth_, paddedKey) )
        {
            ROS_ERROR("Could not create padded min OcTree key at %f %f %f", Pt.x(), Pt.y(), Pt.z());
            paddedKey = Key0;
        }

        return paddedKey;
    }

    octomap::OcTreeKey myocto::InitPaddedMaxKey()
    {
        double X, Y, Z;
        tree_->getMetricMax(X, Y, Z);
        octomap::point3d Pt (X, Y, Z);
        octomap::OcTreeKey Key0 = tree_->coordToKey(Pt, maxTreeDepth_);

        // campare with the given default paddings
        X = std::max( X, ptr_g2d_->defaultPaddedMaxX_ );
        Y = std::max( Y, ptr_g2d_->defaultPaddedMaxY_ );

        ptr_g2d_->paddedMaxX_ = X;
        ptr_g2d_->paddedMaxY_ = Y;

        Pt = octomap::point3d(X, Y, Z);
        octomap::OcTreeKey paddedKey;
        if ( !tree_->coordToKeyChecked(Pt, maxTreeDepth_, paddedKey) )
        {
            ROS_ERROR("Could not create padded max OcTree key at %f %f %f", Pt.x(), Pt.y(), Pt.z());
            paddedKey = Key0;
        }

        return paddedKey;
    }

    void myocto::ProcessKeyToGrid2dmap (octomap::OcTreeKey key)
    {
        //int keySize = 1 << ( maxTreeDepth_ - key->getDepth() );

        //octomap::OcTreeKey minKey = key->getIndexKey();
    }
    */

    void myocto::UpdatePoint (const octomap::point3d &camera_point3d, const octomap::point3d &map_point3d, octomap::KeySet &free_cells, octomap::KeySet &occupied_cells, bool isOutZLimit)
    {
        // free on ray; occupied on endpoint;
        if ((rangeMax_ < 0.0) || ((map_point3d - camera_point3d).norm() <= rangeMax_) )
        {
            // free cells
            if (tree_->computeRayKeys(camera_point3d, map_point3d, keyRay_))
                free_cells.insert(keyRay_.begin(), keyRay_.end());

            // occupied endpoint
            octomap::OcTreeKey key;
            if ( tree_->coordToKeyChecked(map_point3d, key) )
            {
                switch (isOutZLimit)
                {
                    case true : // if outside zlimit, set its value to free
                        tree_->setNodeValue( key, setFreeVal_ , false);
                        break;

                    case false :
                        occupied_cells.insert(key);
                        break;
                }
            }     
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

    void myocto::UpdateTree (octomap::KeySet &free_cells, octomap::KeySet &occupied_cells)
    {
        // mark free cells only if not seen occupied in this cloud
        for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it)
        {
            //if (occupied_cells.find(*it) == occupied_cells.end()){
            for (auto i = 0; i < multi_free_factor_; i++)
                tree_->updateNode(*it, false, false);
            
            //ProcessKeyToGrid2dmap (*it);
            //}
        }

        // now mark all occupied cells:
        for (octomap::KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++)
            tree_->updateNode(*it, true, false);
    }

    void myocto::SingleCallback (const geometry_msgs::PoseArray::ConstPtr& kf_pts_array)
    {
        // loop closure takes time
        if (loop_closure_being_processed_) return;

        n_kf_received_++;
        //std::cout << "Received " << n_kf_received_ << " frames.\n";

        const geometry_msgs::Point cam_pt = kf_pts_array->poses[0].position;
        octomap::point3d camera_point3d ( (float) cam_pt.x, (float) cam_pt.y, (float) cam_pt.z );

        unsigned int n_pts = kf_pts_array->poses.size() - 1;
        unsigned int start_id = 1;
        unsigned int end_id = start_id + n_pts;
        
        octomap::KeySet free_cells, occupied_cells;
        for (unsigned int pi = start_id; pi < end_id; ++pi)
        {
            geometry_msgs::Point pt = kf_pts_array->poses[pi].position;

            // check moving difference between mappoint and camera_position in z direction
            float z_dif = (float) (pt.z - cam_pt.z); 
            if (z_dif > z_max_) continue;
            bool isOutZLimit = (z_dif < z_min_);
            if (isOutZLimit) continue;
            
            octomap::point3d map_point3d ( (float) pt.x, (float) pt.y, (float) pt.z );

            UpdatePoint (camera_point3d, map_point3d, free_cells, occupied_cells, isOutZLimit);
        }

        UpdateTree (free_cells, occupied_cells);

        // compress data by clipping leafs
        tree_->prune();

        // publish
        PublishAllTopics_WhenKFsCome ();
    }
    
    void myocto::AllCallback (const geometry_msgs::PoseArray::ConstPtr& kfs_pts_array)
    {
        loop_closure_being_processed_ = true;

        tree_->clear();
        unsigned int n_kf = kfs_pts_array->poses[0].position.x;
        std::cout << "Resetting grid map with " << n_kf << " key frames\n";

        octomap::KeySet free_cells, occupied_cells;
        unsigned int id = 0;
        for (auto kf_id = 0; kf_id < n_kf; ++kf_id)
        {
            const geometry_msgs::Point &cam_pt = kfs_pts_array->poses[++id].position;
            octomap::point3d camera_point3d ( (float) cam_pt.x, (float) cam_pt.y, (float) cam_pt.z );

            unsigned int n_pts = kfs_pts_array->poses[++id].position.x;
            unsigned int start_id = id + 1;
            unsigned int end_id = start_id + n_pts;

            
            for (unsigned int pi = start_id; pi < end_id; ++pi)
            {
                geometry_msgs::Point pt = kfs_pts_array->poses[pi].position;

                // check moving difference between mappoint and camera_position in z direction
                float z_dif = (float) (pt.z - cam_pt.z); 
                if (z_dif > z_max_) continue;
                bool isOutZLimit = (z_dif < z_min_);
                if (isOutZLimit) continue;
                
                octomap::point3d map_point3d ( (float) pt.x, (float) pt.y, (float) pt.z );

                UpdatePoint (camera_point3d, map_point3d, free_cells, occupied_cells, isOutZLimit);
            }

            

            id += n_pts;
        }
        UpdateTree (free_cells, occupied_cells);
        // check speckle node
        CheckSpeckleNode ();

        // compress data by clipping leafs
        tree_->prune();

        // publish
        PublishAllTopics_WhenKFsCome ();
        
        loop_closure_being_processed_ = false;
    }

    void myocto::PublishAllTopics_FixedRate ()
    {

        ros::Rate loop_rate (fixed_publish_rate_);

        while (ros::ok)
        {
            bool is_publishFullMap   = (!publish_topic_when_subscribed_ || (full_map_publisher_.getNumSubscribers() > 0) );
            bool is_publishBinaryMap = (!publish_topic_when_subscribed_ || (binary_map_publisher_.getNumSubscribers() > 0) );
            //bool is_publishGrid2dmap = (!publish_topic_when_subscribed_ || (grid2dmap_publisher_.getNumSubscribers() > 0) );

            const ros::Time rostime = ros::Time::now();
            
            if (is_publishFullMap)
            {
                PublishFullOctomap (rostime);
            }
            if (is_publishBinaryMap)
            {
                PublishBinaryOctomap (rostime);
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void myocto::PublishAllTopics_WhenKFsCome ()
    {
        bool is_publishFullMap   = (!publish_topic_when_subscribed_ || (full_map_publisher_.getNumSubscribers() > 0) );
        bool is_publishBinaryMap = (!publish_topic_when_subscribed_ || (binary_map_publisher_.getNumSubscribers() > 0) );
        //bool is_publishGrid2dmap = (!publish_topic_when_subscribed_ || (grid2dmap_publisher_.getNumSubscribers() > 0) );

        const ros::Time rostime = ros::Time::now();
        
        if (is_publishFullMap)
        {
            PublishFullOctomap (rostime);
        }
        if (is_publishBinaryMap)
        {
            PublishBinaryOctomap (rostime);
        }
        /*
        if (is_publishGrid2dmap)
        {
            PublishGrid2dmap (rostime);
        }
        */
    }
    

    void myocto::PublishFullOctomap (const ros::Time& rostime)
    {
        octomap_msgs::Octomap msg;
        msg.header.seq = n_kf_received_;
        msg.header.frame_id = frame_id_;
        msg.header.stamp = rostime;
        if (octomap_msgs::fullMapToMsg(*tree_, msg))
            full_map_publisher_.publish( msg );
        else
            ROS_ERROR("Error serializing OctoMap");
        
    }

    void myocto::PublishBinaryOctomap (const ros::Time& rostime)
    {
        octomap_msgs::Octomap msg;
        msg.header.seq = n_kf_received_;
        msg.header.frame_id = frame_id_;
        msg.header.stamp = rostime;
        if (octomap_msgs::binaryMapToMsg(*tree_, msg))
            binary_map_publisher_.publish( msg );
        else
            ROS_ERROR("Error serializing OctoMap");
        
    }
    /*
    void myocto::PublishGrid2dmap (const ros::Time& rostime)
    {
        ptr_g2d_->map.header.stamp = rostime;
        grid2dmap_publisher_.publish(ptr_g2d_->map); 
    }
    */

    void myocto::CheckSpeckleNode ()
    {
        for (octomap::OcTree::iterator it = tree_->begin(), end = tree_->end(); it != end; ++it)
        {
            if ( tree_->isNodeOccupied(*it) )
            {
                //std::cout << "node value : " << it->getValue() << it->getOccupancy()  << std::endl;
                if ( !HaveNeighbor( it.getKey() ) )
                {
                    tree_->setNodeValue( it.getKey(), setFreeVal_ , false); // this does kill occupied cells
                }

            }
        }
    }

    bool myocto::HaveNeighbor(const octomap::OcTreeKey &nKey)
    {
        octomap::OcTreeKey key;
        bool neighborFound = false;
        for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2])
            for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1])
                for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0])
                    if (key != nKey)
                    {
                        octomap::OcTreeNode* node = tree_->search(key);
                        if (node && tree_->isNodeOccupied(node))
                        {
                            // we have a neighbor => break!
                            neighborFound = true;
                        }
                    }
    
        return neighborFound;
    }

    bool myocto::SaveOctreeSrv (orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res) 
    {
        res.success = tree_->writeBinary(req.name);

        if (res.success)
            ROS_INFO_STREAM ("Octree object was saved as " << req.name);
        else
            ROS_ERROR ("Octree object could not be saved.");
        
        return res.success;
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

    if (o3d.publish_in_fixed_rate_)
    {
        o3d.PublishAllTopics_FixedRate ();
    }
    else
    {
        ros::spin();
    }

    ros::shutdown();

    return 0;
}