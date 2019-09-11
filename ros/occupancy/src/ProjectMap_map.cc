#include <ProjectMap.h>


namespace ProjectMap
{
    Map::Map (ros::NodeHandle &node_handle)
    {
        node_handle_ = node_handle;
        name_of_node_ = ros::this_node::getName();

        GetROSParameter ();

        // subscriber
        single_kf_pts_subscriber_ = node_handle_.subscribe(single_kf_pts_topic_param_, 1000, &Map::SingleCallback, this);
        all_kfs_pts_subscriber_ = node_handle_.subscribe(all_kfs_pts_topic_param_, 1000, &Map::AllCallback, this);
        // publisher
        if (publish_grid_map_cost_param_)
        { grid_map_cost_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid> (name_of_node_+"/grid_map_cost", 1000); }
        if (publish_grid_map_visual_param_)
        { grid_map_visual_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid> (name_of_node_+"/grid_map_visual", 1000); }

        SetGridParameter ();
        CreateCvMat (h_, w_);

        if (use_keyboardUI_param_) KeyboardUI ();
    }

    void Map::GetROSParameter ()
    {
        //static parameters
        node_handle_.param<std::string>(name_of_node_+"/all_kfs_pts_topic", all_kfs_pts_topic_param_, "/orb_slam2_mono/all_kfs_pts");
        node_handle_.param<std::string>(name_of_node_+"/single_kf_pts_topic", single_kf_pts_topic_param_, "/orb_slam2_mono/single_kf_pts");
        node_handle_.param<std::string>(name_of_node_+"/frame_id", frame_id_param_, "/map");
        node_handle_.param(name_of_node_+"/publish_grid_map_visual", publish_grid_map_visual_param_, true);
        node_handle_.param(name_of_node_+"/publish_grid_map_cost", publish_grid_map_cost_param_, true);
        node_handle_.param(name_of_node_+"/use_keyboardUI", use_keyboardUI_param_, false);
        node_handle_.param(name_of_node_+"/free_thresh", free_thresh_param_, 0.6);
        node_handle_.param(name_of_node_+"/occupied_thresh", occupied_thresh_param_, 0.4);
        node_handle_.param(name_of_node_+"/scale_factor", scale_factor_param_, 4.0);
        node_handle_.param(name_of_node_+"/cloud_min_x", cloud_min_x_param_, -5.0);
        node_handle_.param(name_of_node_+"/cloud_max_x", cloud_max_x_param_, 16.0);
        node_handle_.param(name_of_node_+"/cloud_min_y", cloud_min_y_param_, -10.0);
        node_handle_.param(name_of_node_+"/cloud_max_y", cloud_max_y_param_, 10.0);
        node_handle_.param(name_of_node_+"/visit_thresh", visit_thresh_param_, 0);

        free_thresh_         = (float) free_thresh_param_;
        occupied_thresh_     = (float) occupied_thresh_param_;
        scale_fac_           = (float) scale_factor_param_;
        cloud_lim_[0]        = (float) cloud_min_x_param_;
        cloud_lim_[1]        = (float) cloud_max_x_param_;
        cloud_lim_[2]        = (float) cloud_min_y_param_;
        cloud_lim_[3]        = (float) cloud_max_y_param_;
        visit_thresh_        = (unsigned int) visit_thresh_param_;
    }

    void Map::SetGridParameter ()
    {
        for (auto i=0; i<4; i++)
            grid_lim_[i] = cloud_lim_[i] * scale_fac_;

        h_ = grid_lim_[1] - grid_lim_[0];
        w_ = grid_lim_[3] - grid_lim_[2];

        norm_fac_[0] = float(h_ - 1) / float(h_);
        norm_fac_[1] = float(w_ - 1) / float(w_);
    }

    void Map::CreateCvMat (const unsigned int h, const unsigned int w)
    {
        global_occupied_counter_.create(h, w, CV_32SC1);
        global_visit_counter_.create(h, w, CV_32SC1);
        global_occupied_counter_.setTo(cv::Scalar(0));
        global_visit_counter_.setTo(cv::Scalar(0));

        grid_map_.create(h, w, CV_32FC1);
        if (publish_grid_map_cost_param_)
        {
            grid_map_cost_msg_.data.resize(h*w);
            grid_map_cost_msg_.info.width = w;
            grid_map_cost_msg_.info.height = h;
            grid_map_cost_msg_.header.frame_id = frame_id_param_;
            grid_map_cost_msg_.info.resolution = 1.0/scale_fac_;
            SetGridOrigin (grid_map_cost_msg_, grid_lim_[0], grid_lim_[2]);
            grid_map_int_ = cv::Mat(h, w, CV_8SC1, (char*)(grid_map_cost_msg_.data.data()));
        }
        if (publish_grid_map_visual_param_)
        {
            grid_map_visual_msg_.data.resize(h*w);
            grid_map_visual_msg_.info.width = w;
            grid_map_visual_msg_.info.height = h;
            grid_map_visual_msg_.header.frame_id = frame_id_param_;
            grid_map_visual_msg_.info.resolution = 1.0/scale_fac_;
            SetGridOrigin (grid_map_visual_msg_, grid_lim_[0], grid_lim_[2]);
            grid_map_thresh_ = cv::Mat(h, w, CV_8SC1, (char*)(grid_map_visual_msg_.data.data()));
        }

        //grid_map_thresh_resized_.create(h * resize_fac_, w * resize_fac_, CV_8UC1);

        local_occupied_counter_.create(h, w, CV_32SC1);
        local_visit_counter_.create(h, w, CV_32SC1);
        local_map_pt_mask_.create(h, w, CV_8UC1);
    }

    void Map::PublishTopic (const ros::Publisher &pub, nav_msgs::OccupancyGrid &msg)
    {
        msg.info.map_load_time = ros::Time::now();
        pub.publish( msg );
    }

    void Map::SingleCallback (const geometry_msgs::PoseArray::ConstPtr& kf_pts_array)
    {
        // loop closure takes time
        if (loop_closure_being_processed_) return;

        UpdateGridMap( kf_pts_array );
        //std::cout << "Received " << n_kf_received_ << " frames.\n"
        //grid_map_msg_.info.map_load_time = ros::Time::now();
        //grid_map_publisher_.publish( grid_map_msg_ );
        if (publish_grid_map_visual_param_)
        { PublishTopic (grid_map_visual_publisher_, grid_map_visual_msg_); }
        if (publish_grid_map_cost_param_)
        { PublishTopic (grid_map_cost_publisher_, grid_map_cost_msg_); }
        map_count_++;
        std::cout << "published in Single Callback" << " : " << map_count_ << std::endl;
    }

    void Map::UpdateGridMap (const geometry_msgs::PoseArray::ConstPtr& kf_pts_array)
    {
        const geometry_msgs::Point &kf_position = kf_pts_array->poses[0].position;
        kf_pos_x_ = kf_position.x * scale_fac_;
        kf_pos_y_ = kf_position.y * scale_fac_;
        kf_pos_grid_x_ = int( floor( (kf_pos_x_ - grid_lim_[0]) * norm_fac_[0] ) );
        kf_pos_grid_y_ = int( floor( (kf_pos_y_ - grid_lim_[2]) * norm_fac_[1] ) );

        if (kf_pos_grid_x_ < 0 || kf_pos_grid_x_ >= h_) return;
        if (kf_pos_grid_y_ < 0 || kf_pos_grid_y_ >= w_) return;

        ++n_kf_received_;

        unsigned int n_pts = kf_pts_array->poses.size() - 1;
        ProcessPts(kf_pts_array->poses, n_pts, 1);
        GetGridMap();
    }

    void Map::ProcessPts (const std::vector<geometry_msgs::Pose> &pts, unsigned int n_pts, unsigned int start_id)
    {
        unsigned int end_id = start_id + n_pts;

        if (use_local_counter_)
        {
            local_map_pt_mask_.setTo(0);
            local_occupied_counter_.setTo(0);
            local_visit_counter_.setTo(0);

            for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id)
                ProcessPt(pts[pt_id].position, local_occupied_counter_, local_visit_counter_, local_map_pt_mask_);
            for (int row = 0; row < h_; ++row)
            {
                for (int col = 0; col < w_; ++col)
                {
                    if (local_map_pt_mask_.at<uchar>(row, col) == 0)
                    {
                        local_occupied_counter_.at<int>(row, col) = 0;
                    }
                    else
                    {
                        local_occupied_counter_.at<int>(row, col) = local_visit_counter_.at<int>(row, col);
                    }
                }
            }
            global_occupied_counter_ += local_occupied_counter_;
            global_visit_counter_ += local_visit_counter_;
        }
        else
        {
            for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id)
                ProcessPt(pts[pt_id].position, global_occupied_counter_, global_visit_counter_, local_map_pt_mask_);
        }
    }

    void Map::ProcessPt (const geometry_msgs::Point &curr_pt, cv::Mat &occupied, cv::Mat &visited, cv::Mat &pt_mask)
    {
        float pt_pos_x = curr_pt.x * scale_fac_;
        float pt_pos_y = curr_pt.y * scale_fac_;

        int pt_pos_grid_x = int(floor((pt_pos_x - grid_lim_[0]) * norm_fac_[0]));
        int pt_pos_grid_y = int(floor((pt_pos_y - grid_lim_[2]) * norm_fac_[1]));

        if ( pt_pos_grid_x < 0 || pt_pos_grid_x >= h_ ) return;
        if ( pt_pos_grid_y < 0 || pt_pos_grid_y >= w_ ) return;

        ++occupied.at<int>(pt_pos_grid_x, pt_pos_grid_y);
        pt_mask.at<uchar>(pt_pos_grid_x, pt_pos_grid_y) = 255;

        int x0 = kf_pos_grid_x_;
        int y0 = kf_pos_grid_y_;
        int x1 = pt_pos_grid_x;
        int y1 = pt_pos_grid_y;

        bool steep = ( abs(y1-y0) > abs(x1-x0) );
        if (steep)
        {
            std::swap(x0, y0);
            std::swap(x1, y1);
        }
        if (x0 > x1)
        {
            std::swap(x0, x1);
            std::swap(y0, y1);
        }

        int dx = x1 - x0;
        int dy = abs(y1 - y0);
        double error = 0;
        double deltaerr = ((double)dy) / ((double)dx);
        int y = y0;
    	int ystep = (y0 < y1) ? 1 : -1;
    	for (int x = x0; x <= x1; ++x)
        {
    		if (steep)
            {
    			++visited.at<int>(y, x);
    		}
    		else {
    			++visited.at<int>(x, y);
    		}
    		error = error + deltaerr;

            if (error >= 0.5){
    			y += ystep;
    			error -= - 1.0;
    		}
    	}
    }

    void Map::GetGridMap ()
    {
        for (int row = 0; row < h_; ++row)
        {
            for (int col = 0; col < w_; ++col)
            {
                int visits    = global_visit_counter_.at<int>(row, col);
                int occupieds = global_occupied_counter_.at<int>(row, col);

                grid_map_.at<float>(row, col) = (visits <= visit_thresh_) ? 0.5 :  (1.0 - float(occupieds / visits));
                if (publish_grid_map_visual_param_)
                {
                    if (grid_map_.at<float>(row, col) >= free_thresh_)
                    { grid_map_thresh_.at<uchar>(row, col) = 255; }
                    else if (grid_map_.at<float>(row, col) < occupied_thresh_)
                    { grid_map_thresh_.at<uchar>(row, col) = 0; }
                    else
                    { grid_map_thresh_.at<uchar>(row, col) = 128; }
                }
                if (publish_grid_map_cost_param_)
                { grid_map_int_.at<char>(row, col) = (1 - grid_map_.at<float>(row, col)) * 100; }

            }
        }
        // cv::resize(grid_map_thresh_, grid_map_thresh_resized_, grid_map_thresh_resized_.size());
    }

    void Map::AllCallback (const geometry_msgs::PoseArray::ConstPtr& kfs_pts_array)
    {
        loop_closure_being_processed_ = true;
        ResetGridMap( kfs_pts_array );

        if (publish_grid_map_visual_param_)
        { PublishTopic (grid_map_visual_publisher_, grid_map_visual_msg_); }
        if (publish_grid_map_cost_param_)
        { PublishTopic (grid_map_cost_publisher_, grid_map_cost_msg_); }
        std::cout << "published in All Callback" << std::endl;

        loop_closure_being_processed_ = false;
    }

    void Map::ResetGridMap (const geometry_msgs::PoseArray::ConstPtr& kfs_pts_array)
    {
        global_visit_counter_.setTo(0);
        global_occupied_counter_.setTo(0);

        unsigned int n_kf = kfs_pts_array->poses[0].position.x;

        std::cout << "Resetting grid map with" << n_kf << "key frames\n";

        unsigned int id = 0;
        for (auto kf_id = 0; kf_id < n_kf; ++kf_id)
        {
            const geometry_msgs::Point &kf_position = kfs_pts_array->poses[++id].position;
            unsigned int n_pts = kfs_pts_array->poses[++id].position.x;

            kf_pos_x_ = kf_position.x * scale_fac_;
            kf_pos_y_ = kf_position.y * scale_fac_;

            kf_pos_grid_x_ = int( floor( (kf_pos_x_ - grid_lim_[0]) * norm_fac_[0] ) );
            kf_pos_grid_y_ = int( floor( (kf_pos_y_ - grid_lim_[2]) * norm_fac_[1] ) );

            if (kf_pos_grid_x_ < 0 || kf_pos_grid_x_ >= h_) return;
            if (kf_pos_grid_y_ < 0 || kf_pos_grid_y_ >= w_) return;

            if (id + n_pts >= kfs_pts_array->poses.size())
            {
                std::cout << "resetGridMap :: Unexpected end of the input array while processing keyframe ";
                std::cout << kf_id << " with " <<  n_pts <<  " points: only ";
                std::cout << kfs_pts_array->poses.size() << " out of " << id + n_pts << " elements found\n";
                return;
            }

            ProcessPts(kfs_pts_array->poses, n_pts, id+1);
            id += n_pts;
        }
        GetGridMap();
        std::cout << "Completed resetting grid map.\n";
    }

    void Map::KeyboardUI () // not working; because it needs cv::imshow to open another thread
    {
        int key = cv::waitKey(1) % 256;
    	if (key == 'D') { cv::destroyAllWindows(); }
    	else if (key == 'f')
        {
    		free_thresh_ -= thresh_diff_;
    		if (free_thresh_ <= occupied_thresh_)
            { free_thresh_ = occupied_thresh_ + thresh_diff_; }

            std::cout << "Setting free_thresh_ to: " << free_thresh_ <<"\n";
    	}
    	else if (key == 'F')
        {
    		free_thresh_ += thresh_diff_;
    		if (free_thresh_ > 1) { free_thresh_ = 1; }

    		std::cout << "Setting free_thresh_ to: " << free_thresh_ <<"\n";
    	}
    	else if (key == 'o')
        {
    		occupied_thresh_ -= thresh_diff_;
    		if (free_thresh_ < 0){ free_thresh_ = 0; }

            std::cout << "Setting occupied_thresh_ to: " << occupied_thresh_ <<"\n";
    	}
    	else if (key == 'O')
        {
    		occupied_thresh_ += thresh_diff_;
    		if (occupied_thresh_ >= free_thresh_)
            { occupied_thresh_ = free_thresh_ - thresh_diff_; }

            std::cout << "Setting occupied_thresh_ to: " << occupied_thresh_ <<"\n";
    	}
    }

    void Map::SetGridOrigin (nav_msgs::OccupancyGrid &grid, float &grid_min_x, float &grid_min_y)
    {
        grid.info.origin.orientation.x = 1;
        grid.info.origin.orientation.y = 0;
        grid.info.origin.orientation.z = 0;
        grid.info.origin.orientation.w = 0;
        grid.info.origin.position.x = grid_min_x * grid.info.resolution ;
        grid.info.origin.position.y = grid_min_y * grid.info.resolution ;
        grid.info.origin.position.z = 0;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "GridMap");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;
    ProjectMap::Map map (node_handle);

    ros::spin();

    ros::shutdown();

    return 0;
}
