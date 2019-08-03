/*

*/

#include <ProjectMap.h>


namespace ProjectMap
{
    Pub::Pub (ORB_SLAM2::System* orb_slam_ptr, ros::NodeHandle &node_handle)
    {
        node_handle_ = node_handle;
        slam_ptr_ = orb_slam_ptr;
        name_of_node_ = ros::this_node::getName();

        all_kfs_pts_publisher_ = node_handle_.advertise<geometry_msgs::PoseArray> (name_of_node_+"/all_kfs_pts", 1);
        single_kf_pts_publisher_ = node_handle_.advertise<geometry_msgs::PoseArray> (name_of_node_+"/single_kf_pts", 1);
    }

    void Pub::GetAndPublishMsgs ()
    {
        if ( (all_pts_pub_gap_ > 0) && (pub_count_ >= all_pts_pub_gap_) )
        {
            pub_all_pts_ = true;
            pub_count_ = 0;
        }
        if (pub_all_pts_ || slam_ptr_->getLoopClosing()->loop_detected || slam_ptr_->getTracker()->loop_detected)
        {
            pub_all_pts_ = slam_ptr_->getTracker()->loop_detected = slam_ptr_->getLoopClosing()->loop_detected = false;

            geometry_msgs::PoseArray kfs_pts_array_ = GetAllKfsPts ();
            all_kfs_pts_publisher_.publish( kfs_pts_array_ );
        }
        else if (slam_ptr_->getTracker()->mCurrentFrame.is_keyframe)
        {
            ++pub_count_;
            slam_ptr_->getTracker()->mCurrentFrame.is_keyframe = false;

            geometry_msgs::PoseArray kf_pts_array_ = GetSingleKfPts ();
            single_kf_pts_publisher_.publish( kf_pts_array_ );
        }
    }

    geometry_msgs::PoseArray Pub::GetAllKfsPts ()
    {
        // camera_pose, n_pts_in_this_kf, pts, ...
        geometry_msgs::PoseArray kfs_pts_array_;

        vector<ORB_SLAM2::KeyFrame*> key_frames_ = slam_ptr_->getMap()->GetAllKeyFrames();

        kfs_pts_array_.poses.push_back(geometry_msgs::Pose());
		sort(key_frames_.begin(), key_frames_.end(), ORB_SLAM2::KeyFrame::lId);
		unsigned int n_kf_ = 0;
        for (auto kf_ : key_frames_)
        {
            if ( kf_->isBad() ) continue;
            // get rotation information
			cv::Mat R_ = kf_->GetRotation().t();
            // get camera position
			cv::Mat T_ = kf_->GetCameraCenter();
            // convert to pose in world_ros, store it
            kfs_pts_array_.poses.push_back( TransformFromCamera_orb2ros(T_, R_) );

            /* ------------------------------------------------
			add position(x,y,z) of all map points of current key_frame
			------------------------------------------------ */
            unsigned int n_pts_id_ = kfs_pts_array_.poses.size();
			//! placeholder for number of points
            kfs_pts_array_.poses.push_back(geometry_msgs::Pose());
			std::set<ORB_SLAM2::MapPoint*> map_pts_ = kf_->GetMapPoints();
			unsigned int n_pts_ = 0;
            for (auto pt_ : map_pts_)
            {
                if ( !pt_ || pt_->isBad() ) continue;

                cv::Mat pt_position_ = pt_->GetWorldPos();
                if ( pt_position_.empty() ) continue;

                kfs_pts_array_.poses.push_back( TransformFromMapPt_orb2ros(pt_position_) );

                ++n_pts_;
            }
            kfs_pts_array_.poses[n_pts_id_].position.x = n_pts_;
            kfs_pts_array_.poses[n_pts_id_].position.y = n_pts_;
            kfs_pts_array_.poses[n_pts_id_].position.z = n_pts_;

            ++n_kf_;
        }
        kfs_pts_array_.poses[0].position.x = n_kf_;
        kfs_pts_array_.poses[0].position.y = n_kf_;
        kfs_pts_array_.poses[0].position.z = n_kf_;

        return kfs_pts_array_;
    }

    geometry_msgs::PoseArray Pub::GetSingleKfPts ()
    {
        ORB_SLAM2::KeyFrame* kf_ = slam_ptr_->getTracker()->mCurrentFrame.mpReferenceKF;

        // camera_pose, pts, ...
        geometry_msgs::PoseArray kf_pts_array_;
        // get rotation information
        cv::Mat R_ = kf_->GetRotation().t();
        // get camera position
        cv::Mat T_ = kf_->GetCameraCenter();

        kf_pts_array_.poses.push_back( TransformFromCamera_orb2ros(T_, R_) );

        std::vector<ORB_SLAM2::MapPoint*> map_pts_ = slam_ptr_->GetTrackedMapPoints();
        for (auto pt_ : map_pts_)
        {
            if ( !pt_ || pt_->isBad() ) continue;

            cv::Mat pt_position_ = pt_->GetWorldPos();
            if (pt_position_.empty()) continue;
            kf_pts_array_.poses.push_back( TransformFromMapPt_orb2ros(pt_position_) );
        }

        return kf_pts_array_;
    }

    geometry_msgs::Pose Pub::TransformFromMapPt_orb2ros (const cv::Mat translation_)
    {
        geometry_msgs::Pose pose_;

        pose_.position.x =        translation_.at<float>(2);
        pose_.position.y = (-1) * translation_.at<float>(0);
        pose_.position.z = (-1) * translation_.at<float>(1);

        return pose_;
    }

    geometry_msgs::Pose Pub::TransformFromCamera_orb2ros (const cv::Mat translation_, cv::Mat rotation_)
    {
        rotation_ = orb2ros * rotation_ * orb2ros.t();
        vector<float> q_ = ORB_SLAM2::Converter::toQuaternion(rotation_);

        geometry_msgs::Pose pose_;

        pose_.position.x =        translation_.at<float>(2);
        pose_.position.y = (-1) * translation_.at<float>(0);
        pose_.position.z = (-1) * translation_.at<float>(1);
        pose_.orientation.x = q_[0];
		pose_.orientation.y = q_[1];
		pose_.orientation.z = q_[2];
		pose_.orientation.w = q_[3];

        return pose_;
    }
}
