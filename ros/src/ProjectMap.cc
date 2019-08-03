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

        all_kfs_pts_publisher_ = node_handle_.advertise<geometry_msgs/PoseArray> (name_of_node_+"/all_kfs_pts", 1);
        single_kf_pts_publisher_ = node_handle_.advertise<geometry_msgs/PoseArray> (name_of_node_+"/single_kf_pts", 1);
    }

    Pub::GetAndPublishMsgs ()
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

    Pub GetAllKfsPts ()
    {

    }

    Pub GetSingleKfPts ()
    {
        
    }

    geometry_msgs::Pose Pub::TransformFromMapPt_orb2ros (const cv::Mat translation_)
    {
        geometry_msgs::Pose pose_;

        pose_.position.x = translation.at<float>(2);
        pose_.position.y = (-1) * translation.at<float>(0);
        pose_.position.z = (-1) * translation.at<float>(1);

        return pose_;
    }

    geometry_msgs::Pose Pub::TransformFromCaerma_orb2ros (const cv::Mat translation_, cv::Mat rotation_)
    {
        rotation_ = orb2ros * rotation_ * orb2ros.t();
        vector<float> q_ = ORB_SLAM2::Converter::toQuaternion(rotation_);

        geometry_msgs::Pose pose_;

        pose_.position.x = translation.at<float>(2);
        pose_.position.y = (-1) * translation.at<float>(0);
        pose_.position.z = (-1) * translation.at<float>(1);
        pose_.orientation.x = q_[0];
		pose_.orientation.y = q_[1];
		pose_.orientation.z = q_[2];
		pose_.orientation.w = q_[3];

        return pose_;
    }
}
