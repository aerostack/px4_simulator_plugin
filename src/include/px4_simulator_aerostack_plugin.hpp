#pragma once
#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <string>
#include <memory>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include "ros_utils_lib/ros_utils.hpp"
#include "ros_utils_lib/control_utils.hpp"
#include "robot_process.h"

#include "tf/transform_datatypes.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#define CAMERA_OFFSET_X 0.01f 
#define CAMERA_OFFSET_Y 0.0f
#define CAMERA_OFFSET_Z 0.0f
// #define CAMERA_OFFSET_R 0.0f
// #define CAMERA_OFFSET_P (-45.0f/180.0f *M_PI)
// #define CAMERA_OFFSET_P (-47.0f/180.0f *M_PI)
// #define CAMERA_OFFSET_YAW 0.0f

#define DEBUG 1

class Px4AerostackPlugin: public RobotProcess{
public:
    Px4AerostackPlugin(){};
private:

	std::string n_space_;
	std::string estimated_pose_topic_;
	std::string estimated_speed_topic_;
	std::string mavros_pose_topic_;
    std::string mavros_speed_topic_;
	
    tf2_ros::TransformBroadcaster tf2_broadcaster_;
    tf2_ros::Buffer tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_ptr_;


    // tf2_ros::StaticTransformBroadcaster tf2_static_broadcaster_;
    std::vector<geometry_msgs::TransformStamped> tf2_fix_transforms_;
    geometry_msgs::TransformStamped odom_to_base_link_transform_;

    ros::NodeHandle nh_;

    ros::Publisher pose_pub_;
    ros::Publisher twist_pub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber speed_sub_;

    void poseCallback(const geometry_msgs::PoseStamped & );
    void speedCallback(const geometry_msgs::TwistStamped & );
    geometry_msgs::PoseStamped estimated_pose_; 
    geometry_msgs::TwistStamped estimated_twist_; 

    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun(){
        publishTFs();
        publishEstimatedPose();
        
    }
        
    void publishTFs(){
        auto timestamp = ros::Time::now();

        for (geometry_msgs::TransformStamped transform:tf2_fix_transforms_){
            transform.header.stamp = timestamp;
            tf2_broadcaster_.sendTransform(transform);
        }
        odom_to_base_link_transform_.header.stamp = timestamp;
        tf2_broadcaster_.sendTransform(odom_to_base_link_transform_);

    }
    void publishEstimatedPose();
};

