#include "px4_simulator_aerostack_plugin.hpp"

geometry_msgs::TransformStamped getTransformation(	const std::string& _frame_id,
												  	const std::string& _child_frame_id,
													double _translation_x,
													double _translation_y,
													double _translation_z,
													double _roll,
													double _pitch,
													double _yaw  ){

geometry_msgs::TransformStamped transformation;

	transformation.header.frame_id = _frame_id;
	transformation.child_frame_id = _child_frame_id;
	transformation.transform.translation.x = _translation_x;
	transformation.transform.translation.y = _translation_y;
	transformation.transform.translation.z = _translation_z;
	tf2::Quaternion q;
	q.setRPY(_roll, _pitch, _yaw);
	transformation.transform.rotation.x = q.x();
	transformation.transform.rotation.y = q.y();
	transformation.transform.rotation.z = q.z();
	transformation.transform.rotation.w = q.w();
	
	return transformation;

}

void Px4AerostackPlugin::ownSetUp()
{
 	
	ros_utils_lib::getPrivateParam<std::string>("~namespace"					, n_space_						    ,"drone1");
	ros_utils_lib::getPrivateParam<std::string>("~estimated_pose_topic" 	    , estimated_pose_topic_ 			,"self_localization/pose");
	ros_utils_lib::getPrivateParam<std::string>("~estimated_speed_topic" 	    , estimated_speed_topic_ 			,"self_localization/speed");
	ros_utils_lib::getPrivateParam<std::string>("~mavros_pose_topic" 	        , mavros_pose_topic_ 			    ,"mavros/local_position/pose");
	ros_utils_lib::getPrivateParam<std::string>("~mavros_speed_topic" 	        , mavros_speed_topic_ 		 	    ,"mavros/local_position/velocity_local");
	 
	tf2_fix_transforms_.clear();
	tf2_fix_transforms_.emplace_back(getTransformation("map","odom",0,0,0,0,0,0));
	// tf2_fix_transforms_.emplace_back(getTransformation("base_link","camera_frame",-CAMERA_OFFSET_X,-CAMERA_OFFSET_Y,-CAMERA_OFFSET_Z,0,0,0));
    tf2_fix_transforms_.emplace_back(getTransformation("base_link","camera_link",CAMERA_OFFSET_X,CAMERA_OFFSET_Y,CAMERA_OFFSET_Z,M_PI/2,M_PI,M_PI/2));


	odom_to_base_link_transform_.header.frame_id = "odom";
	odom_to_base_link_transform_.child_frame_id  = "base_link" ;
	odom_to_base_link_transform_.transform.rotation.w = 1.0f;
	tf2_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf2_buffer_);

}


void Px4AerostackPlugin::ownStart(){

  	pose_sub_  = nh_.subscribe("/" + n_space_ + "/" + mavros_pose_topic_	,1, &Px4AerostackPlugin::poseCallback,this);	
  	speed_sub_  = nh_.subscribe("/" + n_space_ + "/" + mavros_speed_topic_	,1, &Px4AerostackPlugin::speedCallback,this);	
  	pose_pub_  = nh_.advertise<geometry_msgs::PoseStamped>  ("/" + n_space_ + "/" + estimated_pose_topic_ ,1);
  	twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped> ("/" + n_space_ + "/" + estimated_speed_topic_,1);
}

void Px4AerostackPlugin::ownStop(){
	pose_sub_.shutdown();
	speed_sub_.shutdown();
	pose_pub_.shutdown();
	twist_pub_.shutdown();
}


void Px4AerostackPlugin::publishEstimatedPose(){
	
	auto timestamp = ros::Time::now();

	estimated_pose_.header.stamp  = timestamp;
	estimated_twist_.header.stamp = timestamp;


	estimated_pose_.header.frame_id  = "odom";
	estimated_twist_.header.frame_id = "odom";

	pose_pub_.publish(estimated_pose_);
	twist_pub_.publish(estimated_twist_);
	
}

void Px4AerostackPlugin::poseCallback(const geometry_msgs::PoseStamped& _msg){
	
	estimated_pose_   = _msg;
	

	odom_to_base_link_transform_.transform.translation.x = _msg.pose.position.x;
	odom_to_base_link_transform_.transform.translation.y = _msg.pose.position.y;
	odom_to_base_link_transform_.transform.translation.z = _msg.pose.position.z;
	odom_to_base_link_transform_.transform.rotation = _msg.pose.orientation;

}

void Px4AerostackPlugin::speedCallback(const geometry_msgs::TwistStamped& _msg){
	estimated_twist_ = _msg;
}
