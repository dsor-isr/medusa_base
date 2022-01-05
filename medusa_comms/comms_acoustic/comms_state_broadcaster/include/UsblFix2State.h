/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico */
#ifndef CATKIN_WS_USBLFIX2STATE_H
#define CATKIN_WS_USBLFIX2STATE_H

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <exception>

#include <ros/ros.h> //ALWAYS need to include this

// ROS messages and stuff
#include <nav_msgs/Odometry.h>
#include <medusa_msgs/mUSBLFix.h>

// Medusa Libraries and msgs
#include <medusa_gimmicks_library/MedusaGimmicks.h>

// TFs
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Third Party Libraries
#include <GeographicLib/UTMUPS.hpp>

class UsblFix2State
{
public:
	// #############################
	// @.@ Constructor
	// #############################
	UsblFix2State(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);

	// #############################
	// @.@ Destructor
	// #############################
	~UsblFix2State();

private:
	// put private member data here; "private" data will only be available to member functions of this class;
	ros::NodeHandle nh_, nh_private_; // we will need this, to pass between "main" and constructor

	// #####################
	// @.@ Subsctibers
	// #####################
    ros::Subscriber usbl_sub_, state_sub_;

	// #####################
	// @.@ Publishers
	// #####################
	ros::Publisher state_pub_;

	// ####################################################################################################################
	// member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
	// member variables will retain their values even as callbacks come and go
	// ####################################################################################################################

	// +.+ package.xml Parameters from Yaml
	bool p_fix_type;
	int p_t_sync_;
	std::string p_state_in_topic_,  p_state_out_topic_, p_usbl_topic_;
	std::string p_world_frame_id_;
	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;

	// +.+ Problem variables
	nav_msgs::Odometry state;
	std::list<medusa_msgs::mUSBLFix> usblfix_list;

    // #######################################################################################
	// @.@ Encapsulation the gory details of initializing subscribers, publishers and services
	// #######################################################################################
	void initializeSubscribers();
	void initializePublishers();
	void loadParams();

	// #######################################################################################
	// @.@ Callbacks declaration
	// #######################################################################################
	void usblFixBroadcasterCallback(const medusa_msgs::mUSBLFix &msg);
	void stateCallback(const nav_msgs::Odometry &msg);

	// #######################################################################################
	// @.@ Supplimentary declaration
	// #######################################################################################
	geometry_msgs::Pose computePosition(double bearing, double elevation, double range);

};
#endif //CATKIN_WS_UsblFix2State_H
