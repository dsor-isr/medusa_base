/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico */
#ifndef CATKIN_WS_STATE2GNSS_H
#define CATKIN_WS_STATE2GNSS_H

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <exception>

#include <ros/ros.h> //ALWAYS need to include this

// ROS messages and stuff
#include <sensor_msgs/NavSatFix.h>
#include <auv_msgs/NavigationStatus.h>

// Medusa Libraries and msgs
#include <medusa_gimmicks_library/MedusaGimmicks.h>

// Third Party Libraries
#include <GeographicLib/UTMUPS.hpp>

class State2Gnss
{
public:
	// #############################
	// @.@ Constructor
	// #############################
	State2Gnss(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);

	// #############################
	// @.@ Destructor
	// #############################
	~State2Gnss();

private:
	// put private member data here; "private" data will only be available to member functions of this class;
	ros::NodeHandle nh_, nh_private_; // we will need this, to pass between "main" and constructor

	// #####################
	// @.@ Subsctibers
	// #####################
    ros::Subscriber sub_state;

	// #####################
	// @.@ Publishers
	// #####################
	ros::Publisher pub_gnss;

	// ####################################################################################################################
	// member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
	// member variables will retain their values even as callbacks come and go
	// ####################################################################################################################

	// +.+ package.xml Parameters from Yaml
	std::string p_utm_zone;

	// +.+ Problem variables
    bool northp;
    int zone;

    // #######################################################################################
	// @.@ Encapsulation the gory details of initializing subscribers, publishers and services
	// #######################################################################################
	void initializeSubscribers();
	void initializePublishers();
	void loadParams();

	// #######################################################################################
	// @.@ Callbacks declaration
	// #######################################################################################
	void stateBroadcasterCallback(const auv_msgs::NavigationStatus &msg);
};
#endif //CATKIN_WS_State2Gnss_H
