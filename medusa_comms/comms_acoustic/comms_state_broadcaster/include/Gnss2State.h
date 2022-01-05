/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico 
*/
#ifndef CATKIN_WS_GNSS2STATEFIX_H
#define CATKIN_WS_GNSS2STATEFIX_H

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <exception>

#include <ros/ros.h>

// ROS messages and stuff
#include <sensor_msgs/NavSatFix.h>
#include <auv_msgs/NavigationStatus.h>
#include <std_msgs/String.h>

// Medusa Libraries and msgs
#include <medusa_gimmicks_library/MedusaGimmicks.h>

// Third Party Libraries
#include <GeographicLib/UTMUPS.hpp>

class Gnss2State
{
public:
	// #############################
	// @.@ Constructor
	// #############################
	// "main" will need to instantiate a ROS nodehandle, then pass it to the constructor
	Gnss2State(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);

	// #############################
	// @.@ Destructor
	// #############################
	~Gnss2State();

private:
	// put private member data here; "private" data will only be available to member functions of this class;
	ros::NodeHandle nh_, nh_private_; // we will need this, to pass between "main" and constructor

	// #####################
	// @.@ Subsctibers
	// #####################
    ros::Subscriber sub_gnss;

	// #####################
	// @.@ Publishers
	// #####################
	// Example: ros::Publisher uref_pub;
	ros::Publisher pub_state, pub_zone;

	// ####################################################################################################################
	// member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
	// member variables will retain their values even as callbacks come and go
	// ####################################################################################################################

	// +.+ package.xml Parameters from Yaml
	bool p_broadcast_tf;
    double p_default_depth;

	// +.+ Problem variables
    // #######################################################################################
	// @.@ Encapsulation the gory details of initializing subscribers, publishers and services
	// #######################################################################################
	void initializeSubscribers();
	void initializePublishers();
	void loadParams();

	// #######################################################################################
	// @.@ Callbacks declaration
	// #######################################################################################
	void gnssBroadcasterCallback(const auv_msgs::NavigationStatus &msg);
};
#endif //CATKIN_WS_Gnss2State_H
