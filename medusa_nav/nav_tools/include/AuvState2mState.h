/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico 
*/
#ifndef CATKIN_WS_AUVSTATE2MSTATE_H
#define CATKIN_WS_AUVSTATE2MSTATE_H

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <exception>
#include "LowPassFilter.h"

#include <ros/ros.h>

// ROS messages and stuff
#include <auv_msgs/NavigationStatus.h>
#include <medusa_msgs/mState.h>
#include <medusa_msgs/Pressure.h>
#include <sensor_msgs/NavSatFix.h>

// Medusa Libraries and msgs
#include <medusa_gimmicks_library/MedusaGimmicks.h>

// TFs
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class AuvState2mState
{
public:
	// #############################
	// @.@ Constructor
	// #############################
	AuvState2mState(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);

	// #############################
	// @.@ Destructor
	// #############################
	~AuvState2mState();

private:
	ros::NodeHandle nh_, nh_private_;

	// #####################
	// @.@ Subsctibers
	// #####################
    ros::Subscriber sub_inside_pressure_, sub_auv_state_, sub_gps_status_;

	// #####################
	// @.@ Publishers
	// #####################
	ros::Publisher mstate_pub_;

	// #####################
	// @.@ Timers
	// #####################
	ros::Timer timer_in_pressure_;

	// ####################################################################################################################
	// member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
	// member variables will retain their values even as callbacks come and go
	// ####################################################################################################################

	// +.+ Problem variables
	medusa_msgs::Pressure in_pressure;
	LowPassFilter* insidePressure;
	double in_press, in_press_dot;
	unsigned int gps_status_{1};

    // #######################################################################################
	// @.@ Encapsulation the gory details of initializing subscribers, publishers and services
	// #######################################################################################
	void initializeSubscribers();
	void initializePublishers();
	void initializeTimers();
	void loadParams();

	// #######################################################################################
	// @.@ Callbacks declaration
	// #######################################################################################
	/* 
	* Function Name: gnssBroadcasterCallback
	* Inputs : GNSS/GPS Fix of the vehicle
	* Effects: Converts Lat/Lon fix to UTM and publishes the fix
	* Outputs: ...
	*/	
	void mStateBroadcasterCallback(const auv_msgs::NavigationStatus &msg);
	void insidePressureCallback(const medusa_msgs::Pressure &msg);
	void mGPSStatusCallback(const sensor_msgs::NavSatFix &msg);
	void insidePressureTimer(const ros::TimerEvent& event);

};
#endif //AuvState2mState_H
