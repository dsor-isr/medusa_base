/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico 
*/
#ifndef CATKIN_WS_SMOOTHSTATE_H
#define CATKIN_WS_SMOOTHSTATE_H

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <exception>
#include <algorithm>
#include <functional>

#include <ros/ros.h>

// ROS messages and stuff
#include <auv_msgs/NavigationStatus.h>
#include <geometry_msgs/PoseStamped.h>

// Medusa Libraries and msgs
#include <medusa_gimmicks_library/MedusaGimmicks.h>

// Third Party Libraries
#include <GeographicLib/UTMUPS.hpp>

// TFs
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#define DEG2RAD(x) ((x) * ((MedusaGimmicks::PI) / (180.0)))
#define BUFFER_LEN 10

class SmoothState
{
public:
	// #############################
	// @.@ Constructor
	// #############################
	SmoothState(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);

	// #############################
	// @.@ Destructor
	// #############################
	~SmoothState();

private:
	ros::NodeHandle nh_, nh_private_;

	// #####################
	// @.@ Subsctibers
	// #####################
    ros::Subscriber state_sub_;

	// #####################
	// @.@ Publishers
	// #####################
	ros::Publisher pub_pose_;

	// #####################
	// @.@ Timers
	// #####################
	ros::Timer timer_lerp_;

	// ####################################################################################################################
	// member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
	// member variables will retain their values even as callbacks come and go
	// ####################################################################################################################

	// +.+ package.xml Parameters from Yaml

	// +.+ Problem variables
	bool initialized;
	double node_frequency, input_frequency;
	std::vector<double> state, state_averaged, start_state, end_state;
	std::vector<std::vector<double>> state_history;

    // #######################################################################################
	// @.@ Encapsulation the gory details of initializing subscribers, publishers and services
	// #######################################################################################
	void nodeFrequency();
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
	void stateCallback(const auv_msgs::NavigationStatus &msg);
	std::vector<double> movingAverage(std::vector<std::vector<double>> buffer);
	void lerpState(std::vector<double> &start, const std::vector<double> &end, const double &t);
	void lerpTimerCallback(const ros::TimerEvent &event);
	void publishPose(std::vector<double> &state);

};
#endif //CATKIN_WS_Gnss2State_H
