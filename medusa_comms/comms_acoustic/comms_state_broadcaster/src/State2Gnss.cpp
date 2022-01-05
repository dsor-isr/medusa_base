/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
*/
// this header incorporates all the necessary #include files and defines the class "State2Gnss"
#include "State2Gnss.h"

/*
#######################################################################################################################
@.@ CONSTRUCTOR: put all dirty work of initializations here
Note the odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
#######################################################################################################################
*/
State2Gnss::State2Gnss(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private) : nh_(*nodehandle), nh_private_(*nodehandle_private)
{
	ROS_INFO("in class constructor of State2Gnss");
	loadParams();
	initializeSubscribers();
	initializePublishers();
}

/*
#######################################################################################################################
@.@ Destructor
#######################################################################################################################
*/
State2Gnss::~State2Gnss()
{

	// +.+ shutdown publishers
	pub_gnss.shutdown();

	// +.+ shutdown subscribers
    sub_state.shutdown();

	// +.+ shutdown node
	nh_.shutdown();
	nh_private_.shutdown();
}

/*
#######################################################################################################################
@.@ Member Helper function to set up subscribers;
note odd syntax: &State2Gnss::subscriberCallback is a pointer to a member function of State2Gnss
"this" keyword is required, to refer to the current instance of State2Gnss
#######################################################################################################################
*/
void State2Gnss::initializeSubscribers()
{
	ROS_INFO("Initializing Subscribers for State2Gnss");
	std::vector<std::string> ros_subs = MedusaGimmicks::getParameters<std::vector<std::string>>(nh_private_, "topics/subscribers");
    sub_state = nh_.subscribe(ros_subs[0], 10, &State2Gnss::stateBroadcasterCallback, this);
}

/*
#######################################################################################################################
@.@ Member helper function to set up publishers;
#######################################################################################################################
*/
void State2Gnss::initializePublishers()
{
	ROS_INFO("Initializing Publishers for State2Gnss"); // ---> add publishers here
	std::vector<std::string> ros_pubs = MedusaGimmicks::getParameters<std::vector<std::string>>(nh_private_, "topics/publishers");
    pub_gnss = nh_.advertise<sensor_msgs::NavSatFix>(ros_pubs[0], 1, true);
}

/*
#######################################################################################################################
@.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate
#######################################################################################################################
*/

/*
#######################################################################################################################
@.@ Load the parameters
#######################################################################################################################
*/
void State2Gnss::loadParams()
{
	ROS_INFO("Load the State2Gnss parameters");
    p_utm_zone = MedusaGimmicks::getParameters<std::string>(nh_private_, "utm_zone","29S");
    try{
        GeographicLib::UTMUPS::DecodeZone(p_utm_zone, zone, northp);
    }
    catch (GeographicLib::GeographicErr ex){
        ROS_ERROR_DELAYED_THROTTLE(10.0, "%s", ex.what());
        ROS_INFO("Shutting down Node State2Gnss");
        nh_.shutdown();
    }
}

/*
#######################################################################################################################
 @.@ Callbacks Section / Methods
#######################################################################################################################
*/

/*
#######################################################################################################################
 @.@ Callback gnss -> convert lat lon to utm and publish in a state message
#######################################################################################################################
*/

void State2Gnss::stateBroadcasterCallback(const auv_msgs::NavigationStatus &msg)
{
    sensor_msgs::NavSatFix gnss_out;
 
    gnss_out.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    gnss_out.header = msg.header;

    // reverse calculation from UTM to gnss
    std::string zonestr = p_utm_zone;
	// TODO: utm hack is temp, remove this
	try{
	    GeographicLib::UTMUPS::Reverse(zone, northp, msg.position.east, msg.position.north, gnss_out.latitude, gnss_out.longitude);
	}
	catch(...){
		ROS_ERROR_DELAYED_THROTTLE(5.0, "Bad UTM Position Passed as state, frame_id: %s", msg.header.frame_id.c_str());
	}

    gnss_out.position_covariance[0] = msg.position_variance.north;
    gnss_out.position_covariance[4] = msg.position_variance.east;

	if(gnss_out.latitude < 90 && gnss_out.latitude > -90)
		if(gnss_out.longitude < 180 && gnss_out.longitude > -180)
		    pub_gnss.publish(gnss_out);
}

/*
#######################################################################################################################
 @.@ Main
 #######################################################################################################################
 */
int main(int argc, char **argv)
{
	// +.+ ROS set-ups:
	ros::init(argc, argv, "acoustic_converters_node"); //node name
	// +.+ create a node handle; need to pass this to the class constructor
	ros::NodeHandle nh, nh_p("~");

	ROS_INFO("main: instantiating an object of type State2Gnss");

	// +.+ instantiate an State2Gnss class object and pass in pointer to nodehandle for constructor to use
	State2Gnss GPSTOState(&nh, &nh_p);

	// +.+ Added to work with timer -> going into spin; let the callbacks do all the work
	ros::spin();

	return 0;
}
