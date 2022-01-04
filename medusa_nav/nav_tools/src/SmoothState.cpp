/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
*/
// this header incorporates all the necessary #include files and defines the class "Gnss2Utm"
#include "SmoothState.h"

/*
#######################################################################################################################
@.@ CONSTRUCTOR: put all dirty work of initializations here
Note the odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc
#######################################################################################################################
*/
SmoothState::SmoothState(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private) : nh_(*nodehandle), nh_private_(*nodehandle_private)
{
	ROS_INFO("in class constructor of SmoothState");

	initialized = false;
	state.reserve(6);
	state_history.reserve(BUFFER_LEN);

	nodeFrequency();
	loadParams();
	initializeSubscribers();
	initializePublishers();
	initializeTimers();
}

/*
#######################################################################################################################
@.@ Destructor
#######################################################################################################################
*/
SmoothState::~SmoothState()
{

	// +.+ shutdown publishers
	state_sub_.shutdown();

	// +.+ shutdown subscribers
	pub_pose_.shutdown();

	// +.+ shutdown node
	nh_.shutdown();
	nh_private_.shutdown();
}

/*
#######################################################################################################################
@.@ Member Helper function to set up subscribers;
note odd syntax: &SmoothState::subscriberCallback is a pointer to a member function of SmoothState
"this" keyword is required, to refer to the current instance of SmoothState
#######################################################################################################################
*/
void SmoothState::initializeSubscribers()
{
	ROS_INFO("Initializing Subscribers for SmoothState");
	std::vector<std::string> ros_subs = MedusaGimmicks::getParameters<std::vector<std::string>>(nh_private_, "topics/subscribers");
    state_sub_ = nh_.subscribe(ros_subs[0], 1, &SmoothState::stateCallback, this);
}

/*
#######################################################################################################################
@.@ Member helper function to set up publishers;
#######################################################################################################################
*/
void SmoothState::initializePublishers()
{
	ROS_INFO("Initializing Publishers for SmoothState"); // ---> add publishers here
	std::vector<std::string> ros_pubs = MedusaGimmicks::getParameters<std::vector<std::string>>(nh_private_, "topics/publishers");
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(ros_pubs[0], 50, true);
}

/*
#######################################################################################################################
@.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate
#######################################################################################################################
*/
void SmoothState::initializeTimers(){
	ROS_INFO("Initializing Timers for SmoothState"); // ---> add publishers here
	timer_lerp_ = nh_.createTimer(ros::Duration(1.0 / node_frequency), &SmoothState::lerpTimerCallback, this);
}

/*
######################################################################################################################
@.@ Set frequency of the node default is 10
#######################################################################################################################
*/
void SmoothState::nodeFrequency()
{
	nh_.param("node_frequency", node_frequency, 30.0);
	nh_.param("input_frequency", input_frequency, 10.0);
	ROS_INFO("Node will run at : %lf [hz]", node_frequency);
}
/*
#######################################################################################################################
@.@ Load the parameters
#######################################################################################################################
*/
void SmoothState::loadParams()
{
	// ROS_INFO("Load the SmoothState parameters");
}

/*
#######################################################################################################################
 @.@ Callbacks Section / Methods
#######################################################################################################################
*/

/*
#######################################################################################################################
 @.@ Callback gps -> convert lat lon to utm and publish in a state message
#######################################################################################################################
*/
void SmoothState::stateCallback(const auv_msgs::NavigationStatus &msg)
{
	// Store current state
	state[0] = msg.position.north;
	state[1] = msg.position.east;
	state[2] = msg.position.depth;

	state[3] = DEG2RAD(msg.orientation.x);
	state[4] = DEG2RAD(msg.orientation.y);
	state[5] = DEG2RAD(msg.orientation.z);

	if(!initialized){
		for (int i = 0; i < 6; i++)
			start_state.push_back(state[i]);
		initialized = true;
	}

	// Add state to state history buffer
	if (state_history.size() >= BUFFER_LEN){
		state_history.erase(state_history.begin());
	}
	state_history.push_back(state);
	for (int i = 0; i < 6; i++)
		state_history[state_history.size() - 1].push_back(state[i]);

	state_averaged = movingAverage(state_history);
	end_state = state_averaged;
	// end_state.assign(state_averaged.begin(), state_averaged.begin()+2);
	// std::cout << " #6 -------------------" << std::endl;
	// for (int i = 0; i < state_averaged.size(); i++)
	// 	std::cout << std::fixed << "Moving Average = " << state_averaged[i] << std::endl;
	// std::cout << " -------------------" << std::endl;
}

std::vector<double> SmoothState::movingAverage(std::vector<std::vector<double>> buffer){
	std::vector<double> state_averaged (6, 0);

	// Add past BUFFER_LEN positions
	for (unsigned int i = 0; i < buffer.size(); i++){
		for (int j = 0; j < 3; j++)
			state_averaged[j] = state_averaged[j] + buffer[i][j];
	}

	// Divide to get average
	for (int i = 0; i < 3; i++){
		state_averaged[i] = state_averaged[i]/buffer.size();
	}

	return state_averaged;
}

void SmoothState::lerpState(std::vector<double> &start, const std::vector<double> &end, const double &t){
	std::vector<double> pos (6, 0);
	for (int i = 0; i < 3; i++){
		pos[i] = start[i] + (end[i] - start[i]) * t;
		start[i] = pos[i];
	}
}

void SmoothState::lerpTimerCallback(const ros::TimerEvent &event){

	// Return if not initialized
	if(!initialized)
		return;
	// Interpolate 33%(input/node) of the way forward
	lerpState(start_state, end_state, input_frequency/node_frequency);
	// Publish new starting position
	publishPose(start_state);
}

void SmoothState::publishPose(std::vector<double> &state){
	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "hrov/auv";

	pose.pose.position.x = state[0];
	pose.pose.position.y = state[1];
	pose.pose.position.z = state[2];

	tf2::Quaternion quat_tf;
    quat_tf.setRPY(state[3], state[4], state[5]);
    pose.pose.orientation = tf2::toMsg(quat_tf);

	pub_pose_.publish(pose);
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

	ROS_INFO("main: instantiating an object of type SmoothState");

	// +.+ instantiate an SmoothState class object and pass in pointer to nodehandle for constructor to use
	SmoothState smoothState(&nh, &nh_p);

	// +.+ Added to work with timer -> going into spin; let the callbacks do all the work
	ros::spin();

	return 0;
}
