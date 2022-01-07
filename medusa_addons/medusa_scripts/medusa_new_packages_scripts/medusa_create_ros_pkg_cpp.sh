#!/bin/bash

#
# Developers: malaclemys          > jquintas@gmail.com
#
# Description: bash script for creating a ros cpp package according with the medusa stack best practices
#
# Note: This was greatly based on Oscar script. Thank you mexican god.
#
#@~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~@
#@         ?                                      @
#@                  ___                           @
#@             ,,  // \\                          @
#@            (_,\/ \_/ \                         @
#@              \ \_/_\_/>                        @
#@              /_/  /_/              _______     @
#@                                   |START |     @
#@                                   |______|     @
#@                                   |            @
#@___________________________________|____________@
#

################################################################
# @.@ assign first received argument to variable PKG_NAME      # 
################################################################
if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters"
    echo "Usage : bash medusa_create_ros_pkg_cpp.sh fiic_awesome_cpp"
    exit
fi

################################################################
# @.@ assign first received argument to variable PKG_NAME
################################################################
PKG_NAME_IN=${1,,}
PKG_NAME=$(echo "$PKG_NAME_IN" | sed -r 's/_([a-z])/\U\1/g')

################################################################
# @.@ Give some author love                                    # 
################################################################
AUTHOR_NAME="DSOR GROUP"
AUTHOR_EMAIL="dsor@gmail.com"
MAINTAINER_NAME="genX"
MAINTAINER_EMAIL="dsor@gmail.com"

################################################################
# @.@ setup file content                                       # 
################################################################
CMAKELISTS_CONTENT="cmake_minimum_required(VERSION 2.8.3)\nproject(${PKG_NAME_IN})\n\nfind_package(catkin REQUIRED\n  COMPONENTS\n std_msgs \n medusa_msgs\n roscpp\n)\n\ncatkin_package(\n CATKIN_DEPENDS\n)\n\nadd_compile_options(-std=c++11) \n\ninclude_directories(\n include/${PKG_NAME_IN}_ros\n include/${PKG_NAME_IN}_algorithms\n \${catkin_INCLUDE_DIRS}\n)\n\nadd_executable(\${PROJECT_NAME}_node src/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.cpp src/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.cpp)\nadd_dependencies(\${PROJECT_NAME}_node \${catkin_EXPORTED_TARGETS})\ntarget_link_libraries(\${PROJECT_NAME}_node \${catkin_LIBRARIES})"
PACKAGE_XML_CONTENT="<?xml version=\"1.0\"?>\n<package format=\"2\">\n  <name>${PKG_NAME_IN}</name>\n  <version>1.0.0</version>\n  <description>\n    TODO!\n  </description>\n\n  <license>GPLv3</license>\n\n  <author email=\"${AUTHOR_EMAIL}\">${AUTHOR_NAME}</author>\n  <maintainer email=\"${MAINTAINER_EMAIL}\">${MAINTAINER_NAME}</maintainer>\n\n  <buildtool_depend>catkin</buildtool_depend>\n\n  <depend>std_msgs</depend>\n\n <depend>roscpp</depend>\n\n <depend>medusa_msgs</depend>\n\n</package>"
CONFIG_YAML_CONTENT="node_frequency: 2"
README_CONTENT="${PKG_NAME_IN} documentation"
LAUNCH_FILE_CONTENT="<?xml version=\"1.0\"?>\n<launch>\n\n    <!-- small description about your node -->\n    \n    <!--<node   pkg=\"my_package_name\" type=\"my_node_name\" name=\"my_node_name\"\n            respawn=\"false\" output=\"screen\" args=\"\$(find ${PKG_NAME_IN})/config/my_arg_file.yaml\"/>-->\n    \n    <node   pkg=\"${PKG_NAME_IN}\" type=\"${PKG_NAME_IN}_node\" name=\"${PKG_NAME^}Node\" respawn=\"false\" output=\"screen\">\n\t<rosparam command=\"load\" file=\"\$(find ${PKG_NAME_IN})/config/config_${PKG_NAME_IN}.yaml\"/>\n</node>\n\n</launch>"
SCRIPTS_CONTENT="//No need in cpp"
MY_TEST_CONTENT=""

ROS_NODE_CONTENT="/*\nDevelopers: DSOR Team  -> @isr.ist.pt Instituto Superior Tecnico\nDescription: Please check the documentation of this package for more info.\n*/\n
// this header incorporates all the necessary #include files and defines the class \"${PKG_NAME^}Node\"\n
#include \"${PKG_NAME^}Node.h\"\n
#include \"${PKG_NAME^}Algorithm.h\"\n\n
/*\n#######################################################################################################################\n
@.@ CONSTRUCTOR: put all dirty work of initializations here\n
    Note the odd syntax: have to pass nodehandle pointer into constructor for constructor to build subscribers, etc\n
#######################################################################################################################\n
*/\n
${PKG_NAME^}Node::${PKG_NAME^}Node(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_p_(*nodehandle_private) {\n
\tROS_INFO(\"in class constructor of ${PKG_NAME^}Node\");\n
\tloadParams();\n
\tinitializeSubscribers();\n
\tinitializePublishers();\n
\tinitializeTimer();\n\n
}\n\n
/*\n#######################################################################################################################\n
 @.@ Destructor\n
#######################################################################################################################\n
*/\n
${PKG_NAME^}Node::~${PKG_NAME^}Node() {\n\n
\t// +.+ shutdown publishers\n
\t// ---> add publishers here\n
\t// Example: uref_pub.shutdown();\n
\t wp_status_timer_pub.shutdown();\n\n
\t// +.+  shutdown subscribers\n
\t// ---> add subscribers here\n
\t// Example: state_sub.shutdown();\n
\tflag_sub.shutdown();\n\n
\t// +.+  stop timer\n
\ttimer_.stop();\n\n
\t// +.+  shutdown node\n
\tnh_.shutdown();\n
}\n\n
/*\n#######################################################################################################################\n
@.@ Member Helper function to set up subscribers;\n
note odd syntax: &${PKG_NAME^}Node::subscriberCallback is a pointer to a member function of ${PKG_NAME^}Node\n
\"this\" keyword is required, to refer to the current instance of ${PKG_NAME^}Node\n
#######################################################################################################################\n
*/\n
void ${PKG_NAME^}Node::initializeSubscribers() {\n
\tROS_INFO(\"Initializing Subscribers for ${PKG_NAME^}Node\");\n
\t// ---> add subscribers here\n
\t// Example: state_sub = nh_.subscribe(\"State\", 10, &${PKG_NAME^}Node::updateCallback,this);\n\n
\tflag_sub = nh_.subscribe(\"Flag\",10, &${PKG_NAME^}Node::wpStatusCallback,this);\n\n
}\n\n
/*\n#######################################################################################################################\n
@.@ Member helper function to set up publishers;\n
#######################################################################################################################\n
*/\n
void ${PKG_NAME^}Node::initializePublishers() {\n
\tROS_INFO(\"Initializing Publishers for ${PKG_NAME^}Node\");
\t// ---> add publishers here\n
\t//Example: uref_pub = nh_.advertise<std_msgs::Float64>(\"URef\", 10); //Surge Reference\n\n
\twp_status_timer_pub = nh_.advertise<std_msgs::Bool>(\"wp_status\",10);\n
}\n\n
/*\n#######################################################################################################################\n
@.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate\n
#######################################################################################################################\n
*/\n
void ${PKG_NAME^}Node::initializeTimer() {\n
\ttimer_ =nh_.createTimer(ros::Duration(1.0/${PKG_NAME^}Node::nodeFrequency()), &${PKG_NAME^}Node::timerIterCallback, this);\n
\ttimer_.stop();\n
}\n\n
/*\n#######################################################################################################################\n
@.@ Set frequency of the node default is 2\n
#######################################################################################################################\n
*/\n
double ${PKG_NAME^}Node::nodeFrequency()\n
{\n
\tdouble node_frequency;\n
\tnh_.param(\"node_frequency\", node_frequency, 2.0);\n
\tROS_INFO(\"Node will run at : %lf [hz]\", node_frequency);\n
\treturn node_frequency;\n
}\n\n
/*\n#######################################################################################################################\n
@.@ Load the parameters\n
#######################################################################################################################\n
*/\n
void ${PKG_NAME^}Node::loadParams() {\n
\tROS_INFO(\"Load the ${PKG_NAME^}Node parameters\");\n
\t//---> params here, always p_paramName\n
\t//Example: nh_.param(\"/${PKG_NAME^}Node/Ku\", p_ku, 0.5);\n
}\n\n
/*\n#######################################################################################################################\n
@.@ Callbacks Section / Methods\n
#######################################################################################################################\n
*/\n\n
/*\n#######################################################################################################################\n
@.@ Iteration via timer callback\n
#######################################################################################################################\n
*/\n
void ${PKG_NAME^}Node::timerIterCallback(const ros::TimerEvent &event) {\n\n
\t// ####################################################################################################################\n
\t// @.@ Do your algorithm part and publish things here, like while loop with ros::Rate rate and rate.sleep(duration)\n
\t\t// in classical ros\n
\t// ###################################################################################################################\n
\taux_bool.data = true;    
\twp_status_timer_pub.publish(std_msgs::Bool(aux_bool));\n
}\n

/*\n#######################################################################################################################\n
@.@ Callback Flag\n
#######################################################################################################################\n
*/\n
void ${PKG_NAME^}Node::wpStatusCallback(const std_msgs::Int8& msg) {\n\n
\taux_int.data = 4;\n    
\tif (msg.data != aux_int.data){\n
\t\t ROS_INFO(\"Timer will Stop\");\n
\t\t timer_.stop();\n
\t}
\telse{\n
\t\t ROS_INFO(\"Timer will Start\");\n
\t\t timer_.start();\n
\t}\n
\n
}\n

/*\n#######################################################################################################################\n
@.@ Main\n
#######################################################################################################################\n
*/\n
int main(int argc, char** argv)\n
{\n
\t// +.+ ROS set-ups:\n
\tros::init(argc, argv, \"${PKG_NAME_IN}_node\"); //node name\n
\t// +.+ create a node handle; need to pass this to the class constructor\n
\tros::NodeHandle nh;\n\n
\tros::NodeHandle nh_p(\"~\");\n\n
\tROS_INFO(\"main: instantiating an object of type ${PKG_NAME^}Node\");\n\n
\t// +.+ instantiate an ${PKG_NAME^}Node class object and pass in pointer to nodehandle for constructor to use\n
\t${PKG_NAME^}Node ${PKG_NAME}(&nh,&nh_p);\n\n
\t// +.+ Added to work with timer -> going into spin; let the callbacks do all the work\n
\tros::spin();\n\n
\treturn 0;\n
}\n
"
ROS_NODE_H="/*\nDevelopers:  DSOR Team  -> @irt.ist.pt Instituto Superior Tecnico */\n
#ifndef CATKIN_WS_${PKG_NAME^^}NODE_H\n
#define CATKIN_WS_${PKG_NAME^^}NODE_H\n\n
//some generically useful stuff to include...\n
#include <math.h>\n
#include <stdlib.h>\n
#include <std_msgs/String.h>\n
#include <vector>\n\n
#include <ros/ros.h> //ALWAYS need to include this \n\n
// ROS messages and stuff\n
// Examples\n
//#include <medusa_msgs/mState.h>\n
//#include <geometry_msgs/PointStamped.h>\n
//#include <geometry_msgs/Vector3Stamped.h>\n
//#include <std_msgs/Float64.h>\n
#include <std_msgs/Int8.h>\n\n
#include <std_msgs/Bool.h>\n\n
class ${PKG_NAME^}Node {\n
public:\n
\t// #############################\n
\t// @.@ Constructor\n
\t// #############################\n
\t// \"main\" will need to instantiate two ROS nodehandles, then pass it to the constructor\n
\t${PKG_NAME^}Node(ros::NodeHandle* nodehandle, ros::NodeHandle *nodehandle_private);\n\n
\t// #############################\n
\t// @.@ Destructor\n
\t// #############################\n
\t~${PKG_NAME^}Node();\n\n
\t// #############################\n
\t// @.@ Public methods\n
\t// #############################\n
\tdouble nodeFrequency();\n\n
private:\n
\t// put private member data here;  \"private\" data will only be available to member functions of this class;\n
\tros::NodeHandle nh_; // we will need this, to pass between \"main\" and constructor\n\n
\tros::NodeHandle nh_p_; // we will need this, to pass between \"main\" and constructor\n\n
\t// some objects to support subscriber and publishers, these will be set up within the class constructor, hiding the ugly details\n\n
\t// #####################\n
\t// @.@ Subsctibers\n
\t// #####################\n
\t// Example: ros::Subscriber state_sub;\n
\tros::Subscriber flag_sub;\n\n
\t// #####################\n
\t// @.@ Publishers\n
\t// #####################\n
\t// Example: ros::Publisher uref_pub;\n
\tros::Publisher wp_status_timer_pub;\n\n
\t// #######################\n
\t// @.@ Timer\n
\t// #######################\n
\tros::Timer timer_;\n\n
\t// ####################################################################################################################\n
\t// member variable: better than using globals; convenient way to pass data from a subscriber to other member functions\n
\t// member variables will retain their values even as callbacks come and go\n
\t// ####################################################################################################################\n\n
\t// +.+ Parameters from Yaml\n
\t// always p_paraName -> Example: double p_ku;\n\n
\t// +.+ Handy variables\n
\t// Example double var_temp_x, var_temp_y;\n\n
\t// +.+ Problem variables\n
\t// Example: double x_state, y_state;\n
\tstd_msgs::Int8 aux_int;\n  
\tstd_msgs::Bool aux_bool;\n\n
\t// #######################################################################################\n
\t// @.@ Encapsulation the gory details of initializing subscribers, publishers and services\n
\t// #######################################################################################\n
\tvoid initializeSubscribers();\n
\tvoid initializePublishers();\n
\tvoid initializeTimer();\n
\tvoid loadParams();\n\n
\t// #######################################################################################\n
\t// @.@ Callbacks declaration\n
\t// #######################################################################################\n
\tvoid timerIterCallback(const ros::TimerEvent& event);\n\tvoid wpStatusCallback(const std_msgs::Int8& msg);\n};\n#endif //CATKIN_WS_${PKG_NAME^^}NODE_H"

ROS_INDEPENDENT_CLASS_CONTENT="//TODO: Algorithm Code Here\n\n"
ROS_INDEPENDENT_CLASS_H="//TODO: Algorithm declarations Here\n\n"

###############################################################
# @.@ create folder structure                                 #  
###############################################################
mkdir ${PKG_NAME_IN} 
cd ${PKG_NAME_IN}
mkdir -p include/${PKG_NAME_IN}_ros
mkdir -p include/${PKG_NAME_IN}_algorithms
mkdir -p config
mkdir -p doc
mkdir -p launch
mkdir -p scripts
mkdir -p src/${PKG_NAME_IN}_ros
mkdir -p src/${PKG_NAME_IN}_algorithms
mkdir -p test

################################################################
# @.@ create file structure                                    # 
################################################################
touch CMakeLists.txt
touch include/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.h
touch include/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.h
touch package.xml
touch config/config_${PKG_NAME_IN}.yaml
touch doc/README.md
touch launch/${PKG_NAME_IN}.launch
touch scripts/${PKG_NAME}Script
touch src/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.cpp
touch src/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.cpp
touch test/${PKG_NAME_IN}_test.cpp

################################################################
# @.@ fill files with information from parameter section       #
################################################################
echo -e $CMAKELISTS_CONTENT  > CMakeLists.txt
echo -e $PACKAGE_XML_CONTENT > package.xml
echo -e $CONFIG_YAML_CONTENT > config/config_${PKG_NAME_IN}.yaml
echo -e $README_CONTENT > doc/README.md
echo -e $LAUNCH_FILE_CONTENT > launch/${PKG_NAME_IN}.launch 
echo -e $MY_TEST_CONTENT > test/${PKG_NAME_IN}_test.cpp
echo -e $ROS_NODE_CONTENT > src/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.cpp
echo -e $ROS_NODE_H > include/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.h
echo -e $ROS_INDEPENDENT_CLASS_CONTENT > src/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.cpp
echo -e $ROS_INDEPENDENT_CLASS_H > include/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.h

################################################################
# @.@ Make or build the workspace, you decide                  #
################################################################
medusa_cbt

################################################################
# @.@ Unset the created local variables of this file           #
################################################################
unset PKG_NAME
unset AUTHOR_NAME
unset AUTHOR_EMAIL
unset MAINTAINER_NAME
unset MAINTAINER_EMAIL
unset CMAKELISTS_CONTENT
unset PACKAGE_XML_CONTENT
unset CONFIG_YAML_CONTENT
unset README_CONTENT
unset LAUNCH_FILE_CONTENT
unset SCRIPTS_CONTENT
unset MY_TEST_CONTENT
unset ROS_NODE_CONTENT
unset ROS_NODE_H
unset ROS_INDEPENDENT_CLASS_CONTENT

################################################################
# @.@ Source bashrc and echo an ecouriging goodbye message     #
################################################################
source ~/.bashrc
echo "${PKG_NAME_IN} ros cpp package created, you are now inside it. Let the fun begin..." 
unset PKG_NAME_IN