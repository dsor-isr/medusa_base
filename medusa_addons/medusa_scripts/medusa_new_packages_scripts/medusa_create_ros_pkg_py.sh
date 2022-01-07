#!/bin/bash
#
# Developers: malaclemys          > jquintas@gmail.com
#
# Description: bash script for creating a ros PY package according with the medusa stack best practices
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
    echo "Usage : bash medusa_create_ros_pkg_py.sh fiic_awesome_py"
    exit
fi

################################################################
# @.@ Give some author love                                    # 
################################################################
PKG_NAME_IN=${1,,}
PKG_NAME=$(echo "$PKG_NAME_IN" | sed -r 's/_([a-z])/\U\1/g')
AUTHOR_NAME="DSOR GROUP"
AUTHOR_EMAIL="dsor@gmail.com"
MAINTAINER_NAME="genX"
MAINTAINER_EMAIL="dsor@gmail.com"

################################################################
# @.@ setup file content                                       # 
################################################################
CMAKELISTS_CONTENT="cmake_minimum_required(VERSION 2.8.3)\nproject(${PKG_NAME_IN})\n\nfind_package(catkin REQUIRED\n  COMPONENTS\n std_msgs\n medusa_msgs\n rospy\n)\n\ncatkin_python_setup()\n\ncatkin_package(\n CATKIN_DEPENDS\n)"
ROS_INDEPENDENT_CLASS_CONTENT="#!/usr/bin/env python\n\ndef my_generic_sum_function(a, b):\n    \"\"\"\n    TODO!! documentation\n    \"\"\"\n    return a + b"
PACKAGE_XML_CONTENT="<?xml version=\"1.0\"?>\n<package format=\"2\">\n  <name>${PKG_NAME_IN}</name>\n  <version>1.0.0</version>\n  <description>\n    TODO!\n  </description>\n\n  <license>GPLv3</license>\n\n  <author email=\"${AUTHOR_EMAIL}\">${AUTHOR_NAME}</author>\n  <maintainer email=\"${MAINTAINER_EMAIL}\">${MAINTAINER_NAME}</maintainer>\n\n  <buildtool_depend>catkin</buildtool_depend>\n\n <depend>std_msgs</depend> \n\n <depend>rospy</depend>\n\n <depend>medusa_msgs</depend>\n\n</package>"
CONFIG_YAML_CONTENT="node_frequency: 10"
README_CONTENT="${PKG_NAME_IN} documentation"
LAUNCH_FILE_CONTENT="<?xml version=\"1.0\"?>\n<launch>\n\n    <!-- small description about your node -->\n    \n    <!--<node   pkg=\"my_package_name\" type=\"my_node_name\" name=\"my_node_name\"\n            respawn=\"false\" output=\"screen\" args=\"\$(find ${PKG_NAME_IN})/config/my_arg_file.yaml\"/>-->\n    \n    <node   pkg=\"${PKG_NAME_IN}\" type=\"${PKG_NAME_IN}_node\" name=\"${PKG_NAME^}Node\" respawn=\"false\" output=\"screen\">\n\t<rosparam command=\"load\" file=\"\$(find ${PKG_NAME_IN})/config/config_${PKG_NAME_IN}.yaml\"/>\n</node>\n\n</launch>"
SCRIPTS_CONTENT="#!/usr/bin/env python\n\nimport ${PKG_NAME_IN}_ros.${PKG_NAME^}Node\n\nif __name__ == '__main__':\n    ${PKG_NAME_IN}_ros.${PKG_NAME^}Node.main()"
MY_TEST_CONTENT="#TODO"
SETUP_PY_CONTENT="#!/usr/bin/env python\n\nfrom distutils.core import setup\nfrom catkin_pkg.python_setup import generate_distutils_setup\n\n# for your packages to be recognized by python\nd = generate_distutils_setup(\n  packages=['${PKG_NAME_IN}_algorithms', '${PKG_NAME_IN}_ros'],\n  package_dir={'${PKG_NAME_IN}_algorithms': 'src/${PKG_NAME_IN}_algorithms', '${PKG_NAME_IN}_ros': 'src/${PKG_NAME_IN}_ros'}\n)\n\nsetup(**d)"
ROS_NODE_CONTENT="#!/usr/bin/env python\n\n\"\"\" \nDevelopers: DSOR Team  -> @isr.ist.pt Instituto Superior Tecnico\nDescription: Please check the documentation of this package for more info.\n\"\"\"\nimport rospy\nfrom ${PKG_NAME_IN}_algorithms.${PKG_NAME^}Algorithm import my_generic_sum_function\nfrom std_msgs.msg import Int8, Bool\n\nclass ${PKG_NAME^}Node():\n\tdef __init__(self):\n\t\t\"\"\"\n\t\tConstructor for ros node\n\t\t\"\"\"\n\n\t\t\"\"\"\n\t\t###########################################################################################\n\t\t@.@ Init node\n\t\t###########################################################################################\n\t\t\"\"\"\n\t\trospy.init_node('${PKG_NAME_IN}_node')\n\n\t\t\"\"\"\n\t\t###########################################################################################\n\t\t@.@ Handy Variables\n\t\t###########################################################################################\n\t\t# Declare here some variables you might think usefull -> example: self.fiic = true\n\n\t\t\"\"\"\n\t\tself.h_timerActivate = False\n\n\t\t\"\"\"\n\t\t###########################################################################################\n\t\t@.@ Dirty work of declaring subscribers, publishers and load parameters \n\t\t###########################################################################################\n\t\t\"\"\"\n\t\tself.initializeSubscribers()\n\t\tself.initializePublishers()\n\t\tself.loadParams()\n\n\t\t\"\"\"\n\t###########################################################################################\n\t@.@ Member Helper function to set up subscribers; \n\t###########################################################################################\n\t\"\"\"\n\tdef initializeSubscribers(self):\n\t\trospy.loginfo('Initializing Subscribers for ${PKG_NAME^}Node')\n\t\trospy.Subscriber('Flag', Int8, self.wpStatusCallback)\n\n\t\"\"\"\n\t###########################################################################################\n\t@.@ Member Helper function to set up publishers; \n\t###########################################################################################\n\t\"\"\"\n\tdef initializePublishers(self):\n\t\trospy.loginfo('Initializing Publishers for ${PKG_NAME^}Node')\n\t\tself.wp_status_timer_pub = rospy.Publisher('wp_status', Bool, queue_size=10)\n\n\t\"\"\"\n\t###########################################################################################\n\t@.@ Member Helper function to set up parameters; \n\t###########################################################################################\n\t\"\"\"\n\tdef loadParams(self):\n\t\tself.node_frequency = rospy.get_param('~node_frequency', 10)\n\n\t\"\"\"\n\t###########################################################################################\n\t@.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate \n\t###########################################################################################\n\t\"\"\"\n\tdef initializeTimer(self):\n\t\tself.timer = rospy.Timer(rospy.Duration(1.0/self.node_frequency),self.timerIterCallback)\n\t\tself.h_timerActivate = True\n\n\t\"\"\"\n\t###########################################################################################\n\t@.@ Member helper function to shutdown timer;\n\t###########################################################################################\n\t\"\"\"\n\tdef shutdownTimer(self):\n\t\tself.timer.shutdown()\n\t\tself.h_timerActivate = False\n\n\t\"\"\"\n\t###########################################################################################\n\t@.@ Callback functions/methods \n\t###########################################################################################\n\t\"\"\"\n\n\t# +.+ flag callback\n\tdef wpStatusCallback(self, flag):\n\t\tif self.h_timerActivate and flag.data != Int8(4).data:\n\t\t\trospy.loginfo('Timer will Stop')\n\t\t\tself.shutdownTimer()\n\t\telif not self.h_timerActivate and flag.data == Int8(4).data:\n\t\t\trospy.loginfo('Timer will start')\n\t\t\tself.initializeTimer()\n\n\t# +.+ timer iter callback\n\tdef timerIterCallback(self, event=None):\n\t\tself.wp_status_timer_pub.publish(True)\n\ndef main():\n\tprint 'success'\n\n\t${PKG_NAME} = ${PKG_NAME^}Node()\n\n\t# +.+ Added to work with timer -> going into spin; let the callbacks do all the work\n\n\trospy.spin()\n\nif __name__ == '__main__':\n\tmain()" 

###############################################################
# @.@ create folder structure                                 #  
###############################################################
mkdir ${PKG_NAME_IN} 
cd ${PKG_NAME_IN}
mkdir -p config
mkdir -p doc
mkdir -p launch
mkdir -p scripts
mkdir -p src/${PKG_NAME_IN}_algorithms
mkdir -p src/${PKG_NAME_IN}_ros
mkdir -p test

################################################################
# @.@ create file structure                                    # 
################################################################
touch CMakeLists.txt
touch src/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.py
touch package.xml
touch config/config_${PKG_NAME_IN}.yaml
touch doc/README.md
touch launch/${PKG_NAME_IN}.launch
touch scripts/${PKG_NAME_IN}_node
touch src/${PKG_NAME_IN}_ros/__init__.py
touch src/${PKG_NAME_IN}_algorithms/__init__.py
touch src/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.py
touch test/${PKG_NAME_IN}_test.py
touch setup.py

################################################################
# @.@ make python node executable:                             # 
################################################################
chmod +x scripts/${PKG_NAME_IN}_node

################################################################
# @.@ fill files with information from parameter section       #
################################################################
echo -e $CMAKELISTS_CONTENT  > CMakeLists.txt
echo -e $ROS_INDEPENDENT_CLASS_CONTENT > src/${PKG_NAME_IN}_algorithms/${PKG_NAME^}Algorithm.py
echo -e $PACKAGE_XML_CONTENT > package.xml
echo -e $CONFIG_YAML_CONTENT > config/config_${PKG_NAME_IN}.yaml
echo -e $README_CONTENT > doc/README.md
echo -e $LAUNCH_FILE_CONTENT > launch/${PKG_NAME_IN}.launch 
echo -e $SCRIPTS_CONTENT > scripts/${PKG_NAME_IN}_node
echo -e $MY_TEST_CONTENT > test/${PKG_NAME_IN}_test.py
echo -e $SETUP_PY_CONTENT > setup.py
echo -e $ROS_NODE_CONTENT > src/${PKG_NAME_IN}_ros/${PKG_NAME^}Node.py

################################################################
# @.@ Make or build the workspace, you decide                  #
################################################################
#catkin build --this
medusa_cm

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
unset ROS_INDEPENDENT_CLASS_CONTENT
unset SETUP_PY_CONTENT

################################################################
# @.@ Source bashrc and echo an ecouriging goodbye message     #
################################################################
source ~/.bashrc
echo "${PKG_NAME_IN} ros py package created, you are now inside it. Let the fun begin..." 
unset PKG_NAME_IN