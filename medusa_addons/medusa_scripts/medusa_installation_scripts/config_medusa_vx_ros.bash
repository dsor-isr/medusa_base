#!/usr/bin/env bash

# '####################################################'
# ' DSOR: MEDUSAS TEAM                                 '
# ' Last version date: 30/03/2021                      '
# ' Supports: Ubuntu 18.04LTS                          '
# '           Ubuntu 20.04LTS                          '
# '                                                    '
# '                                                    '
# ' Installation script by:                            '
# '                                                    '
# '    @name: Marcelo Jacinto                          '
# '    @email: marcelo.jacinto@tecnico.ulisboa.pt      '
# '                                                    '
# ' Updated by:                                        '  
# '          malaclemys                                '
# '                                                    '
# ' Based on the previous scripts from: Joao Quintas   '
# '                                     Shubahm Garg   '
# '                                                    '
# ' Tested by: Joao Quintas                            '
# '            David Souto                             '
# '            Francisco Branco                        '
# '####################################################'

# ------------------------------------------------
# BADGE WITH THE CODERS
# ------------------------------------------------
echo ''
echo '################# Welcome to #######################'
echo '    __  ___         __                      __      '                    
echo '   /  |/  /__  ____/ /_  ___________ _     _||____  '     
echo '  / /|_/ / _ \/ __  / / / / ___/ __ `/    (_______) '     
echo ' / /  / /  __/ /_/ / /_/ (__  ) /_/ /      _||____  '     
echo '/_/  /_/\___/\__,_/\__,_/____/\__,_/      (_______) '     
echo '                                                    '
echo '############### Enjoy the ride #####################'
echo ''

###############################################################################################
###############################################################################################
###############################################################################################
# THESE ARE THE ONLY FUNCTION DECLARED ON THE TOP OF THIS SCRIPT, BECAUSE THERE ARE MANY
# VARIABLES THAT DEPEND ON CATKIN_ROOT, WHICH IS SET BY THIS FUNCTION
# ALSO, INTERPRETING THE ARGUMENTS THIS SCRIPT IS RUNNING WITH SHOULD BE DONE IN THE BEGINING

# SPECIAL FUNCTION DECLARATION TO CHOOSE CATKIN_ROOT
function choose_path_catkin_root(){

  printf "\n\n\n"
  echo "---------------------------------------------"
  echo "| Choosing catkin_ws installation location  |"
  echo "---------------------------------------------"

  printf "\nDefault installation location: ${CATKIN_ROOT}\n"
  printf "Do you wish to change the default installation folder before continuing (y/n)?\n"
  read PROCEED_VAR

  echo "After successfully setting the installation folder, the script will now check for internet connectivity, set Ubuntu version and check for system updates\n"
  local CHANGE_LOCATION="FALSE"
  # Ask for user to change the location
  if [ "$PROCEED_VAR" != "${PROCEED_VAR#[Yy]}" ] ; then
    local CHANGE_LOCATION="TRUE"
  fi

    # If the user want to change the default folder, then ask the new directory
    while [ $CHANGE_LOCATION == "TRUE" ] ; do

        # Get from the user the new path for ${CATKIN_ROOT}
        printf "\nSelecting a new CATKIN_ROOT directory\n"
        printf "Insert the new installation directory (full path - I also accept the ~ character): \n"
        read INSTALLATION_DIR

        # Remove the trailing "/" at the end, if it has one
        local length=${#INSTALLATION_DIR}
        local last_char=${INSTALLATION_DIR:length-1:1}
        [[ $last_char == "/" ]] && INSTALLATION_DIR=${INSTALLATION_DIR:0:length-1}; :

        # Expand the directory if it finds ~ as the first character
        local first_char=${INSTALLATION_DIR:0:1}
        if [[ $first_char == "~" ]] ; then
          local HOME_DIRECTORY=~
          INSTALLATION_DIR="${HOME_DIRECTORY}${INSTALLATION_DIR:1:length}"
        fi

        # Check if the user selected the desired directory properly
        printf "\nIs this your desired directory: ${INSTALLATION_DIR} (y/n)?"
        read PROCEED_VAR

        # If user is satisfied with his choice, save the new catkin_root directory
        if [ "$PROCEED_VAR" != "${PROCEED_VAR#[Yy]}" ] ; then
          # Stop executing the loop
          local CHANGE_LOCATION="FALSE"

            # Set the new catkin_root directory
            CATKIN_ROOT=$INSTALLATION_DIR

            # If the directory does not exist yet, create it
            if [ ! -d "${CATKIN_ROOT}" ]; then
              # Showing the command to the user
              echo "mkdir -p ${CATKIN_ROOT}"
              # Actually executing the command
              sudo mkdir -p ${CATKIN_ROOT}
              # Give all permissions to the new CATKIN_ROOT folder
              #sudo chmod a+rwx ${CATKIN_ROOT}
            fi
        fi

      done

    # Unset used variables
    unset CHANGE_LOCATION
    unset PROCEED_VAR
    unset INSTALLATION_DIR
    unset HOME_DIRECTORY
    unset length
    unset last_char
    unset first_char
  }

# SPECIAL FUNCTION TO DECLARE THE SPECIAL ARGUMENTS ONE CAN SPECIFY WHEN RUNNING THIS SCRIPT
function help_arguments() {
  echo "Script to install ROS and all the medusa code stack"
  echo ""
  echo "-V | --version          Version of this script                                   "
  echo "-h | -H | --help        Help with the optional parameters for this script        "
  echo "-mr | --medusa_repo     The name of the medusa code repository                   "
  echo "-mb | --medusa_branch   The name of the branch to use in the medusa code         "
  echo "-cr | --console_repo    The name of the medusa console repository                "
  echo "-cb | --console_branch  The name of the branch to use in the medusa console      "
  echo "-dr | --docs_repo       The name of the medusa documentation repository          "
  echo "-db | --docs_branch     The name of the branch to use in the medusa documentation"
}

# SPECIAL FUNCTION TO INTERPRET EXECUTION ARGUMENTS
# args accessed with $1: the cmd arguments
function interpret_cmd_arguments() {
  while [ -n "$1" ]; do # while loop starts
    case "$1" in
      -V | --version )
        echo "Script version: ${VERSION}"
        echo "Coded by: Marcelo Jacinto"
        echo ""
        ;;
      -h | -H | --help )
        help_arguments
        echo ""
        echo "Exiting shell in 15 seconds..."
        sleep 15
        exit 1
        ;;
      -mr | --medusa_repo )
        local param="$2"
        BITBUCKET_MEDUSA_REPO=$param
        shift
        ;;
      -mb | --medusa_branch )
        local param="$2"
        MEDUSA_GIT_BRANCH=$param
        shift
        ;;
      -cr | --console_repo )
        local param="$2"
        BITBUCKET_CONSOLE_REPO=$param
        shift
        ;;
      -cb | --console_branch )
        local param="$2"
        CONSOLE_GIT_BRANCH=$param
        shift
        ;;
      -dr | --docs_repo )
        local param="$2"
        BITBUCKET_DOCUMENTATION_REPO=$param
        shift
        ;;
      -db | --docs_branch )
        local param="$2"
        DOCUMENTATION_GIT_BRANCH=$param
        shift
        ;;
      -U  | --username )
        local param="$2"
        BITBUCKET_USERNAME=$param
        shift
        ;;
      -P  | --password )
        local param="$2"
        BITBUCKET_PASSWORD=$param
        shift
        ;;
      *) echo "Option $1 not recognized" ;;
    esac
    shift
  done

}

###############################################################################################
###############################################################################################
###############################################################################################

#--------------------------------------------------------------
#   ------------- GLOBAL VARIABLES DECLARATION -------------
#--------------------------------------------------------------
# Get all the execution arguments received when this script was called
CMD_ARGUMENTS=$@

#--------------------------------------------------------------
# VARIABLE THAT CAN BE CHANGED AS CMD ARGUMENTS WHEN RUNNING THE SCRIPT
#--------------------------------------------------------------

# Version of this script
VERSION=1.1

# The ubuntu version where the medusa stack is being installed
UBUNTU_VERSION=""
ROS_VERSION_NAME=""

# Bitbucket information for cloning a repository
BITBUCKET_MEDUSA_REPO="medusa_vx"
MEDUSA_GIT_BRANCH="development"

# Bitbucket information for cloning the console
BITBUCKET_CONSOLE_REPO="console_fix"
CONSOLE_GIT_BRANCH="master"

# Bitbucket information for cloning the documentation
BITBUCKET_DOCUMENTATION_REPO="medusa_docs"
DOCUMENTATION_GIT_BRANCH="main"

# Bitbucket credentials for cloning from the repositories
BITBUCKET_USERNAME=""
BITBUCKET_PASSWORD=""

# Interpret the CMD Arguments which can receive environmental variables specified above
interpret_cmd_arguments $CMD_ARGUMENTS

#--------------------------------------------------------------
# VARIABLE THAT CAN NOT BE CHANGED AS CMD ARGUMENTS
#--------------------------------------------------------------

# The catkin_root directory
CATKIN_ROOT=~
choose_path_catkin_root # Special function called here, because everything depends on catkin_root

# Usefull Alias location and catkin root
ALIAS_PKG="medusa_permanent_alias/alias.sh"
ALIAS_LOCATION=$CATKIN_ROOT/catkin_ws/src/$BITBUCKET_MEDUSA_REPO/medusa_scripts/system_configurations/easy_alias/$ALIAS_PKG

# Which type of instalation is being performed
# Supported tags: DEV_MACHINE, DOCKER, ROBOT_INSTALL - FLAG not being used for now
INSTALATION_METHOD="DEV_MACHINE"

# Flag to check if there was an error when running a function
FUNCTION_ERROR="FALSE"

# Time to wait before closing shell in case a function throws an error
ABORT_SLEEP_TIME=15     

# Basic necessary programs that must be installed before the main installation starts
BASIC_NECESSARY_PROGRAMS=(
  curl              # Curl - to make an https call - might me needed when trying to add the ROS repository signed key
  git               # Git - to clone the repositories with the packages
  wget              # Wget - to download packages manually without clonning git repositories
  unzip             # Unzip - to unzip zipped folders
  net-tools         # net-tools - to check the computer IP using ifconfig command
  nano              # Nano - simple text editor
  vim               # Vim - the most powerfull code editor EVER XD
  ssh               # SSH - to allow remote connections using SSH protocol
  terminator        # Terminator - to have a terminal with split windows
  expect            # Expect - for running eco vpn
  openvpn           # OpenVPN - allows to connect to vpns
  iputils-ping      # IPUtils - set of small useful utilities for Linux networking.
  gnupg2            # GNUpg2 - an encryption tool that includes digital signatures and certificates.
  lsb-release       # LSB - simple tool to help identify the Linux distribution being used and its compliance with the Linux Standard Base
)

# External Repositories with code not made by DSOR, that are used in medusa
EXTERNAL_RESPOS_FOLDER=3rdParty
EXTERNAL_REPOS_LOCATION=$CATKIN_ROOT/catkin_ws/src/${EXTERNAL_RESPOS_FOLDER}
EXTERNAL_REPOS=(
  "https://github.com/okebkal/dmac"                            # dmac
  "https://github.com/team-vigir/flexbe_behavior_engine.git"   # flexbe_behavior_engine
  "https://github.com/ros-drivers/nmea_msgs"                   # nmea_msgs
  "https://github.com/oceansystemslab/auv_msgs"		         # auv_msgs
)

# Packages to ignore during compilation in a development machine
PACKAGES_TO_IGNORE=(
  medusa_drivers
)

# Python packages to install
PYTHON_PACKAGES=(
  mkdocs              # For generating and reading the documentation
  scipy               # Scipy python package
  numpy               # Numpy python package, operations with matrix
  pandas              # Pandas python package, cool when working with big datasets
  matplotlib          # Matplotlib python package - good for ploting graphics

    # In case ros does not install these libraries automatically
    rospkg              # Rospkg package - to use ROS methods - ROS NOETIC does not install this for python2 automatically
    catkin_pkg          # Catkin package
    future              # for http server console
  )

# Custom scripts to run after ROS installation (to run with: sudo bash $CUSTOM_SCRIPTS_TO_RUN_AFTER_ROS_INSTALL)
CUSTOM_SCRIPTS_TO_RUN_AFTER_ROS_INSTALL=(
  ${CATKIN_ROOT}/catkin_ws/src/${BITBUCKET_MEDUSA_REPO}/medusa_scripts/system_configurations/hosts_files/hosts_cpy.sh
)

# Libraries not specific to ROS to install (to run with: sudo apt-get install $APT_GET_INSTALL_LIBRARIES)
APT_GET_INSTALL_LIBRARIES=(
  libudev-dev
  libusb-1.0-0-dev
  libgeographic-dev
  libxmlrcpp-dev
  librosconsole-dev
)

# Extra ROS-PACKAGES to install (to run with: sudo apt-get install ros-$ROS_VERSION_NAME-$EXTRA_ROS_PACKAGES)
EXTRA_ROS_PACKAGES=(
  nmea-msgs
  geographic-msgs         # Geographic messages, which makes use of GeographicLib (cpp library)
  dynamic-reconfigure     # dynamic-reconfigure to allow dynamical change of controllers parameters in the code
  rosbridge-suite         # rosbridge-suite (necessary to work with the console)
)

# Estra ROSDEP-PACKAGES to install (to run with: rosdep install $ROSDEP_PACKAGES)
ROSDEP_PACKAGES=(
  )

# Extra developer tools that are not necessary to run the code
# but are a nice addition
EXTRA_DEVELOPER_TOOLS=(
  exfat-fuse        # to acces exfat hard drives
  terminator        # terminal with split windows option
  tree              # draw (print) folder structure from terminal
  python-pygments   # to cat files in colors
  wmctrl            # to maximize a terminal from cmd line 
  ntp               # network time protocol for sync pcs
  ntpdate           # network time protocol for clients have the capability to sync with server
  minicom           # to talk with serial ports
  byobu             # simple tmux
  zip               # for zipping files
  screen            # terminal multiplexer, allows to open any number of windows(virtual terminals) iniside a session
)

#--------------------------------------------------------------
#   ----------------- FUNCTION DECLARATION -----------------
#--------------------------------------------------------------

# Function to test if there is internet connection before starting the installation
function test_internet_connection() {

  echo "After the system is updated, the script will request you bitbucket credentials to clone the medusa stack repository\n"
  # Restore the function error flag
  FUNCTION_ERROR="FALSE"

  printf "\n\n\n"
  echo "---------------------------------------------"
  echo "| Checking if there is internet connection  |"
  echo "---------------------------------------------"   

  local INTERNET_CONNECTION_TEST=""

    # Try to ping google (in silent mode)
    ping -q -w1 -c1 google.com &>/dev/null && INTERNET_CONNECTION_TEST="UP" || INTERNET_CONNECTION_TEST="DOWN"

    # Print the result of the test to the user
    if [ $INTERNET_CONNECTION_TEST == "UP" ] ; then
      echo ""
      echo "Internet connection is UP"
      echo ""
    else
      echo ""
      echo "The internet connection is DOWN or VERY SLOW"
      echo ""
      # Signal that there was an error
      FUNCTION_ERROR="TRUE"
    fi
  }

# Function to check the version of Ubuntu
function check_ubuntu_version() {

    # Restore the function error flag
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "---------------------------------------------"
    echo "| Checking Ubuntu Version                   |"
    echo "---------------------------------------------"    

    # Check the ubuntu version
    UBUNTU_VERSION=$(sed -n 's/VERSION_ID=//p' /etc/os-release)
    local EXIT_STATUS=$?

    # Check if there was a problem executing the commands
    local EXIT_STATUS=$?
    if [ $EXIT_STATUS -ne 0 ] ; then
      FUNCTION_ERROR="TRUE"
    elif [ $EXIT_STATUS -eq 0 ] ; then
      echo "UBUNTU VERSION: ${UBUNTU_VERSION}"

        # Set the ROS_VERSION_NAME
        # Note: careful - use notation "\"20.04\"" because the way ubuntu_version comes
        if [ "${UBUNTU_VERSION}" == "\"20.04\"" ] ; then
          ROS_VERSION_NAME=noetic
        elif [ "${UBUNTU_VERSION}" == "\"18.04\"" ] ; then
          ROS_VERSION_NAME=melodic
        else
          echo "ERROR: Ubuntu version not supported by this script"
          echo "       Will exit in ${ABORT_SLEEP_TIME} seconds"
          sleep $ABORT_SLEEP_TIME
          exit
        fi

        echo "ROS VERSION: ${ROS_VERSION_NAME}"
    fi
  }

# Function to append a bash to bashrc and CATKIN_ROOT variable
function prepare_bash_rc_to_receive_data() {

    # Restore the function error flag
    # No installation checks are done in this function
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "----------------------------------------------"
    echo "| Appending catkin_root variable to ~/.bashrc|"
    echo "----------------------------------------------"   

    # Print on the screen what will happen
    echo "echo export CATKIN_ROOT=${CATKIN_ROOT} >> ~/.bashrc "

    # Actually doing the action of saving to bash rc
    echo "" >> ~/.bashrc
    echo "#----------------------------------------------" >> ~/.bashrc
    echo "#| MEDUSA CONFIGURATIONS BELLOW               |" >> ~/.bashrc
    echo "#----------------------------------------------" >> ~/.bashrc
    echo "" >> ~/.bashrc
    echo "export CATKIN_ROOT=${CATKIN_ROOT}" >> ~/.bashrc
  }

# Function to receive bitbucket credentials (username and password)
function receive_BITBUCKET_credentials() {

    # Restore the function error flag
    # No installation checks are done in this function
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "---------------------------------------------"
    echo "| Getting BITBUCKET Credentials             |"
    echo "---------------------------------------------"
    printf "\n\nBITBUCKET USERNAME: "
    read BITBUCKET_USERNAME
    printf "BITBUCKET PASSWORD: "
    stty -echo
    read BITBUCKET_PASSWORD
    stty echo
  }

# Function to create a root instalation folder for the packages
function create_root_folder() {
  # Restore the function error flag
  FUNCTION_ERROR="FALSE"

  printf "\n\n\n"
  echo "---------------------------------------------"
  echo "| Creating ${CATKIN_ROOT}/catkin_ws          "
  echo "---------------------------------------------"

    # Save the current directory
    local current_directory=$PWD
    # Check if the catkin_ws directory already exists
    local DIRECTORY=$CATKIN_ROOT/catkin_ws

    # If the directory exits, prompt a message saying it will removed
    if [ -d "$DIRECTORY" ] ; then
      # Print the message asking the user if wants to proceed
      printf "\nBACKUP WARNING: If you have a folder named ${CATKIN_ROOT}/catkin_ws, IT WILL BE DELETED\n"
      printf "IF YOU WISH TO KEEP THIS FOLDER, BACK IT UP BEFORE CONTINUING\n"
      printf "DO YOU WISH TO PROCEED (y/n)?\n"
      read PROCEED_VAR
      # If user still wants to proceed
      if [ "$PROCEED_VAR" != "${PROCEED_VAR#[Yy]}" ] ; then
        echo Yes
        # Create catkin_ws and destroy an older one if exists
        cd $CATKIN_ROOT
        sudo rm -R $CATKIN_ROOT/catkin_ws
        mkdir catkin_ws
        # Give all permissions to catkin_ws and all folders inside
        #sudo chmod a+rwx catkin_ws
        cd catkin_ws
        mkdir src
        echo "Deleted old ${CATKIN_ROOT}/catkin_ws and created a new one"
        # If user wants to abort
      else
        # If there was no permission to delete the catkin_ws folder, then 
        # throw an error 
        FUNCTION_ERROR="TRUE"
      fi
      # If the directory does not exist, just create it
    else
      cd $CATKIN_ROOT
      mkdir catkin_ws
      cd catkin_ws
      mkdir src
      echo "Created ${CATKIN_ROOT}/catkin_ws"
    fi

    # Restore the previous directory
    cd $current_directory
  }

# Function to check for system and packages updates
function check_for_system_update() {

    # Restore the function error flag
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "---------------------------------------------"
    echo "| Checking for system updates               |"
    echo "---------------------------------------------"

    sudo apt-get update
    sudo apt-get upgrade -y 

    # Check if there was a problem executing the commands
    local EXIT_STATUS=$?
    if [ $EXIT_STATUS -ne 0 ] ; then
      FUNCTION_ERROR="TRUE"
    fi
  }

# Function to install basic programs that are always needed for a developer
function install_basic_programs() {

    # Restore the function error flag
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "---------------------------------------------"
    echo "| Installing basic programs                 |"
    echo "---------------------------------------------"
    sudo apt-get install ${BASIC_NECESSARY_PROGRAMS[@]} -y 

    # Clean the installation removing no longer used packages
    sudo apt-get clean all

    # Check if there was a problem executing the commands
    local EXIT_STATUS=$?
    if [ $EXIT_STATUS -ne 0 ] ; then
      FUNCTION_ERROR="TRUE"
    fi
  }

# Function to clone the main medusa repository
# args: $1 - location where to clone the repository 
function clone_medusa_repo() {

    # Restore the function error flag
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "---------------------------------------------"
    echo "| Cloning medusa repository                 |"
    echo "---------------------------------------------"

    # Save the current directory
    local current_directory=$PWD
    # Go to the desired directory passed as input argument
    echo $1
    cd $1
    # Check if there was already a medusa repository. If there was, delete it
    # and clone from scratch
    local DIRECTORY=$BITBUCKET_MEDUSA_REPO
    if [ -d "$DIRECTORY" ] ; then
      echo ""
      echo "Deleting old ${BITBUCKET_MEDUSA_REPO}"
      echo "Cloning new ${BITBUCKET_MEDUSA_REPO}"
      echo ""
      sudo rm -R $BITBUCKET_MEDUSA_REPO
    fi
    # Clone the repository
    git clone https://$BITBUCKET_USERNAME:$BITBUCKET_PASSWORD@bitbucket.org/dsor_isr/$BITBUCKET_MEDUSA_REPO.git

    # Check if the desired branch is not main and checkout to it
    if [[ "$BITBUCKET_MEDUSA_REPO" != "main" ]] ; then
      cd $BITBUCKET_MEDUSA_REPO
      # Checkout to the desired stable branch of the project
      git checkout --track origin/$MEDUSA_GIT_BRANCH
      # Check if there was any problem executing the command
      local EXIT_STATUS=$?
      # Go back to the previous directory
      cd $current_directory
      # Check if there was a problem executing the commands
      if [ $EXIT_STATUS -ne 0 ] ; then
        FUNCTION_ERROR="TRUE"
      fi
    fi
  }

# Function to clone additional necessary packages
# This function installs other repositories with stuff needed such as ros messages, etc...
# args: $1 - location where to clone the repository 
function clone_additional_repos() {
  # Restore the function error flag
  # No installation checks are done in this function
  FUNCTION_ERROR="FALSE"

  printf "\n\n\n"
  echo "---------------------------------------------------------"
  echo "| Installing external ROS1 PACKAGES:                    |"
  echo "---------------------------------------------------------"

    # Save the current directory
    local current_directory=$PWD

    # Check if the directory passed as parameter exists. If it does not, create it
    if [ ! -d "${1}" ]; then
      # Showing the command to the user
      echo "mkdir -p ${1}"
      # Actually executing the command
      mkdir -p ${1}
    fi
    # Give all permissions to the folder
    #sudo chmod a+rwx ${1}

    # Go to the desired directory passed as input argument
    cd $1

    # Clonning all the extra repos  
    for i in ${EXTERNAL_REPOS[@]}
    do  
      echo ""
      echo "->Clonning: ${i}"
      echo ""
      git clone ${i}
    done

    # Go back to the previous directory
    cd $current_directory
  }

# Function to clone the medusa console repository
# args: $1 - location where to clone the repository 
function clone_medusa_console() {

    # Restore the function error flag
    FUNCTION_ERROR="FALSE"


    printf "\n\n\n"
    echo "---------------------------------------------"
    echo "| Cloning medusa console                    |"
    echo "---------------------------------------------"

     # Save the current directory
     local current_directory=$PWD
     # Go to the desired directory passed as input argument
     cd $1
     # Check if there was already a medusa console. If there was, delete it
     # and clone from scratch
     local DIRECTORY=$BITBUCKET_CONSOLE_REPO
     if [ -d "$DIRECTORY" ] ; then
       echo ""
       echo "Deleting old ${BITBUCKET_CONSOLE_REPO}"
       echo "Cloning new ${BITBUCKET_CONSOLE_REPO}"
       echo ""
       sudo rm -R $BITBUCKET_CONSOLE_REPO
     fi

     # Clone the repository
     git clone https://$BITBUCKET_USERNAME:$BITBUCKET_PASSWORD@bitbucket.org/dsor_isr/$BITBUCKET_CONSOLE_REPO.git
     cd $BITBUCKET_CONSOLE_REPO
     # Checkout to the desired stable branch of the project
     # git checkout --track origin/$CONSOLE_GIT_BRANCH
     # Check if there was any problem executing the command
     local EXIT_STATUS=$?
     # Go back to the previous directory
     cd $current_directory

     # Check if there was a problem executing the commands
     local EXIT_STATUS=$?
     if [ $EXIT_STATUS -ne 0 ] ; then
       FUNCTION_ERROR="TRUE"
     fi

   }

# Function to clone the medusa documentation repository
# args: $1 - location where to clone the repository 
function clone_medusa_documentation() {

    # Restore the function error flag
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "---------------------------------------------"
    echo "| Cloning medusa documentation              |"
    echo "---------------------------------------------"

    # Save the current directory
    local current_directory=$PWD
    # Go to the desired directory passed as input argument
    cd $1
    # Check if there was already a medusa documentation folder. If there was, delete it
    # and clone from scratch
    local DIRECTORY=$BITBUCKET_DOCUMENTATION_REPO
    if [ -d "$DIRECTORY" ] ; then
      echo ""
      echo "Deleting old ${BITBUCKET_DOCUMENTATION_REPO}"
      echo "Cloning new ${BITBUCKET_DOCUMENTATION_REPO}"
      echo ""
      sudo rm -R $BITBUCKET_DOCUMENTATION_REPO
    fi
    # Clone the repository
    git clone https://$BITBUCKET_USERNAME:$BITBUCKET_PASSWORD@bitbucket.org/dsor_isr/$BITBUCKET_DOCUMENTATION_REPO.git
    cd $BITBUCKET_DOCUMENTATION_REPO
    # Checkout to the desired stable branch of the project
    # git checkout --track origin/$DOCUMENTATION_GIT_BRANCH
    local EXIT_STATUS=$?
    # Go back to the previous directory
    cd $current_directory

    # Check if there was a problem executing the commands
    local EXIT_STATUS=$?
    if [ $EXIT_STATUS -ne 0 ] ; then
      FUNCTION_ERROR="TRUE"
    fi
  }

# Function to install ROS MELODIC (UBUNTU-18.04LTS)
function install_ros_melodic() {

    # Restore the function error flag
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "---------------------------------------------"
    echo "| Installing ROS MELODIC                    |"
    echo "---------------------------------------------"

    # Add the ROS sources 
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    # Get a key from keyserver with timeout if keyserver is not working
    timeout 10 curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
    local EXIT_STATUS=$?
    # If it times_out, EXIT_STATUS != 0 and we should try another keyserver
    if [ $EXIT_STATUS -ne 0 ] ; then
      timeout 10 sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
      local EXIT_STATUS=$?
    fi
    # If there was no problem getting a key, start installing ROS
    if [ $EXIT_STATUS -eq 0 ] ; then
      sudo apt update
      sudo apt install ros-melodic-desktop-full -y 
      echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
      source ~/.bashrc

        # Extra ROS dependencies for building packages (extra step not mentioned in ros webpage)
        sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
        sudo apt-get install ros-melodic-rosbridge-suite -y 

        # Initialize ROSDEP
        sudo apt install python-rosdep
        sudo rosdep init
        rosdep update
        local EXIT_STATUS=$?
    fi

  }

# Function to install ROS NOETIC (UBUNTU-20.04LTS)
function install_ros_noetic() {

    # Restore the function error flag
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "---------------------------------------------"
    echo "| Installing ROS NOETIC                     |"
    echo "---------------------------------------------"

    # Add the ROS sources 
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    # Get a key from keyserver with timeout if keyserver is not working
    timeout 10 curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
    local EXIT_STATUS=$?
    # If it times_out, EXIT_STATUS != 0 and we should try another keyserver
    if [ $EXIT_STATUS -ne 0 ]; then
      timeout 10 sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
      local EXIT_STATUS=$?
    fi
    # If there was no problem getting a key, start installing ROS
    if [ $EXIT_STATUS -eq 0 ] ; then
      sudo apt update
      sudo apt install ros-noetic-desktop-full -y 
      echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
      source ~/.bashrc
      # Install rosdep
      sudo apt-get install python3-rosdep
      # Initialize ROSDEP
      sudo rosdep init
      rosdep update
      local EXIT_STATUS=$?
    fi

    # Check if there was a problem executing the commands
    if [ $EXIT_STATUS -ne 0 ] ; then
      FUNCTION_ERROR="TRUE"
    fi
  }

# Function to install extra libraries using: sudo apt-get install $APT_GET_INSTALL_LIBRARIES
function install_extra_libraries() (

    # Restore the function error flag
    # No installation checks are done in this function
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "---------------------------------------------"
    echo "| Installing EXTRA LIBRARIES                |"
    echo "---------------------------------------------"

    # Installing all the extra necessary ros packages
    for i in ${APT_GET_INSTALL_LIBRARIES[@]}
    do
      echo ""
      echo "->Installing: ${i}"
      echo ""
      sudo apt-get install "${i}" -y
    done
  )

# Function to install extra ros packages using: sudo apt-get install ros-$ROS_VERSION_NAME-$EXTRA_ROS_PACKAGES
function install_extra_ros_packages() {

    # Restore the function error flag
    # No installation checks are done in this function
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "---------------------------------------------"
    echo "| Installing EXTRA ROS PACKAGES             |"
    echo "---------------------------------------------"

    # Installing all the extra necessary ros packages
    for i in ${EXTRA_ROS_PACKAGES[@]}
    do
      echo ""
      echo "->Installing: ros-$ROS_VERSION_NAME-${i}"
      echo ""
      sudo apt-get install "ros-${ROS_VERSION_NAME}-${i}" -y
    done
  }

# Function to install rosdep packages using: rosdep install $ROSDEP_PACKAGES
function install_rosdep_packages() {

    # Restore the function error flag
    # No installation checks are done in this function
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "---------------------------------------------"
    echo "| Installing ROSDEP PACKAGES                |"
    echo "---------------------------------------------"

    # Installing all the extra necessary ros packages
    for i in ${ROSDEP_PACKAGES[@]}
    do
      echo ""
      echo "->Installing: ${i}"
      echo ""
      rosdep install "${i}"
    done
  }

# Function to setup python2 and python3 to work with 
# older packages and newer ones as well
function python_installation_20.04LTS() {

    # Restore the function error flag
    # No installation checks are done in this function
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "----------------------------------------------------------------------"
    echo "|        Installing python2                                          |"
    echo "|        Installing pip2 and pip3                                    |"
    echo "|        Installing scipy, numpy, pandas, matplotlib, rospkg, roslib |"
    echo "|        Binding python2 to python (for backwards compatibility)     |"
    echo "----------------------------------------------------------------------"

    # Save the current directory
    local current_directory=$PWD

    # Installing python2 and pip2 (needed for backwards compatibility with older packages)
    cd $CATKIN_ROOT
    sudo add-apt-repository universe
    sudo apt update
    sudo apt-get install python2 -y 
    curl https://bootstrap.pypa.io/get-pip.py --output get-pip.py
    sudo python2 get-pip.py
    sudo rm get-pip.py

    # Installing pip3 package manager for the newest python3
    sudo apt install python3-pip -y 

    # Python-tk is needed to use matplotlib in python2 packages
    sudo apt-get install python-tk -y 

    # Upgrade both pip2 and pip3
    sudo pip2 install --upgrade pip
    sudo pip3 install --upgrade pip

    # Install python modules for both python2 and python3
    for i in ${PYTHON_PACKAGES[@]}
    do
      echo ""
      echo "->Python2 - installing: ${i}"
      echo ""
      sudo pip2 install "${i}"

      echo ""
      echo "->Python3 - installing: ${i}"
      echo ""
      sudo pip3 install "${i}"
    done

    #Binding python2 package manager for the newest python3	
    sudo rm /usr/bin/python
    sudo ln -s /usr/bin/python3 /usr/bin/python

    # Go back to the previous directory
    cd $current_directory


  }

# Function to setup python2 and python3 to work with 
# older packages and newer ones as well
function python_installation_18.04LTS() {

    # Restore the function error flag
    # No installation checks are done in this function
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "----------------------------------------------------------------------"
    echo "|        Installing python2                                          |"
    echo "|        Installing pip2 and pip3                                    |"
    echo "|        Installing scipy, numpy, pandas, matplotlib, rospkg, roslib |"
    echo "|        Binding python2 to python (for backwards compatibility)     |"
    echo "----------------------------------------------------------------------"

    # Save the current directory
    local current_directory=$PWD

    # No need to install python2 - ROS melodic installation already does that
    # for us

    # Installing pip2
    sudo apt install python-pip -y

    # Installing python3 and pip3
    sudo apt install python3-pip -y

    # Python-tk is needed to use matplotlib in python2 packages
    sudo apt-get install python-tk -y 

    # Upgrade both pip2 and pip3
    sudo pip2 install --upgrade pip
    sudo pip3 install --upgrade pip

    # Install necessary additions for python3
    sudo apt-get install python3-yaml

    # Install python modules for both python2 and python3
    for i in ${PYTHON_PACKAGES[@]}
    do
      echo ""
      echo "->Python2 - installing: ${i}"
      echo ""
      sudo pip2 install "${i}"

      echo ""
      echo "->Python3 - installing: ${i}"
      echo ""
      sudo pip3 install "${i}"
    done

    # Go back to the previous directory
    cd $current_directory
  }

# Function to install external cpp libraries manually
# This function installs: Geographiclib (cpp)
function external_libraries_instalation() {

    # Restore the function error flag
    # No installation checks are done in this function
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "---------------------------------------------------------"
    echo "| Installing external libraries:                        |"
    echo "|          - Geographiclib (cpp)                        |"
    echo "---------------------------------------------------------"

    # Installing Geographiclib (cpp)
    sudo apt-get install libgeographic-dev ros-noetic-geographic-msgs -y 
    sudo apt install ros-noetic-geodesy -y 

    # Manually install the cpp library
    cd $CATKIN_ROOT
    wget https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.50.1.tar.gz/download
    tar xfpz download
    cd GeographicLib-1.50.1 
    mkdir BUILD
    cd BUILD
    cmake ..
    sudo make
    sudo make test
    sudo make install
    cd $CATKIN_ROOT
    sudo rm download
    sudo rm -R GeographicLib-1.50.1

    # Binding the cmake installation to the cmake-3.10 (now the new default)
    # NOTE: This should be done only as a last resource manually link the library so that cmake can find it
    # Bellow is the command to do it manually:
    # sudo ln -s /usr/share/cmake/geographiclib/FindGeographicLib.cmake /usr/share/cmake-3.16/Modules/
  }

# Function to give read, write and execution permission to everything inside the passed directory
# args: $1 - the directory to give permissions to
function give_read_write_execute_permisions() {
  # Restore the function error flag
  # No permission checks are done in this function
  FUNCTION_ERROR="FALSE"

  printf "\n\n\n"
  echo "---------------------------------------------------------"
  echo "| Giving read/write/execute permission:                  "
  echo "|        to everything inside ${CATKIN_ROOT}/catkin_ws   "
  echo "---------------------------------------------------------"

    # Give the execution permission
    echo ""
    echo "sudo chmod a+rwx ${1}"
    echo ""
    sudo chmod a+rwx $1
  }

# Function to ask if want to CATKIN_IGNORE driver package that are only necessary
# for the robots
function ignore_packages() {
  printf "\n Do you want to ignore the drivers package (y/n)? Press y if you will run the stack on simulation only.\n"
  read READ_VAR
  if [[ "$READ_VAR" != "${READ_VAR#[Yy]}" ]] ; then
    packages_to_ignore_in_dev
  fi
  FUNCTION_ERROR="FALSE"
}

# Function to place CATKIN_IGNORE in packages that are only necessary for the robots
# and not for the development computers (such as bluetooth drivers, USBL drivers, etc)
function packages_to_ignore_in_dev() {
  # Restore the function error flag
  # No compilation checks are done in this function
  FUNCTION_ERROR="FALSE"
  printf "\n\n\n"
  echo "---------------------------------------------------------"
  echo "| Touching CATKIN_IGNORE                                |"
  echo "---------------------------------------------------------"

    # ADD a CATKIN_IGNORE file to all packages inside the main
    # MEDUSA repo that need to be ignored
    echo ""
    echo "Ignoring the following packages:"
    for i in ${PACKAGES_TO_IGNORE[@]}
    do
      echo ""
      echo "${i}"
      echo "${CATKIN_ROOT}/catkin_ws/src/${BITBUCKET_MEDUSA_REPO}/${i}/CATKIN_IGNORE"
      touch $CATKIN_ROOT/catkin_ws/src/${BITBUCKET_MEDUSA_REPO}/${i}/CATKIN_IGNORE
    done
    echo ""

  }

# Function to execute custom bash script after ROS installation (to run with: sudo bash $CUSTOM_SCRIPTS_TO_RUN_AFTER_ROS_INSTALL)
function execute_custom_scripts() {

    # Restore the function error flag
    # No compilation checks are done in this function
    FUNCTION_ERROR="FALSE"

    printf "\n\n\n"
    echo "---------------------------------------------------------"
    echo "| Running custom bash scripts from medusa repo          |"
    echo "---------------------------------------------------------"

    for i in ${CUSTOM_SCRIPTS_TO_RUN_AFTER_ROS_INSTALL[@]}
    do
      echo ""
      echo "-> source ${i}"
      sudo bash ${i}
    done
    echo ""
  }

# Compile the medusa stack
function compile_medusa_code() {
  # Restore the function error flag
  # No compilation checks are done in this function
  FUNCTION_ERROR="FALSE"

  printf "\n\n\n"
  echo "---------------------------------------------------------"
  echo "| Compiling the code                                    |"
  echo "---------------------------------------------------------"

    # Save the current directory
    local current_directory=$PWD

    # Compile the code
    source /opt/ros/$ROS_VERSION_NAME/setup.bash
    cd $CATKIN_ROOT/catkin_ws
    echo "The scripts will now proceed to install any dependencies such as geographic-lib, pip3 and python3 as well as compiling the stack with catkin build\n"
    catkin build
    echo "source ${CATKIN_ROOT}/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

    # Restore the previous directory
    cd $current_directory
  }

# Function to add usefull alias created by Joao Quintas
function add_usefull_alias() {
  # Restore the function error flag
  # No checks are done in this function
  FUNCTION_ERROR="FALSE"

  printf "\n\n\n"
  echo "---------------------------------------------------------"
  echo "| Adding medusa permanent alias to ~/.bashrc            |"
  echo "---------------------------------------------------------"

    # Printing the alias being added on the screen
    echo "source ${ALIAS_LOCATION}"     

    # Add the alias
    echo "source ${ALIAS_LOCATION}" >> ~/.bashrc
    source ~/.bashrc
  }

#Function to install some recommended developer tools (EXTRAS - NOT NECESSARY)
function install_developer_tools() {
  # Restore the function error flag   
  # No checks are done in this function
  FUNCTION_ERROR="FALSE"

  printf "\n\n\n"
  echo "---------------------------------------------------------"
  echo "| Installing recommended tools                          |"
  echo "---------------------------------------------------------"

  printf "\nYou are about to install tools that are not really necessary. Namely:\n"
  for i in ${EXTRA_DEVELOPER_TOOLS[@]}
  do
    echo "${i}"
  done
  echo ""
  printf "Some developers like to have them in their PCs...\n"
  printf "Press [Y] if you want to install them. Press [N] if you want to skip them.\n"
  read PROCEED_VAR
  if [ "$PROCEED_VAR" != "${PROCEED_VAR#[Yy]}" ] ; then
    echo ""
    echo "Installing tools"
    echo ""
    # Installing extra developer tools that are not strictly necessary
    for i in ${EXTRA_DEVELOPER_TOOLS[@]}
    do
      echo ""
      echo "->Installing: ${i}"
      echo ""
      sudo apt-get install ${i} -y
    done
  else
    echo ""
    echo "Skipping this step..."
    echo ""
  fi

}

# Function to remove packages that are not needed anyore
function clean_apt_packages() {
  # Restore the function error flag   
  # No checks are done in this function
  FUNCTION_ERROR="FALSE"

  printf "\n\n\n"
  echo "---------------------------------------------------------"
  echo "| Cleaning apt-get                                      |"
  echo "---------------------------------------------------------"
  echo ""
  echo "sudo apt autoremove -y"
  sudo apt autoremove -y
}

function print_full_installation_complete() {
  # Restore the function error flag   
  # No checks are done in this function
  FUNCTION_ERROR="FALSE"

  printf "\n\n\n"
  echo "---------------------------------------------------------"
  echo "| Finished the installation                             |"
  echo "---------------------------------------------------------"
  echo ""
  echo "Please, do some scrolling up to see if nothing terribly failed 💩"
  echo ""
  echo "If everything went according to plan, you should now have a folder with all the medusa code already compiled:"
  echo "${CATKIN_ROOT}/catkin_ws/src"
  echo ""
  echo "Inside this folder you should find:"
  echo "${BITBUCKET_MEDUSA_REPO} - contains all the code written by DSOR team"
  echo "${EXTERNAL_RESPOS_FOLDER} - contains all the packages written by other teams"
  echo ""
  echo "The documentation for the Medusas can be found in:"
  echo "${CATKIN_ROOT}/${BITBUCKET_DOCUMENTATION_REPO}" 
  echo ""
  echo "In order to read it, run:"
  echo "cd ${CATKIN_ROOT}/${BITBUCKET_DOCUMENTATION_REPO}"
  echo "mkdocs serve"
  echo ""
  echo "The console to control the medusas can be found in:"
  echo "${CATKIN_ROOT}/${BITBUCKET_CONSOLE_REPO}" 
  echo ""
  echo "To check the new available alias you have, please open the file:"
  echo "$CATKIN_ROOT/catkin_ws/src/$BITBUCKET_MEDUSA_REPO/medusa_scripts/system_configurations/easy_alias/$ALIAS_PKG"
  echo ""
  echo "To test the installion run the following:"
  echo "Run roscd medusa"
  echo "This should change directory to medusa_vx"
  echo "Hope you enjoy the ride! 🍺"
  echo "The medusas team (by DSOR-ISR Lisbon)"
}


#--------------------------------------------------------------
#   ----------------------- MAIN ---------------------------
#--------------------------------------------------------------

# ----------------------------------------
# -> Check if there is internet connection
# ----------------------------------------
test_internet_connection
if [ $FUNCTION_ERROR == "TRUE" ] ; then
  echo "" 
  echo "ERROR: Without solid internet connection cannot proceed"
  echo "EXITING IN ${ABORT_SLEEP_TIME} seconds..."
  echo "" 
  sleep $ABORT_SLEEP_TIME
  exit
  FUNCTION_ERROR="FALSE" # Restore the error flag
fi

# ----------------------------------------
# -> Appending ${CATKIN_ROOT} variable to ~/.bashrc
# ----------------------------------------
prepare_bash_rc_to_receive_data
if [ $FUNCTION_ERROR == "TRUE" ] ; then
  echo "" 
  echo "ERROR: Could not append $CATKIN_ROOT variable to ${~}/.bashrc"
  echo "EXITING IN ${ABORT_SLEEP_TIME} seconds..."
  echo "" 
  sleep $ABORT_SLEEP_TIME
  exit
  FUNCTION_ERROR="FALSE" # Restore the error flag
fi

# ----------------------------------------
# -> Check which ubuntu version is running
# ----------------------------------------
check_ubuntu_version
if [ $FUNCTION_ERROR == "TRUE" ] ; then
  echo "" 
  echo "ERROR: Could not check ubuntu version - cannot proceed"
  echo "EXITING IN ${ABORT_SLEEP_TIME} seconds..."
  echo "" 
  sleep $ABORT_SLEEP_TIME
  exit
  FUNCTION_ERROR="FALSE" # Restore the error flag
fi

# ---------------------------
# -> Check for system updates
# ---------------------------
check_for_system_update
if [ $FUNCTION_ERROR == "TRUE" ] ; then
  echo ""
  echo "ERROR: Could not update system using sudo apt-get update/upgrade"
  echo "Cannot proceed with the installation."
  echo "EXITING IN ${ABORT_SLEEP_TIME} seconds..."
  echo ""
  sleep $ABORT_SLEEP_TIME
  exit
  FUNCTION_ERROR="FALSE" # Restore the error flag
fi

# -----------------------------------------------------------------
# -> Install basic programs that are always needed by any developer
# -----------------------------------------------------------------
install_basic_programs
if [ $FUNCTION_ERROR == "TRUE" ] ; then
  echo ""
  echo "ERROR: Could not install basic programs such as: curl, unzip, etc..."
  echo "Cannot proceed with the installation"
  echo "EXITING IN ${ABORT_SLEEP_TIME} seconds..."
  echo ""
  sleep $ABORT_SLEEP_TIME
  exit
  FUNCTION_ERROR="FALSE" # Restore the error flag
fi

# ------------------------------------------------
# -> Create the folder to install all the packages
# ------------------------------------------------
create_root_folder 
if [ $FUNCTION_ERROR == "TRUE" ] ; then
  echo "" 
  echo "ERROR: Could not create folder to install the packages"
  echo "       Without this folder creation, the installation will no proceed"
  echo "EXITING IN ${ABORT_SLEEP_TIME} seconds..."
  echo ""
  sleep $ABORT_SLEEP_TIME
  exit
  FUNCTION_ERROR="FALSE" # Restore the error flag
fi

# --------------------------------
# -> Receive bitbucket credentials
# --------------------------------
receive_BITBUCKET_credentials

# ------------------------------
# -> Clone the medusa repository
# ------------------------------
clone_medusa_repo $CATKIN_ROOT/catkin_ws/src
if [ $FUNCTION_ERROR == "TRUE" ] ; then
  echo "ERROR: Could not clone medusa_repository"
  echo "Are you sure the password and username were correct?"
  echo "Do you have internet connection?"

    # Try getting the credentials and clone until it works
    while [ $FUNCTION_ERROR == "TRUE" ] ; do

        # Receive bitbucket credentials
        receive_BITBUCKET_credentials

        # Clone the medusa repository
        clone_medusa_repo $CATKIN_ROOT/catkin_ws/src
      done
      FUNCTION_ERROR="FALSE" # Restore the error flag
fi

# ---------------------
# -> Ignore packages such as drivers
#    in development pc
# ---------------------
ignore_packages
if [ $FUNCTION_ERROR == "TRUE" ] ; then
  echo ""
  echo "ERROR: Could not touch CATKIN_IGNORE in package drivers "
  echo "       Compilation might fail latter on. This is not "
  echo "       a critical mistake."
  echo "Installation will proceed in ${ABORT_SLEEP_TIME} seconds"
  echo ""
  sleep $ABORT_SLEEP_TIME
  FUNCTION_ERROR="FALSE" # Restore the error flag
fi

# -------------------------------
# -> Clone the console repository
# -------------------------------
printf "\n Do you want to clone yebisu console (y/n)? Press y if you want a console to plan missions directly on a web browser. The console is big, it may take a few minutes.\n "
read READ_VAR
if [[ "$READ_VAR" != "${READ_VAR#[Yy]}" ]] ; then
  clone_medusa_console $CATKIN_ROOT/
  if [ $FUNCTION_ERROR == "TRUE" ] ; then
    echo ""
    echo "ERROR: Could not clone medusa console. DO IT MANUALLY:"
    echo "git clone https://${BITBUCKET_USERNAME}@bitbucket.org/dsor_isr/${BITBUCKET_CONSOLE_REPO}.git"
    echo "Installation will proceed in ${ABORT_SLEEP_TIME} seconds"
    echo ""
    sleep $ABORT_SLEEP_TIME
    FUNCTION_ERROR="FALSE" # Restore the error flag
  fi
fi

# --------------------------------------------
# -> Clone the medusa documentation repository
# --------------------------------------------
printf "\n Do you want to clone medusa documention (y/n)?"
read READ_VAR
if [[ "$READ_VAR" != "${READ_VAR#[Yy]}" ]] ; then
  clone_medusa_documentation $CATKIN_ROOT/
  if [ $FUNCTION_ERROR == "TRUE" ] ; then
    echo ""
    echo "ERROR: Could not clone medusa documentation repository. DO IT MANUALLY:"
    echo "git clone https://${BITBUCKET_USERNAME}@bitbucket.org/dsor_isr/${BITBUCKET_DOCUMENTATION_REPO}.git"
    echo "Installation will proceed in ${ABORT_SLEEP_TIME} seconds"
    echo ""
    sleep $ABORT_SLEEP_TIME
    FUNCTION_ERROR="FALSE" # Restore the error flag
  fi
fi
# ----------------------------------------------
# -> Clone the additional necessary repositories
# ----------------------------------------------
clone_additional_repos $EXTERNAL_REPOS_LOCATION
if [ $FUNCTION_ERROR == "TRUE" ] ; then
  echo ""
  echo "ERROR: Could not clone medusa packages not coded by DSOR"
  echo "       This is an error that does not allow to proceed installation"
  echo "Exiting in ${ABORT_SLEEP_TIME} seconds..."
  sleep $ABORT_SLEEP_TIME
  exit
fi
FUNCTION_ERROR="FALSE" # Restore the error flag

# ------------------------------------------------------------
# -> Give read, write and execute permission to
#    everything inside the main folder containing the packages
# ------------------------------------------------------------
#give_read_write_execute_permisions $CATKIN_ROOT/catkin_ws
#FUNCTION_ERROR="FALSE" # Restore the error flag

# ----------------------------------------------
# -> Version dependend install part
# ----------------------------------------------

# -> Ubuntu 20.04LTS
if [ $ROS_VERSION_NAME == noetic ] ; then

    # ---------------------
    # -> Install ROS NOETIC
    # ---------------------
    install_ros_noetic
    if [ $FUNCTION_ERROR == "TRUE" ] ; then
      echo ""
      echo "ERROR: Could not install ROS NOETIC"
      echo "       This is an error that does not allow to proceed installation"
      echo "Exiting in ${ABORT_SLEEP_TIME} seconds..."
      sleep $ABORT_SLEEP_TIME
      exit
    fi
    FUNCTION_ERROR="FALSE" # Restore the error flag

    # ---------------------
    # -> Configure python 
    #    for backwards compatiblity 
    # ---------------------
    python_installation_20.04LTS
    if [ $FUNCTION_ERROR == "TRUE" ] ; then
      echo ""
      echo "ERROR: Could not setup python properly"
      echo "       This is an error that does not allow to proceed installation"
      echo "Exiting in ${ABORT_SLEEP_TIME} seconds..."
      sleep $ABORT_SLEEP_TIME
      exit
    fi
    FUNCTION_ERROR="FALSE" # Restore the error flag

# ->Ubuntu 18.04LTS
elif [ $ROS_VERSION_NAME == melodic ] ; then

    # ---------------------
    # -> Install ROS MELODIC
    # ---------------------
    install_ros_melodic
    if [ $FUNCTION_ERROR == "TRUE" ] ; then
      echo ""
      echo "ERROR: Could not install ROS MELODIC"
      echo "       This is an error that does not allow to proceed installation"
      echo "Exiting in ${ABORT_SLEEP_TIME} seconds..."
      sleep $ABORT_SLEEP_TIME
      exit
    fi
    FUNCTION_ERROR="FALSE" # Restore the error flag

    # ---------------------
    # -> Configure python 2 and 3 
    #    to use with newer code 
    # ---------------------
    python_installation_18.04LTS
    if [ $FUNCTION_ERROR == "TRUE" ] ; then
      echo ""
      echo "ERROR: Could not setup python properly"
      echo "       This is an error that does not allow to proceed installation"
      echo "Exiting in ${ABORT_SLEEP_TIME} seconds..."
      sleep $ABORT_SLEEP_TIME
      exit
    fi
    FUNCTION_ERROR="FALSE" # Restore the error flag
fi

# ---------------------
# -> Run costume bash scripts from medusa repository
# ---------------------
execute_custom_scripts
FUNCTION_ERROR="FALSE" # Restore the error flag

# ---------------------
# -> Install extra libraries
# ---------------------
install_extra_libraries
FUNCTION_ERROR="FALSE" # Restore the error flag

# ---------------------
# -> Install extra ROS PACKAGES
# ---------------------
install_extra_ros_packages
FUNCTION_ERROR="FALSE" # Restore the error flag

# ---------------------
# -> Install ROSDEP PACKAGES
# ---------------------
#install_rosdep_packages
#FUNCTION_ERROR="FALSE" # Restore the error flag

# ---------------------
# -> Install external libraries
# (libraries that need to be installed manually)
# ---------------------
external_libraries_instalation
if [ $FUNCTION_ERROR == "TRUE" ] ; then
  echo ""
  echo "ERROR: Could not setup external cpp libraries properly"
  echo "       This is an error that does not allow to proceed installation"
  echo "Exiting in ${ABORT_SLEEP_TIME} seconds..."
  echo ""
  sleep $ABORT_SLEEP_TIME
  exit
fi
FUNCTION_ERROR="FALSE" # Restore the error flag


# ---------------------
# -> Install catkin python tools
# ------------------------
# If the Ubuntu version is 20.04 install catkin tools manually
if [ "${UBUNTU_VERSION}" == "\"20.04\"" ] ; then
  sudo pip3 install git+https://github.com/catkin/catkin_tools.git
else
  sudo apt-get install python-catkin-tools
fi

# ---------------------
# -> Add usefull alias
# ---------------------
add_usefull_alias
if [ $FUNCTION_ERROR == "TRUE" ] ; then
  echo ""
  echo "ERROR: Could not setup usefull alias"
  echo "       This is not a critical error. They can be found"
  echo "       Continuing in ${ABORT_SLEEP_TIME} seconds..."
  sleep $ABORT_SLEEP_TIME
fi
FUNCTION_ERROR="FALSE" # Restore the error flag

# -------------------------------------------------------
# -> Install extra tools that are NOT strictly necessary
# -------------------------------------------------------
install_developer_tools
if [ $FUNCTION_ERROR == "TRUE" ] ; then
  echo ""
  echo "ERROR: Could not install extra developer tools"
  echo "       This is not a critical error."
  echo "       Continuing in ${ABORT_SLEEP_TIME} seconds..."
  sleep $ABORT_SLEEP_TIME
fi
FUNCTION_ERROR="FALSE" # Restore the error flag

# -------------------------
# -> Clear the installation
# -------------------------
#clean_apt_packages
#FUNCTION_ERROR="FALSE" # Restore the error flag

cd $CATKIN_ROOT
mkdir ROSData
mkdir paths_from_console


# ---------------------------------------------------------
# -> Clone uuvsimulator and medusa_gazebo for simulations
# ----------------------------------------------------------

echo ""
printf "If you want amazing 3D simulations(UUVSimulator), please check if you have more than 20Gb  ...\n"
printf "Press [Y] if you want to install them. Press [N] if you want to skip them.\n"
read PROCEED_VAR
if [ "$PROCEED_VAR" != "${PROCEED_VAR#[Yy]}" ] ; then
  echo ""
  echo "Clonning neceessary medusa_gazebo"
  echo ""
  cd $CATKIN_ROOT/catkin_ws/src/
  git clone https://$BITBUCKET_USERNAME:$BITBUCKET_PASSWORD@bitbucket.org/dsor_isr/medusa_gazebo.git

  echo ""
  echo "Clonning UUVSimulator"
  echo "" 
  if [ "${UBUNTU_VERSION}" == "\"20.04\"" ] ; then
    git clone https://github.com/arturmiller/uuv_simulator.git
  else
    sudo apt install ros-melodic-uuv-simulator
  fi
else
  echo ""
  echo "Skipping this step..."
  echo ""
fi

cd $CATKIN_ROOT/catkin_ws

# ---------------------
# -> Compile medusa code
# ---------------------
compile_medusa_code
FUNCTION_ERROR="FALSE" # Restore the error flag


# -------------------------
# -> PRINT INSTALLATION COMPLETE
# -------------------------
print_full_installation_complete
FUNCTION_ERROR="FALSE" # Restore the error flag

#-----------------------------------
# -> source bashrc for activating the stack
#-----------------------------------
source ~/.bashrc 
