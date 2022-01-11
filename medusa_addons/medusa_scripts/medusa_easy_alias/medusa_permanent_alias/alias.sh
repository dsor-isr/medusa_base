############################################################
# -- medusa alias: used to save time in typing commands -- #
############################################################

# source devel setup.bash
source ${ROS_WORKSPACE}/devel/setup.bash

# enable git completition
source ${MEDUSA_SCRIPTS}/medusa_scripts_for_bash/git-completion.bash

# display git branch on console prompt
source ${MEDUSA_SCRIPTS}/medusa_scripts_for_bash/display_git_branch_in_prompt.sh

# git pull repo and get the latest updates in the corresponding submodules
source ${MEDUSA_SCRIPTS}/medusa_scripts_for_bash/git_special_commands.sh

# roscat cat a file by pkg_name and filename
source ${MEDUSA_SCRIPTS}/medusa_scripts_for_bash/roscat.sh

# source autocomplete extensions
source ${MEDUSA_SCRIPTS}/medusa_scripts_for_bash/autcomplete.sh

# source transfer.sh
source ${MEDUSA_SCRIPTS}/medusa_scripts_for_bash/transfer.sh

#####################
# -- ROS related -- #
#####################

# kill all nodes
alias kill_all_ros_nodes='sudo pkill -f ros' 

# view state machine
alias smach_viewer='rosrun smach_viewer smach_viewer.py'

# open rviz visualization
alias rviz='rosrun rviz rviz'

# view current tf frames
alias tf_view_frames='cd /var/tmp && rosrun tf2_tools view_frames.py && evince frames.pdf &'

# clean package from workspace
alias clean_pkg_from_ws='source ${MEDUSA_SCRIPTS}/medusa_scripts_for_bash/clean_pkg_from_ws.sh'

# clean logs from ros, handy when over 1gb message appears
alias clean_ros_logs='rosclean purge -y'


#####################
# -- compilation -- #
#####################

# build all the code
alias medusa_cb='roscd; catkin build; cd $OLDPWD' # build the entire catkinworkspace (you need to be somewhere inside your catkin workspace)

# build only one package
alias medusa_cbt='catkin build --this' # build one pkg (you need to be somewhere inside your ros pkg)

# compile all the medusa stack
alias medusa_cm='roscd; catkin_make; cd $OLDPWD'

alias medusa_cb_vim='roscd; bash ${MEDUSA_SCRIPTS}/medusa_scripts_for_bash/medusa_build_vim.bash'

############################
# -- common handy tools -- #
############################

# when typing fast sometimes ls gets typed as sl
alias sl='ls'

# replace cat with python-pygments to cat with colors
alias cap='pygmentize -g'

# going back one directory and showing files convenient alias
alias ..='cd .. && ls'

# toggle terminal from restored to maximized
alias m='wmctrl -r :ACTIVE: -b toggle,maximized_vert,maximized_horz'

# shutdown pc
alias poweroff='sudo shutdown -h now'

# different way to shutdown pc
alias pcdown='sudo shutdown -P now'

# restart the pc
alias pcrestart='sudo shutdown -r now'

# That anoying moments you forgot sudo
alias please_fiic='sudo $(history -p !!)' # run last command as sudo

# source your bashrc
alias S='source ${HOME}/.bashrc'

# give some sanity to your shell
alias sane_your_shell='stty sane; clear;'

######################
# -- ntpdate sync -- #
######################

# medusa ntpdate with console pc
alias medusa_ntpdate='sudo ntpdate -bu ntpServer'

# ntpdate with the world
alias world_ntpdate='sudo service ntp stop && sudo ntpdate -s time.nist.gov && sudo service ntp start'

#####################
# -- Text editor -- #
#####################

# remove automatically spaces at the end of files, needs the file as argument at the end, i.e. remove_spaces my_file.txt
alias remove_endline_spaces="sed -i 's/\s*$//'"

# to clean temp files *.~ recursively
alias clean_temp_files='find . -name "*~" -type f -exec /bin/rm -fv -- {} +'

######################
# -- Change gains -- #
######################
alias medusa_change_inners_gains='bash ${MEDUSA_SCRIPTS}/medusa_scripts_for_bash/change_inner_forces_gains.bash'
alias medusa_change_pfollowing_gains='bash ${MEDUSA_SCRIPTS}/medusa_scripts_for_bash/change_pfollowing_gains.bash'

##########################
# -- ROS MEDUSA PATHS -- #
##########################
export ROS_BAG_FOLDER="${CATKIN_ROOT}"
export ROS_PACKAGE_PATH="${ROS_WORKSPACE}/src:/opt/ros/${ROS_DISTRO}/share"

if [ -d "$(find ${ROS_WORKSPACE}/src/ -type d -iname medusa_base | head -n 1)" ]; then
	export ROS_LOCATIONS="medusa=$(find ${ROS_WORKSPACE}/src/ -type d -iname medusa_base | head -n 1)"
fi
if [ -d "${ROS_WORKSPACE}/src/medusa_real" ]; then
	export ROS_LOCATIONS="medusa_real=${ROS_WORKSPACE}/src/medusa_real:${ROS_LOCATIONS}"
fi
if [ -d "${ROS_WORKSPACE}/src/medusa_simulation" ]; then
	export ROS_LOCATIONS="medusa_simulation=${ROS_WORKSPACE}/src/medusa_simulation:${ROS_LOCATIONS}"
fi
export ROSCONSOLE_FORMAT='[${severity}] [${time}]: ${node}: ${message}'
export EDITOR=vim

###################################
# -- new medusa package python -- #
###################################
alias medusa_pkg_py="source ${MEDUSA_SCRIPTS}/medusa_new_packages_scripts/medusa_create_ros_pkg_py.sh"

#########################
# -- new package c++ -- #
#########################
alias medusa_pkg_cpp="source ${MEDUSA_SCRIPTS}/medusa_new_packages_scripts/medusa_create_ros_pkg_cpp.sh"

##########################
# -- new package meta -- #
##########################
alias medusa_pkg_meta="source ${MEDUSA_SCRIPTS}/medusa_new_packages_scripts/medusa_create_ros_pkg_meta.sh"

######################################################################
# -- Personal configs if hostname file exists (vehicles included) -- #
######################################################################
if [ -f "${MEDUSA_SCRIPTS}/medusa_easy_alias/medusa_personal_alias/${HOSTNAME}_alias.sh" ]; then
	source ${MEDUSA_SCRIPTS}/medusa_easy_alias/medusa_personal_alias/${HOSTNAME}_alias.sh
fi