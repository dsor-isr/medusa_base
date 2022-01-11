# Trials personal alias
# *.* MEDUSA SSH connections

alias myellow='ssh medusa@myellow'  # myellow
alias mredLab='ssh medusa@mredLab'  # mred lab
alias mblackLab='ssh isr@mblackLab' # mblack lab
alias delfim='ssh dsor@delfim'      # Delfim
alias muned='ssh medusa@muned'      # muned
alias mblack='ssh medusa@mblack'    # mblack
alias mred='ssh medusa@mred'        # mred
alias hrov_pts_otg='ssh oceantech@hrov_otg'

# *.* MEDUSA Safety Features
alias sf_myellow='$(find ${ROS_WORKSPACE}/src/ -type d -iname safety_feature)/src/Client/SafetyFeatureCLT myellow'
alias sf_mvector='$(find ${ROS_WORKSPACE}/src/ -type d -iname safety_feature)/src/Client/SafetyFeatureCLT mvector'
alias sf_mred='$(find ${ROS_WORKSPACE}/src/ -type d -iname safety_feature)/src/Client/SafetyFeatureCLT mred'
alias sf_mblack='$(find ${ROS_WORKSPACE}/src/ -type d -iname safety_feature)/src/Client/SafetyFeatureCLT mblack'
alias sf_delfim='$(find ${ROS_WORKSPACE}/src/ -type d -iname safety_feature)/src/Client/SafetyFeatureCLT delfim'
alias sf_muned='$(find ${ROS_WORKSPACE}/src/ -type d -iname safety_feature)/src/Client/SafetyFeatureCLT muned'
alias sf_msim='$(find ${ROS_WORKSPACE}/src/ -type d -iname safety_feature)/src/Client/SafetyFeatureCLT localhost'

