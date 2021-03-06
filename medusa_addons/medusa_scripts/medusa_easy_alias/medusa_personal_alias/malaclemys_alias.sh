# Malaclemys personal alias

# MEDUSA SSH connections

alias myellow='ssh medusa@myellow'          # Myellow
alias mredLab='ssh medusa@mredLab'          # Mred lab
alias mblackLab='ssh isr@mblackLab'         # Mblack lab
alias delfim='ssh dsor@delfim'              # Delfim
alias muned='ssh medusa@muned'              # Muned
alias mblack='ssh medusa@mblack'            # Mblack
alias mred='ssh medusa@mred'                # Mred
alias hrov_pts_otg='ssh oceantech@hrov_otg' # Hrov port side
alias mvector='ssh medusa@mvector'	        # Mvector

# *.* MEDUSA Safety Features
alias sf_myellow='$(find ${ROS_WORKSPACE}/src/ -type d -iname safety_feature)/src/Client/SafetyFeatureCLT myellow'
alias sf_mvector='$(find ${ROS_WORKSPACE}/src/ -type d -iname safety_feature)/src/Client/SafetyFeatureCLT mvector'
alias sf_mred='$(find ${ROS_WORKSPACE}/src/ -type d -iname safety_feature)/src/Client/SafetyFeatureCLT mred'
alias sf_mblack='$(find ${ROS_WORKSPACE}/src/ -type d -iname safety_feature)/src/Client/SafetyFeatureCLT mblack'
alias sf_delfim='$(find ${ROS_WORKSPACE}/src/ -type d -iname safety_feature)/src/Client/SafetyFeatureCLT delfim'
alias sf_muned='$(find ${ROS_WORKSPACE}/src/ -type d -iname safety_feature)/src/Client/SafetyFeatureCLT muned'
alias sf_msim='$(find ${ROS_WORKSPACE}/src/ -type d -iname safety_feature)/src/Client/SafetyFeatureCLT malaclemys'

# VPN DSOR
alias vpn_dsor_on='sudo systemctl start wg-quick@vpn-dsor'
alias vpn_dsor_off='sudo systemctl stop wg-quick@vpn-dsor'