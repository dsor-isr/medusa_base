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

# Position in console for support vessel
alias console_position='roslaunch console_pos pos.launch'

# VPN DSOR
alias vpn_dsor_on='sudo systemctl start wg-quick@vpn-dsor'
alias vpn_dsor_off='sudo systemctl stop wg-quick@vpn-dsor'