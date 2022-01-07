# Medusa Base
This repository holds the Medusa Base code stack for underwater marine vehicles of DSOR-ISR (Dynamical Systems for Ocean Robotics - Institute for System Robotics). It contains the base of the control and navigation stack found in the MEDUSA class of marine vehicles.

### Requirements
This code stack was developed with ROS1 in mind. In order to use, you are required to have:
- Ubuntu 20.04LTS (64-bit)
- ROS1 Noetic
- Python 3

### Installation
Install the Python requirements:
```
TODO
```
Install C++ requirements:
```
TODO
```

- Clone this repository and its submodules:
```
git clone --recursive git@github.com:dsor-isr/medusa_base.git
```

### Using Medusa Scripts and Alias
In order to make use of the scripts and alias developed to make the development of code easier, please add the following lines to your ~/.bashrc file
```
source /opt/ros/${ROS_DISTRO}/setup.bash
export CATKIN_ROOT=$(~/<path_to_workspace>)
export ROS_WORKSPACE=${CATKIN_ROOT}/<catkin_ws>
export MEDUSA_SCRIPTS=$(find ${ROS_WORKSPACE}/src/ -type d -iname medusa_scripts | head -n 1)
source ${MEDUSA_SCRIPTS}/medusa_easy_alias/medusa_permanent_alias/alias.sh
```

### Compile the code
- Compile the code
```
cd ~/<path_to_workspace>/<catkin_ws>/
catkin build
```

### Citation
If you use medusa_base in a scientific publication, we would appreciate citations: TODO

### Help and Support
Documentation TODO

### Active Developers
- João Quintas <jquintas@gmail.com>
- Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
- David Souto <david.souto@tecnico.ulisboa.pt>
- André Potes <andre.potes@tecnico.ulisboa.pt>
- Francisco Rego
- David Cabecinhas

### Previous Contributors
- João Cruz
- Jorge Ribeiro
- Miguel Ribeiro
- Shubham Garg

### License
MIT