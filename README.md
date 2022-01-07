# Medusa Base
This repository holds the Medusa Base code stack for underwater marine vehicles of DSOR-ISR (Dynamical Systems for Ocean Robotics - Institute for System Robotics). It contains the base of the control and navigation stack found in the MEDUSA class of marine vehicles.

### Requirements
This code stack was developed with ROS1 in mind. In order to use, you are required to have:
- Ubuntu 20.04LTS (64-bit)
- ROS1 Noetic
- Python 3

### Installation
- Install the Python requirements:
```
pip3 install --user numpy pandas matplotlib scipy sklearn rospkg catkin_pkg future joystick-controller
```

- Install C++ (apt-get) requirements:
```
sudo apt-get install libgeographic-dev ros-noetic-geographic-msgs libxmlrcpp-dev librosconsole-dev libudev-dev libusb-1.0-0-dev ros-noetic-geodesy -y
```

- Manual install Geographiclib 1.50.1 (C++ library):
```
wget https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.50.1.tar.gz/download
tar xfpz download
cd GeographicLib-1.50.1 
mkdir BUILD
cd BUILD
cmake ..
sudo make
sudo make test
sudo make install
cd ..
cd ..
sudo rm -R download GeographicLib-1.50.1
```

- Manual install Eigen version 3.4.0 (C++ equivalent of numpy in python):
```
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar xfpz eigen-3.4.0.tar.gz
mkdir BUILD
cd BUILD
cmake ..
sudo make
sudo make install
cd ..
cd ..
sudo rm -R eigen-3.4.0 eigen-3.4.0.tar.gz
```

- Clone this repository and its submodules:
```
git clone --recursive git@github.com:dsor-isr/medusa_base.git
```

### Using Medusa Scripts and Alias
In order to make use of the scripts and alias developed to make the development of code easier, please add the following lines to your ~/.bashrc file
```
# Function to change between different catkin workspaces on the fly - this is not compulsory, but it is a nice addition 🤓

# Create a file to store the latest catkin workspace (if it does not exist) and put in the first line the default name, i.e. catkin_ws
if [ ! -f ~/.catkin_ws_config ]; then touch ~/.catkin_ws_config && echo catkin_ws > ~/.catkin_ws_config ;fi

# Check if the variable CATKIN_PACKAGE is set, if not set it with the workspace in the catkin_ws_config file
if [ -z ${CATKIN_PACKAGE+x} ]; then export CATKIN_PACKAGE=$(head -n 1 ~/.catkin_ws_config);fi

# Function to update the default catkin workspace variable and store the last setting in the file
set_catkin_ws_function() {
    #set CATKIN_PACKAGE according the an input parameter
    export CATKIN_PACKAGE=catkin_ws_$1
    echo CATKIN_PACKAGE = ${CATKIN_PACKAGE}
    
    # save into a hidden file the catkin workspace setting
    echo $CATKIN_PACKAGE > ~/.catkin_ws_config
    source ~/.bashrc
}

# This is required (to source the ROS and medusa files)
source /opt/ros/noetic/setup.bash
export CATKIN_ROOT=$(~/<path_to_workspace>)
export ROS_WORKSPACE=${CATKIN_ROOT}/${CATKIN_PACKAGE}
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
- Francisco Rego <ffcrego@gmail.com>
- David Cabecinhas <dcabecinhas@isr.tecnico.ulisboa.pt>

### Previous Contributors
- João Cruz
- Hung Tuan
- Shubham Garg
- Jorge Ribeiro
- Miguel Ribeiro
- Henrique Silva
- João Botelho
- Filipa Almeida

### Omnipresent
- Prof. António Pascoal
- Prof. Carlos Silvestre
- Prof. Rita Cunha
- Prof. Bruno Guerreiro
- Prof. Pedro Batista
- Luís Sebastião
- Manuel Rufino
- Pedro Gois
- Helena Santana

### License
MIT