# Medusa Base
This repository holds the Medusa Base code stack for underwater marine vehicles of DSOR-ISR (Dynamical Systems for Ocean Robotics - Institute for System Robotics). It contains the base of the control and navigation stack found in the MEDUSA class of marine vehicles.

[![Build Status](https://ci.dsor.isr.tecnico.ulisboa.pt/buildStatus/icon?job=GitHub+DSOR%2Fmedusa_base%2Fmain)](https://ci.dsor.isr.tecnico.ulisboa.pt/job/GitHub%20DSOR/job/medusa_base/job/main/)
![GitHub last commit (branch)](https://img.shields.io/github/last-commit/dsor-isr/medusa_base/main)
![GitHub contributors](https://img.shields.io/github/contributors/dsor-isr/medusa_base)
[![GitHub issues](https://img.shields.io/github/issues/dsor-isr/medusa_base)](https://github.com/dsor-isr/medusa_base/issues)
[![GitHub forks](https://img.shields.io/github/forks/dsor-isr/medusa_base)](https://github.com/dsor-isr/medusa_base/network)
[![GitHub stars](https://img.shields.io/github/stars/dsor-isr/medusa_base)](https://github.com/dsor-isr/medusa_base/stargazers)
[![License](https://img.shields.io/github/license/dsor-isr/medusa_base?color=blue)](https://github.com/dsor-isr/medusa_base/blob/main/LICENSE)

### Requirements
This code stack was developed with ROS1 in mind. In order to use, you are required to have:
- Ubuntu 20.04LTS (64-bit)
- ROS1 Noetic
- Python 3

### Installation
- Run the installation script (note: you will require administrator priviledges)
```
wget https://github.com/dsor-isr/medusa_base/blob/main/install_requirements.sh
./install_requirements.sh
rm install_requirements.sh
```

- Clone this repository and its submodules to the catkin workspace:
```
git clone --recursive git@github.com:dsor-isr/medusa_base.git
```

### Using Medusa Scripts and Alias
In order to make use of the scripts and alias developed to make the development of code easier, please add the following lines to your ~/.bashrc file.
NOTE: replace '/<path_to_workspace>' with the folder where you put you catkin_ws inside. If you put in your home folder, then this variable should be left empty!
```
# Function to change between different catkin workspaces on the fly - this is not compulsory, but it is a nice addition 🤓

# Create a file to store the latest catkin workspace (if it does not exist) and put in the first line the default name, i.e. catkin_ws
if [ ! -f ~/.catkin_ws_config ]; then touch ~/.catkin_ws_config && echo catkin_ws > ~/.catkin_ws_config ;fi

# Set the variable CATKIN_PACKAGE with the workspace in the catkin_ws_config file
export CATKIN_PACKAGE=$(head -n 1 ~/.catkin_ws_config)

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
export CATKIN_ROOT=${HOME}/<path_to_workspace>
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

### Add submodules

To add submodules to this repo (or another within the DSOR infrastructure) the standard practice is the following:

```
git submodule add git@github.com:dsor-isr/<repository name>.git
```

Now, go to ```.gitmodules``` file inside the repo, via ```vim .gitmodules``` or another editor of choice, and change the following line

```
url = ...
```
to
```
url = ..\<repository name>.git
```

### Citation
If you use Medusa Base in a scientific publication, please cite:
```
TODO
```

### Documentation
https://dsor-isr.github.io/medusa_base/

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
MedusaBase is open-sourced under the MIT license. See the [LICENSE](LICENSE) file for details.
