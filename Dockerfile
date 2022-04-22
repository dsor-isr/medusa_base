ARG ros_version=noetic
FROM osrf/ros:$ros_version-desktop-full
LABEL maintainer="Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>"

# -------------------------------
# Create the container user
# -------------------------------
ARG user=medusa
ARG passwd=medusa
ARG uid=1000
ARG gid=1000
ENV USER=$user
ENV PASSWD=$passwd
ENV UID=$uid
ENV GID=$gid
RUN groupadd $USER && \
    useradd --create-home --no-log-init -g $USER $USER && \
    usermod -aG sudo $USER && \
    echo "$PASSWD:$PASSWD" | chpasswd && \
    chsh -s /bin/bash $USER && \
    usermod --uid $UID $USER && \
    groupmod --gid $GID $USER

ENV HOME=/home/$USER
WORKDIR $HOME

# -------------------------------
# Install basic tools
# -------------------------------
RUN apt-get update && \
     apt-get install -y --no-install-recommends \
     wget \
     curl \
     git \
     tmux \
     && rm -rf /var/lib/apt/lists/*

# -------------------------------
# Install the Medusa requirements
# -------------------------------
RUN wget https://raw.githubusercontent.com/dsor-isr/medusa_base/main/install_requirements.sh 
RUN /bin/bash install_requirements.sh
RUN rm install_requirements.sh

# ----------------------------------------------------
# Get the script to use as entrypoint to the container
# ----------------------------------------------------
RUN wget https://raw.githubusercontent.com/dsor-isr/medusa_base/main/medusa_docker/scripts/basic_entrypoint.sh

# ----------------------------------
# Change the User to the medusa user
# ----------------------------------
USER $USER

# ----------------------------------
# Create a catkin workspace
# ----------------------------------
ENV CATKIN_ROOT=${HOME}
ENV ROS_WORKSPACE=${CATKIN_ROOT}/catkin_ws
RUN mkdir $ROS_WORKSPACE

# Adding elements to the .bashrc file
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# ----------------------------------
# Expose container ports
# ----------------------------------
EXPOSE 7080
EXPOSE 8080
EXPOSE 9090
EXPOSE 11311

# -----------------------------------------------------------------
# Setup the image entry point to be anything specified as arguments
# -----------------------------------------------------------------
ENTRYPOINT ["/basic_entrypoint.sh"]