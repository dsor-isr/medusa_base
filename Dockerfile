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
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
    wget \
    curl \
    git \
    python3-pip \
    python3-catkin-tools \
    libgeographic-dev \
    ros-noetic-geographic-msgs \
    librosconsole-dev \
    ros-noetic-geodesy \
    doxygen \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------
# Install the Medusa requirements
# -------------------------------
RUN wget https://raw.githubusercontent.com/dsor-isr/medusa_base/main/install_requirements.sh 
RUN /bin/bash install_requirements.sh
RUN rm install_requirements.sh

# -------------------------------------------------
# Install the documentation generation requrirement
# -------------------------------------------------
RUN pip3 install --no-cache-dir \
    mkdocs \
    mkdocs-material \
    mkdocs-bibtex \
    mkdocs-git-revision-date-plugin \
    mkdocs-git-revision-date-localized-plugin \
    mkdocs-monorepo-plugin \
    mkdocs-macros-plugin \
    mkdocs-include-markdown-plugin \
    mkdocs-git-revision-date-localized-plugin \
    mike \
    ruamel.yaml

# Used for generating the markdown documentation from the xml generated by doxygen
# Note (for this we need nodejs, in particular at least version 12)
#RUN curl -sL https://deb.nodesource.com/setup_12.x  | bash -
#RUN apt-get install -y --no-install-recommends nodejs && rm -rf /var/lib/apt/lists/*
#RUN git clone https://github.com/sourcey/moxygen.git && cd moxygen && npm install -g .
#RUN npm install -g moxygen
RUN mkdir doxybook2 && \
    cd doxybook2 && \
    wget https://github.com/matusnovak/doxybook2/releases/download/v1.4.0/doxybook2-linux-amd64-v1.4.0.zip && \
    unzip doxybook2-linux-amd64-v1.4.0.zip && \
    cd bin && mv doxybook2 /usr/bin/ && \
    cd ${HOME} && rm -f -r doxybook2

# ----------------------------------
# Change the User to the medusa user
# ----------------------------------
USER $USER

# ----------------------------------------------------
# Get the script to use as entrypoint to the container
# ----------------------------------------------------
RUN wget https://raw.githubusercontent.com/dsor-isr/medusa_base/main/medusa_docker/scripts/basic_entrypoint.sh
RUN chmod +x /home/medusa/basic_entrypoint.sh

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
ENTRYPOINT ["/home/medusa/basic_entrypoint.sh"]
CMD ["bash"]