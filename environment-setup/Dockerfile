# use osrf/ros instead of ros because it has desktop environment
FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]

# configure user to avoid permission problems with host in volumes
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USER_NAME=user
ARG PASSWORD=user
ENV HOME=/home/$USER_NAME
RUN groupadd -g ${GROUP_ID} $USER_NAME && \
    useradd -l -u ${USER_ID} -g $USER_NAME -p $(echo $PASSWORD | base64) $USER_NAME && \
    install -d -m 0755 -o $USER_NAME -g $USER_NAME $HOME
ENV PATH="/home/$USER_NAME/.local/bin:$PATH"

# enable passwordless sudo
USER root
RUN echo "${USER_NAME}            ALL = (ALL) NOPASSWD: ALL" >> /etc/sudoers

# add apt repos and install apt packages
USER root
RUN apt-get -y update

# install basic packages
RUN apt-get install -y \
    git wget vim build-essential cmake \
    tk tk-dev
# install libs
RUN apt-get install -y \
    liblua5.3-dev libsm6 libxext6 libopencv-dev liblua5.2-dev screen libarmadillo-dev libopenmpi-dev zlib1g-dev
# install ros packages
RUN apt-get install -y \
    ros-noetic-tf2 ros-noetic-tf ros-noetic-tf2-geometry-msgs \
    ros-noetic-navigation ros-noetic-teb-local-planner ros-noetic-mpc-local-planner \
    ros-noetic-nlopt ros-noetic-turtlebot3-description ros-noetic-turtlebot3-navigation
# install Python dependent packages
RUN apt-get install -y \
    python3 python3-pip python-is-python3 \
    python3-rospy python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
    python3-dev libpython3-dev python3-catkin-pkg-modules python3-rospkg-modules \
    python3-empy python3-setuptools python-tk python3-tk

# apt cleanup
RUN apt-get clean && apt-get autoremove

USER $USER_NAME
WORKDIR $HOME

# install python dependencies globally with pip
RUN pip install --upgrade --no-cache-dir \
    numpy stable-baselines3[extra] flake8 autopep8

# prepare catkin workspace (src directory contains packages)
RUN mkdir -p $HOME/src/

# prepare logging directory
RUN mkdir -p $HOME/.ros && chown -R $USER_NAME:$USER_NAME $HOME/.ros

# install ros dependencies
WORKDIR $HOME/src
COPY .rosinstall .
RUN rosws update

# build ros dependencies to cache them (j1 = only one thread -> less errors, but slower)
WORKDIR $HOME
RUN source "/opt/ros/noetic/setup.sh" \
    && catkin_make -j1

# add ros setup to .bashrc to do autonatically when opening new bash shell
RUN echo -e "source /opt/ros/noetic/setup.bash" >> $HOME/.bashrc
RUN echo -e "source $HOME/devel/setup.bash" >> $HOME/.bashrc