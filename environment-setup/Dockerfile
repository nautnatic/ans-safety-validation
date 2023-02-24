# use osrf/ros instead of ros because it has desktop environment
FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]

# configure user to avoid permission problems with host in volumes
# source: https://jtreminio.com/blog/running-docker-containers-as-current-host-user/
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
RUN apt-get -y update && \
    # install apt packages
    apt-get install -y \
    git \
    vim \
    liblua5.3-dev \
    # cleanup
    && apt-get clean && apt-get autoremove

USER $USER_NAME
WORKDIR $HOME

# prepare catkin workspace (src directory contains packages)
RUN mkdir -p $HOME/src/

# install ros dependencies
WORKDIR $HOME/src
COPY .rosinstall .
RUN rosws update

# build ros dependencies to cache them (j1 = only one thread -> less errors, but slower)
WORKDIR $HOME
RUN source "/opt/ros/noetic/setup.sh" \
    && catkin_make -j1

# build new packages and launch ros launch file
#CMD catkin_make \
#    && source $HOME/devel/setup.sh \
#    && roslaunch ${LAUNCH_PACKAGE} ${LAUNCH_FILE} ${ADDITIONAL_LAUNCH_PARAMS}

# add ros setup to .bashrc to do autonatically when opening new bash shell
RUN echo -e "source /opt/ros/noetic/setup.bash" >> $HOME/.bashrc
RUN echo -e "source $HOME/devel/setup.bash" >> $HOME/.bashrc
