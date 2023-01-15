# use osrf/ros instead of ros because it has desktop environment
FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]

# configure user
# source: https://jtreminio.com/blog/running-docker-containers-as-current-host-user/
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USER_NAME=user
# environment variable for user home directory is required
ENV HOME=/home/$USER_NAME
RUN groupadd -g ${GROUP_ID} $USER_NAME &&\
    useradd -l -u ${USER_ID} -g $USER_NAME $USER_NAME &&\
    install -d -m 0755 -o $USER_NAME -g $USER_NAME $HOME

# install apt packages
USER root
RUN apt-get -y update \
    && apt-get install -y \
    apt-utils \
    software-properties-common \
    git \
    wget \
    ros-noetic-tf2 \
    ros-noetic-tf \
    ros-noetic-tf2-geometry-msgs \
    ffmpeg \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential \
    libsm6 \
    libxext6  \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get -y update \
    && apt-get -y install \
    libopencv-dev \
    liblua5.2-dev \
    screen \
    python3 \
    python3-dev \
    libpython3-dev \
    python3-catkin-pkg-modules \
    python3-rospkg-modules \
    python3-empy \
    python3-setuptools \
    ros-noetic-navigation \
    ros-noetic-teb-local-planner \
    ros-noetic-mpc-local-planner \
    libarmadillo-dev \
    ros-noetic-nlopt \
    python3 \
    python3-pip \
    tk \
    ros-noetic-turtlebot3-description \
    ros-noetic-turtlebot3-navigation \
    python-tk \
    python3-tk \
    tk-dev

USER $USER_NAME

# install python packages
RUN pip3 install poetry \
    && pip3 install --upgrade pip
# TODO: install dependencies with poetry / pipenv
# poetry shell && poetry install

# install PyEnv
WORKDIR $HOME
RUN git clone --depth=1 https://github.com/pyenv/pyenv.git $HOME/.pyenv
ENV PYENV_ROOT="$HOME/.pyenv"
ENV PATH="${PYENV_ROOT}/shims:${PYENV_ROOT}/bin:${PATH}"
RUN export PATH="$PYENV_ROOT/bin:$PATH" >> $HOME/.bashrc \
    eval "$(pyenv init -)" >> $HOME/.bashrc

# make src directory for packages in catkin workspace
RUN mkdir -p $HOME/src/

# add to .bashrc to source ros setup when executing bash directly
RUN echo -e "source /opt/ros/noetic/setup.bash" >> $HOME/.bashrc
# RUN echo -e "source $HOME/devel/setup.bash" >> $HOME/.bashrc

# install ros dependencies
WORKDIR $HOME/src
COPY .rosinstall ./
RUN rosws update

# build existing packages to cache them
WORKDIR $HOME
RUN source $HOME/.bashrc \
    && source /opt/ros/noetic/setup.sh \
    && catkin_make

# build new packages and launch ros launch file
WORKDIR $HOME
CMD catkin_make \
    && source $HOME/devel/setup.sh \
    && roslaunch ${LAUNCH_PACKAGE} ${LAUNCH_FILE} ${ADDITIONAL_LAUNCH_PARAMS}