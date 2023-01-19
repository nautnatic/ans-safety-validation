# use osrf/ros instead of ros because it has desktop environment
# use the filesystem of the host
FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]

# configure user
# source: https://jtreminio.com/blog/running-docker-containers-as-current-host-user/
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USER_NAME=user
ENV HOME=/home/$USER_NAME
RUN groupadd -g ${GROUP_ID} $USER_NAME && \
    useradd -l -u ${USER_ID} -g $USER_NAME $USER_NAME && \
    install -d -m 0755 -o $USER_NAME -g $USER_NAME $HOME
ENV PATH="/home/$USER_NAME/.local/bin:$PATH"

# add apt repos and install apt packages
USER root
RUN apt-get -y update && \
    # add apt repos
    apt-get install -y apt-utils software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get -y update && \
    # install apt packages
    apt-get install -y \
    git wget ffmpeg \
    libsm6 libxext6 libopencv-dev liblua5.2-dev screen libarmadillo-dev \
    ros-noetic-tf2 ros-noetic-tf ros-noetic-tf2-geometry-msgs \
    ros-noetic-navigation ros-noetic-teb-local-planner ros-noetic-mpc-local-planner \
    ros-noetic-nlopt ros-noetic-turtlebot3-description ros-noetic-turtlebot3-navigation \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential \
    python3 python3-dev python3-pip libpython3-dev python3-catkin-pkg-modules python3-rospkg-modules \
    python3-empy python3-setuptools python-tk python3-tk \
    tk tk-dev \
    && apt-get clean && apt-get autoremove

# install poetry and update pip
USER $USER_NAME
WORKDIR $HOME
RUN pip3 install poetry \
    && pip3 install --upgrade pip

# create virtualenv and install poetry dependencies
COPY src/poetry.lock src/pyproject.toml ./
RUN poetry install

# install pyenv
WORKDIR $HOME
RUN git clone --depth=1 https://github.com/pyenv/pyenv.git $HOME/.pyenv
ENV PYENV_ROOT="$HOME/.pyenv"
ENV PATH="${PYENV_ROOT}/shims:${PYENV_ROOT}/bin:${PATH}"
RUN export PATH="$PYENV_ROOT/bin:$PATH" >> $HOME/.bashrc \
    eval "$(pyenv init -)" >> $HOME/.bashrc

# prepare catkin workspace (src directory contains packages)
RUN mkdir -p $HOME/src/

# add setup to .bashrc to source ros setup when executing bash directly
RUN echo -e "source /opt/ros/noetic/setup.bash" >> $HOME/.bashrc

# install ros dependencies
WORKDIR $HOME/src
COPY src/.rosinstall .
RUN rosws update

# build ros dependencies to cache them
WORKDIR $HOME
RUN source $HOME/.bashrc \
    && source /opt/ros/noetic/setup.sh \
    && catkin_make

# install pip dependencies of stable baselines (has to be installed as ros dep before)
WORKDIR $HOME/src/utils/stable-baselines3
# does this install torch double?
RUN pip install -e .

# build new packages and launch ros launch file
WORKDIR $HOME
USER $USER_NAME
CMD catkin_make \
    && source $HOME/devel/setup.sh \
    && roslaunch ${LAUNCH_PACKAGE} ${LAUNCH_FILE} ${ADDITIONAL_LAUNCH_PARAMS}