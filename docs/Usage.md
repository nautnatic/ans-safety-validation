# Usage
To avoid environment-specific problems, the environment is run in a Docker container. The development workflow with Docker basically consists of the following steps:
1. Build image containing all dependencies that change very rarely by executing [`build-image.sh`](../scripts/build-image.sh)
2. Run a container of the image (this projects code is mounted as a volume) by executing [`run-container`](../scripts/run-container.sh)
3. Attach to the container shell by executing [`exec-container-shell.sh`](../scripts/exec-container-shell.sh)
4. Build the project with `catkin_make`
5. Source the generated setup script with `source /root/devel/setup.sh`
6. Launch a launch configuration of a ROS package e.g. with `roslaunch arena_bringup start_arena.launch`


## Build image
Everything that doesn't need to be redone (or is only changed very rarely) when modifying this projects behavior, should be built into the image. This includes the fetching of dependencies and an initial build to cache the built dependencies and be able to build faster when using the image later.

You can execute [this](../scripts/build-image.sh) shell script to build the image. In the script a Dockerfile is used to build an image called arena.

## Run container
To launch the environment, run a container of the created image by executing [this](../scripts/run-container.sh) shell script.

In this script, a container of the previously built image is created through a docker-compose file. Also, in this file the X11 server is mapped to the outside to show the GUIs of the windows opening in the container on the host system.

## Open a shell inside the container
### Remote Development with VS Code
For development we recommend to do remote development inside the container with VS Code. This allows to view the filesystem as well. To allow development with Python, the pip packages _flake8_ and _autopep8_ are already installed in the Dockerfile.

First, you need to install the [Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) extension pack.

Next, build and run the image as described in the steps above.

Last, execute the action `Remote-Containers: Attach to Running Container...` in the command palette (Ctrl+Shift+P) and select the container name (default: _ans-env_).

### Execute shell inside the container
Alternatively, you can start a bash shell inside the container by executing the [`exec-container-shell.sh`](../scripts/exec-container-shell.sh) shell script.

## Launch simulation
When you have a shell inside the container, you can launch the simulation with the `launch.sh` script, which is mounted in the users home directory inside the container. This script rebuilds the project (`catkin_make`), sources the ROS setup scripts and launches the simulation (`roslaunch`).