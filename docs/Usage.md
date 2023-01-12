# Usage
## Build image
Everything that doesn't need to be redone when modifying this projects behavior, should be built into the image. This includes the fetching of dependencies and an initial build to cache the built dependencies and be able to build faster when using the image later.

You can execute [this](../runtime/build.sh) shell script to build the image. In the script a Dockerfile is used to build an image called arena.

When running a container of this image without overwriting the command, the default command is to execute a launch script according to the environment variables: LAUNCH_PACKAGE, LAUNCH_FILE, ADDITIONAL_LAUNCH_PARAMS, which need to be provided in the run command.

## Run container
To launch the environment, run a container of the created image by executing [this](../runtime/launch.sh) shell script.

In this shell script, a container of the previously built image is created. The environment variables LAUNCH_PACKAGE, LAUNCH_FILE, ADDITIONAL_LAUNCH_PARAMS are passed in to configure which application to launch.

Additionally, the script takes care of showing the GUI on the host operating system.

## Execute shell inside the container
To execute a bash shell inside the running container execute the [shell.sh](../runtime/shell.sh) script.