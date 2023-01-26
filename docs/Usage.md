# Usage
## Configure global settings as environment variables through .envrc
Global settings are set as environment variables. I use *direnv* for automatically loading the environment variables, when I'm in the project directory from a `.envrc` file in the root directory of the project. As the file also contains environment-specific setting like local paths, it is not part of the repository. Instead, a template for the .envrc called [`.envrc-template`](../.envrc-template) is committed, which contains placeholders. 

When direnv detects changes in the `.envrc` file, it requests to confirm the changes. To do that execute `direnv allow`. Now, you can use the environment variables in all terminal sessions started from within the root directory or one of its sub directories.

## Build image
Everything that doesn't need to be redone when modifying this projects behavior, should be built into the image. This includes the fetching of dependencies and an initial build to cache the built dependencies and be able to build faster when using the image later.

You can execute [this](../runtime/build.sh) shell script to build the image. In the script a Dockerfile is used to build an image called arena.

When running a container of this image without overwriting the command, the default command is to execute a launch script according to the environment variables: LAUNCH_PACKAGE, LAUNCH_FILE, ADDITIONAL_LAUNCH_PARAMS, which need to be provided in the run command.

## Run container
To launch the environment, run a container of the created image by executing [this](../runtime/run.sh) shell script.

In this shell script, a container of the previously built image is created. The environment variables LAUNCH_PACKAGE, LAUNCH_FILE, ADDITIONAL_LAUNCH_PARAMS are passed in to configure which application to launch.

Additionally, the script takes care of showing the GUI on the host operating system.

## Execute shell inside the container
To execute a bash shell inside the running container execute the [shell.sh](../runtime/shell.sh) script.

## Common errors
### Use correct arena-rosnav repository
Make sure to use the correct Arena Rosnav repository: *git@github.com:Arena-Rosnav/arena-bench.git*. This old, incorrect one is *git@github.com:Arena-Rosnav/arena-bench.git*. The error I got was:

```
arena    | Multiple packages found with the same name "observations":
arena    | - arena-rosnav/arena_navigation/observations
arena    | - utils/arena-utils/observations
arena    | Multiple packages found with the same name "plan_visualization":
arena    | - arena-rosnav/utils/plan_visualization
arena    | - utils/arena-utils/plan_visualization
arena    | Multiple packages found with the same name "sensor_simulator":
arena    | - arena-rosnav/arena_navigation/arena_local_planer/model_based/sensor_simulator
arena    | - utils/arena-utils/sensor_simulator
```