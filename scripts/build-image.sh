# define the id of the current host user as the user run the
# commands and own the files inside the container. This is necessary
# to avoid file permission problems in the mounted volumes.

if [ "$USE_IMAGE_BUILD_CACHE" = true ]; then
  docker build \
    -t $DOCKER_IMAGE_TAG \
    --build-arg USER_ID="$(id -u "${USER}")" \
    --build-arg GROUP_ID="$(id -g "${USER}")" \
    ../environment-setup
else
  docker build \
    -t $DOCKER_IMAGE_TAG \
    --build-arg USER_ID="$(id -u "${USER}")" \
    --build-arg GROUP_ID="$(id -g "${USER}")" \
    --no-cache \
    ../environment-setup
fi
