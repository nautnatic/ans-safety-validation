# define the id of the current host user as the user run the
# commands and own the files inside the container. This is necessary
# to avoid file permission problems in the mounted volumes.
docker build \
  --build-arg USER_ID="$(id -u "${USER}")" \
  --build-arg GROUP_ID="$(id -g "${USER}")" \
  -t arena "${PROJECT_DIR}"/runtime