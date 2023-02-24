# Installation
In the following the installation for this project is described.

The following preconditions need to be fulfilled before using this project:

* Linux as operating system
* X11 window manager
* Docker installed
* docker compose plugin installed
* direnv installed and configured for the current shell

All the scripts are provided as shell scripts, so you need to run on **Linux** to execute them. Also this setup was only tested on Linux (Ubuntu 22.04).

Also, you need to run a **X11 window manager**, because X11 is used to map the GUIs from the container to your host system.

The application is run inside a Docker container, so **Docker** needs to be installed to build and run the docker container. Also, the **docker compose plugin** is used to write the run configuration in a readable way, so it needs to be installed as well.

Environment-specific configuration is done through environment variables, which makes it easy to configure it in a pipeline as well. When executing the scripts locally, I recommend using **direnv**. Direnv automatically loads the environment variables defined in a `.envrc` file in the root directory of the projects, when you cd in the directory itself or one of its subdirectories. As the `.envrc` file is specific to my setup, it isn't committed in git. However, you can find a `.envrc-template` file in the project root directory, which is a template containing all configurable environment variables and a comment with their meaning. You can copy this file to `.envrc` and configure it according to your environment.

When direnv detects changes in the `.envrc` file, it requests to confirm the changes. To do that execute `direnv allow`. Now, you can use the environment variables in all terminal sessions started from within the root directory or one of its subdirectories.

## Installation of Docker
You can find instructions about the installation of Docker in the official documentation [here](https://docs.docker.com/engine/install/ubuntu/).

## Installation of Docker compose plugin
To install the docker compose plugin run:

```bash
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install docker-compose-plugin
```
## Installation of direnv
You can find instructions about the installation of Docker in the official documentation [here](https://direnv.net/docs/installation.html).
Basically, you need to execute 2 steps:
* install the package or binary
* hook direnv into your shell
