# Installation
In the following the installation for this project is described.

The following preconditions need to be fulfilled before using this project:

* Linux as operating system
* Docker installed
* docker-compose installed
* direnv installed (optional)

All the scripts are provided as shell scripts, so you need to run on **Linux** to execute them. Also this setup was only tested on Linux (Ubuntu 22.04).

The application is run inside a Docker container, so **Docker** needs to be installed to build and run the docker container. Also, the **docker compose plugin** is used to write the run configuration in a readable way, so it needs to be installed as well.
To install the docker compose plugin run:

```bash
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get udpate
sudo apt-get install docker-compose-plugin
```

As you need to set environment variables before running the shell scripts, I recommend using **direnv**, which is a tool to set environment variables automatically depending on the directory you are in.
