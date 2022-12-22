# Installation
In this project the application is run in a Docker container. To be able to 
customize the contents of the Docker image, not the Docker image itself, but the
Dockerfile is provided. 

## Build Docker image
You can customize the Dockerfile if necessary and then build the image by 
executing *build.sh* in the runtime directory. This creates a Docker image called *ros*.

## Open shell inside the ros container
To enter the shell in the ros container, execute the *shell.sh* script in the runtime directory.

## Launch a ros launch file
Execute the *launch.sh* script in the runtime directory, which uses the launch settings defined in the environment variables.

---

# Installation documentation for non docker builds

Change into dir

```
cd arena-rosnav
```

Ros install

```
rosws update
```

Install python pkgs, you need poetry for this

```
poetry shell && poetry install
```

Install stable baselines

```
cd ../forks/stable-baselines3 && pip install -e .
```

Build catkin

```
cd ../../.. && catkin_make
```

Finished!

#### TODO

Everything
