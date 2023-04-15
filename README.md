# Black box safety validation of autonomous navigation systems
## Abstract
<!-- Safety validation is an important aspect of autonomous navigation -->
With the increasing prevalence of autonomous technology in various fields such as transportation and manufacturing, it is essential to ensure that these systems not only have the best possible navigation performance, but also operate safely and reliably.

<!-- Falsification as a method for safety validation-->
While the navigation performance can be calculated through the mean of many problem samples, the safety validation has to be handled differently, because only one self-inflicted counter example is enough to cause the safety validation to fail completely. This disproof of a scientific statement (here: "The ANS is safe.") through a counter example is called **falsification**.

<!-- Formulation as a black box safety validation problem -->
One way to approach safety validation is as a **black box validation problem**, meaning that the validation of a system is done without information about the internals of this system. 
This approach can be useful, because information about the internal behavior is often not available.

<!-- Simulation of realistic pedestrian behavior is essential for safety validation -->
As the environments of the autonomous agents almost always include other agents like pedestrians, an essential part of the training and safety validation of autonomous systems is the **simulation of realistic pedestrian behavior**. Without this, unexpected pedestrian movements or interactions could potentially lead to safety issues like crashes. 

<!-- Safety performance tradeoff -->
Although one can reduce these risks by implementing stricter safety measures, such as driving slower or maintaining a larger distance from other agents, this negatively affects the system's performance by increasing the time required to reach the destination. So there is a **tradeoff between safety and performance**.

<!-- Increased difficulty of simulation because of multiple agents -->
However, the presence of other moving agents in the environment creates a significant challenge, as each agent has not only a position (as static objects), but a velocity and acceleration as well.
This leads to a **combinatorial explosion** of possible system states, resulting in large observation and action spaces.

<!-- Problem formulation as criticality plausibility optimization problem -->
The falsification process can be described as an algorithm that terminates, when a counter example was found that disproves the safety (= finds a critical situation) and is also plausible.
Criticality in this context means, if a situation breaks or gets close to breaking a safety property, like resulting in a crash or being smaller than a pre-defined time-to-collision.
The degree of plausibility is a measure of how closely the behavior of simulated agents matches that of real agents.
There is a **tradeoff between plausibility and criticality**, meaning you have a bigger set of critical counter examples, if you don't need to make sure that they're plausible as well.

<!-- Project setup as a Python ROS package-->
The whole project is implemented in Python as a ROS packages. The Robot Operating System (ROS) is a popular framework for developing autonomous systems and provides a lot of useful tools for simulation, visualization and communication between different components of the system.

<!-- Flatland as training environment -->
The first step to do safety validation is the setup of a **training environment** - either 2D or 3D. Gazebo is the most popular ROS simulation environment for 3D. In this project, we chose a 2D simulation environment called *Flatland*.

<!-- openai gym env as standardized interface to the simulation environment -->
It is best practice to conform to a standardized environment interface the most wide-spread being *openai gym*, so we implemented our own environment conforming to the *openai gym* interface.

<!-- Only if we get synchronized steps working -->
Also, the *flatland* implementation doesn't support synchronized steps, which is a requirement for the *openai gym* interface. So we creates a fork of the *flatland* repository to provide a service for synchronized steps. The changes are mostly copied from the ArenaRosnav fork of the *flatland* repository. The fork couldn't be used directly because of non-modularized mixup with pedsim features added in this fork.

<!-- Stable baselines 3 as training process wrapper and for reinforcement learning algorithms -->
We use *stable-baselines3* as a lightweight wrapper around the *openai gym* interface and to provide a lot of useful features like logging, saving and loading of models and reinforcement learning algorithms.

 containing both the System Under Test (SUT) and the surrounding pedestrians. As mentioned above, the SUT's behavior is considered as a black box and is not part of the training process - it's just an agent in the environment. Only the pedestrians are trained.

<!-- Reinforcement learning problem -->
The problem can be formulated as a reinforcement learning problem with the environment being the 

<!-- Focus and status of this project -->
This project mostly focused on the general concept and the environment setup. 

## Features of this project
Features of this project include:
* flatland as 2D simulation environment
* ROS Noetic as middleware
* Python 3.8.10 as programming language to define the training process in a ROS package
* stable-baselines3 as training process wrapper and for reinforcement learning algorithms
* runs as a Docker container to simplify the setup process and to make it more portable
* share windows inside container with host (with X11 forwarding)
* add a custom user during image build process to avoid permission issues in the mounted volumes
* train an agent to reach a target in a 2D rectangle environment without obstacles

## Installation
The installation process is documented [here](docs/Installation.md).

## Usage
The usage is described [here](docs/Usage.md).

## Abbreviations
| Syntax | Description                  |
|--------|------------------------------|
| ANS    | Autonomous Navigation System |
| SUT    | System Under Test |

## Resources
### Related papers
* [Towards simulation-based verification of autonomous navigation systems](https://www.sciencedirect.com/science/article/abs/pii/S092575352030196X)
* [Arena-Rosnav: Towards Deployment of Deep-Reinforcement-Learning-Based Obstacle Avoidance into Conventional Autonomous Navigation Systems](https://arxiv.org/abs/2104.03616)
* [A Survey of Algorithms for Black-Box Safety Validation of Cyber-Physical Systems](https://dl.acm.org/doi/pdf/10.1613/jair.1.12716)

### Used third party repos
* [Arena Bench Github](https://github.com/ignc-research/arena-rosnav)
* [Arena Bench Docs](https://github.com/ignc-research/arena-rosnav)
