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

## Implementation details

<!-- Focus and status of this project -->

This project mostly focused on the general concept and the environment setup. We evaluated the _Arena Rosnav_ framework and implemented an initial training setup using the _Flatland_ environment.

<!-- Experiments with Arena Rosnav showed it is too unstable -->

In the beginning, we tried to implement this project within the _Arena Rosnav_ framework of the _TU Berlin_, who provide a whole training environment and some trained navigation algorithms, that could be validated.

While working with _Arena Rosnav_ we found and fixed some bugs in there, but in time decided it was too unstable at this point in time, so we decided to implement our own solution and use _Arena Rosnav_ only for guidance.

Analog to _Arena Rosnav_ we used _flatland_ as a 2D simulation environment. So if the _Arena Rosnav_ project stabilizes in the future and we decide to use it again, it should be easier to migrate there.

<!-- Project setup as a Python ROS package-->

Our project is implemented as a ROS packages in Python. The Robot Operating System (ROS) is a popular framework for developing autonomous systems and provides a lot of useful tools for simulation, visualization and communication between different components of the system.

<!-- Flatland as training environment -->

The first step to do safety validation is the setup of a **training environment** - either 2D or 3D. Gazebo is the most popular ROS simulation environment for 3D. In this project, we chose a 2D simulation environment called _Flatland_.

<!-- openai gym env as standardized interface to the simulation environment -->

It is best practice to conform to a standardized environment interface the most wide-spread being _openai gym_, so we implemented our own environment conforming to the _openai gym_ interface.

<!-- Stable baselines 3 as training process wrapper and for reinforcement learning algorithms -->

We use _stable-baselines3_ as a lightweight wrapper around the _openai gym_ interface and to provide a lot of useful features like logging, saving and loading of models and reinforcement learning algorithms.


<!-- Synchronized steps not working yet -->

Also, the _flatland_ implementation doesn't support synchronized world steps, meaning the simulation is continuously running. Actions can be executed, but instead of executing a world step afterwards, we wait for a short duration before making the observation.

Arena Rosnav solved this problem by creating a fork of the _flatland_ repository and adding a _step_world_ service to it.

First, we tried to use the fork in our setup directly, but it didn't work out of the box, because it contained other changes regarding _pedsim_.

Next, we tried to extract the parts of the fork responsible for the synchronized steps, but didn't succeed in remaining time.

We committed the changes made in our attempt in a separate branch in this repository, which uses our Flatland fork as a ROS dependency in _.rosinstall_.

## Features of this project

Features of this project include:

- flatland as 2D simulation environment
- ROS Noetic as middleware
- Python 3.8.10 as programming language to define the training process in a ROS package
- stable-baselines3 as training process wrapper and for reinforcement learning algorithms
- runs as a Docker container to simplify the setup process and to make it more portable
- share windows inside container with host (with X11 forwarding)
- add a custom user during image build process to avoid permission issues in the mounted volumes
- train an agent to reach a target in a 2D rectangle environment without obstacles

## Installation

The installation process is documented [here](docs/Installation.md).

## Usage

The usage is described [here](docs/Usage.md).

## Abbreviations

| Syntax | Description                  |
| ------ | ---------------------------- |
| ANS    | Autonomous Navigation System |
| SUT    | System Under Test            |

## Resources

### Related papers

- [Towards simulation-based verification of autonomous navigation systems](https://www.sciencedirect.com/science/article/abs/pii/S092575352030196X)
- [Arena-Rosnav: Towards Deployment of Deep-Reinforcement-Learning-Based Obstacle Avoidance into Conventional Autonomous Navigation Systems](https://arxiv.org/abs/2104.03616)
- [A Survey of Algorithms for Black-Box Safety Validation of Cyber-Physical Systems](https://dl.acm.org/doi/pdf/10.1613/jair.1.12716)

### Used third party repos

- [Arena Bench Github](https://github.com/ignc-research/arena-rosnav)
- [Arena Bench Docs](https://github.com/ignc-research/arena-rosnav)
