# Abstract
The topic of this project is **Black-box safety validation of autonomous navigation systems through falsification**. This chapter should give some background.

The quality criteria for an **Autonomous Navigation System (ANS)** don't only include an efficient navigation, but a safety guarantee as well. While efficiency can be calculated through the mean of many problem samples, the **safety validation** has to be done very differently, because only one self-inflicted counter example is enough to cause the safety validation to fail completely.

This disproof of a scientific statement (here: "The ANS is safe.") through a counter example is called **falsification** and is used in this project.

**Black-box** safety validation means that for this validation method, the interior of the ANS doesn't have to be known to validate its safety.

# Simulation environment
This project uses *Arena Rosnav* as a 2D simulation environment. Its setup and usage is described in the [simulation-environment](simulation-environment/README.md) subdirectory.

# Abbreviations
| Syntax      | Description |
| ----------- | ----------- |
| ANS		      | Autonomous Navigation system |

# Related papers
* [Towards simulation-based verification of autonomous navigation systems](https://www.sciencedirect.com/science/article/abs/pii/S092575352030196X)
* [Arena-Rosnav: Towards Deployment of Deep-Reinforcement-Learning-Based Obstacle Avoidance into Conventional Autonomous Navigation Systems](https://arxiv.org/abs/2104.03616)
* [A Survey of Algorithms for Black-Box Safety Validation of Cyber-Physical Systems](https://dl.acm.org/doi/pdf/10.1613/jair.1.12716)

# Used third party repos
* [Arena Rosnav](https://github.com/ignc-research/arena-rosnav)