# Towards Interaction Regulation from Human Feedback via Free Energy Minimization

This repository contains the code and experimental material for a human-in-the-loop control architecture based on **variational free energy minimization**.

The goal of the project is to integrate **human preferences online** into the policy of an autonomous robot. Human inputs are provided through gestures and are incorporated into the robot decision-making process while preserving the control objective.

The experimental validation is carried out on a **lane-following task** with a real robot and a remote human operator equipped with a VR interface.

## Repository structure

- 'Gesture_control-Lane_following/'  
  Robot-side code for lane following and free-energy-based policy computation.

- 'gesture_recognition/'
  PC-side code for gesture recognition and generation of human input commands.

- 'videos/' 
  Experimental videos.

## Overview

The overall system is composed of two interacting sides:

- a **robot side**, where control actions are computed online;
- a **human side**, where gestures are recognized and translated into preferences sent to the robot.

The resulting architecture enables the robot to mediate between:
- its own task objective;
- externally provided human preferences.

## Documentation

This top-level README only provides a general overview of the project.

For implementation details, dependencies, usage instructions, and package-specific documentation, see the README files inside:

- 'Gesture_control-Lane_following/'
- 'gesture_recognition/'

## Experimental material

The repository also includes video material showing representative experiments, including the straight-lane navigation test with time-varying human preferences.

## Reference

If you use this repository, please cite the associated work:

**[Towards Interaction Regulation from Human Feedback via Free Energy Minimization](https://arxiv.org/abs/2609.18853)**

## Authors of the manuscript

- M. Paula Diaz Monfort  
- Cinzia Tomaselli  
- Michael Richardson  
- Giovanni Russo

## Authors of the code

- M. Paula Diaz Monfort  
- Cinzia Tomaselli  

