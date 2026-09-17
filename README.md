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

## Experimental parameters

In the following table, we report the parameters used during the experiments and their corrensponding values. 

| Parameter | Value | Description |
|:---|:---:|:---|
| **Action space** | | |
| $v_{\max}$ | 0.25 | Maximum linear velocity |
| $\omega_{\max}$ | 2.5 | Maximum angular velocity |
| $v_{\mathrm{ref}}$ | 0.2 | Linear velocity for *Forward* preference |
| $\omega_{\mathrm{ref}}$ | 2 | Angular velocity for *Left* preference |
| $M_v$ | 11 | Linear velocity bins |
| $M_\omega$ | 21 | Angular velocity bins |
| $M$ | 193 | Feasible actions |
| **Generative model** | | |
| $\sigma_v$ | 0.1 / 0.2 | Standard deviation of $q_{k\mid k-1}^{(u)}$ on linear velocity (Straight lane / Full-lap) |
| $\sigma_\omega$ | 1.5 / 2 | Standard deviation of $q_{k\mid k-1}^{(u)}$ on angular velocity (Straight lane / Full-lap) |
| **Cost function** | | |
| $d_{\mathrm{scale}}$ | 0.105 | Lateral deviation scale |
| $\phi_{\mathrm{scale}}$ | 0.4 | Heading error scale |
| $\gamma_d$ | 7 | Lateral error weight |
| $\gamma_\phi$ | 2 | Heading error weight |
| $\gamma_c$ | 2.5 | Cross-term weight |
| $\gamma_{\mathrm{barrier}}$ | 20 | Barrier gain |
| $\beta$ | 10 | Barrier sharpness |
| $d_{\mathrm{th}}$ | 0.8 | Barrier threshold |
| **Other models** | | |
| $\sigma$ | 0.1 | Standard deviation of $p_{k\mid k-1}^{(x)}$ and $q_{k\mid k-1}^{(x)}$ |


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

