# Medusa Base

This repository holds the Medusa Base code stack for underwater marine vehicles of DSOR-ISR (Dynamical Systems for Ocean Robotics - Institute for System Robotics). It contains the base of the control and navigation stack found in the MEDUSA class of marine vehicles.

[![Build Status](https://ci.dsor.isr.tecnico.ulisboa.pt/buildStatus/icon?job=GitHub+DSOR%2Fmedusa_base%2Fmain)](https://ci.dsor.isr.tecnico.ulisboa.pt/job/GitHub%20DSOR/job/medusa_base/job/main/)
![GitHub last commit (branch)](https://img.shields.io/github/last-commit/dsor-isr/medusa_base/main)
![GitHub contributors](https://img.shields.io/github/contributors/dsor-isr/medusa_base)
[![GitHub issues](https://img.shields.io/github/issues/dsor-isr/medusa_base)](https://github.com/dsor-isr/medusa_base/issues)
[![GitHub forks](https://img.shields.io/github/forks/dsor-isr/medusa_base)](https://github.com/dsor-isr/medusa_base/network)
[![GitHub stars](https://img.shields.io/github/stars/dsor-isr/medusa_base)](https://github.com/dsor-isr/medusa_base/stargazers)
[![License](https://img.shields.io/github/license/dsor-isr/medusa_base?color=blue)](https://github.com/dsor-isr/medusa_base/blob/main/LICENSE)

Introduction
============

The Medusa Base stack is the set of software utilities used at DSOR for simulation and field trials of actual marine vehicles. It comprises a set of ROS packages written in Python and C++ together with external dependencies.

Currently the DSOR vehicles run Ubuntu 18.04 with ROS Melodic and the stack supports and is actively tested on Ubuntu 20.04 (Focal) with ROS Noetic. For this check out our repositories Medusa Gazebo and Medusa Simulation

A simple vehicle dynamic model simulator is included with the stack. It is possible to connect the stack to a Gazebo simulator for access to advanced sensor models, actuators and dynamics.

The software is currently hosted at bitbucket.org and set as private to the DSOR group. A migration for Github and public repositories is envisioned to happen soon.

## Platforms

The stack runs natively in Linux (Ubuntu 20.04LTS) and you will need a ROS NOETIC capable operative system. 

- **Linux users** should consider using a distribution supporting ROS Noetic, ideally Ubuntu 20.04. The medusa stack can run on other OSs or ROS Melodic but support will soon cease.

- For **Windows and macOS users** there are two alternatives:
    - Install a virtualization software (Virtualbox, VMware, Parallels) and create a virtual machine where you will install Ubuntu.

- Install Docker Desktop for Windows/macOS and follow the instructions to run the Medusa Stack in a docker container.

- It is recommended to set up SSH keys and add them to your github account. This will enable you to interact with the DSOR software repositories without having to explicitly using your github credentials.

- If you are connecting to another computer it is recommended to setup `ssh-agent` and key forwarding. This will give you passwordless access also on the computer you are connecting to.