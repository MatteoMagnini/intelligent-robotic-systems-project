# Intelligent Robotic Systems Project

This was my project for the exam of the course "Intelligent Robotic Systems" that I did in 2020.
The project is about reinforcement learning (RL) applied to software controllers for robots.
The kind of RL used is Q-learning.
The project consists in evaluating the performance of a Q-learning controller for a robot in a simulated environment for different tasks.
A report (in Italian) is available as a pdf file in the repository (report-ita.pdf).

## Environment

### Simulator: ARGoS
[ARGoS](https://www.argos-sim.info/) is a multi-robot simulator.
It is used to simulate the behavior of robots in a 2D or 3D environment.
It provides a physics engine, a rendering engine, and a plugin system for the robots' controllers.

### Robot: Foot-bot
The robot used in the simulation is the Foot-bot.
Basically, it is a robot with two treels and multiple sensors such as proximity sensors, light sensors and ground sensors.

### Arena
The arena is a squared 3D environment and can be customized with obstacles, paths, and light sources.

## Tasks

### Path Following
The robot has to follow a path in the environment.
The path is a free-form curve.
The goal is to keep moving while staying as close as possible to the path.

### Obstacle Avoidance
The robot has to avoid obstacles in the environment.
The obstacles are static and can be of different shapes.
The goal is to keep moving while avoiding collisions with the obstacles.

### Phototaxis
The robot has to move towards a light source in the environment.

