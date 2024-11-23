# cogrobs-project

## Description

This repository contains the code for the cogrobs project. The project is a final submition for the course "Cognitive Robots" at the University of Technion Israel.\\
The project includes an online-planner that outputs instructions for the robots, and a simulation that runs on Webots.\\

## About the project

The project is ment to simulate an outomated restaurant.
The simulation includes the following stochastic events (all following poisson processes with different rates):
- customers arrival to the restaurant.
- customers ready to order food.
- food is ready in the kitchen.
- customers finish eating.
- cleaner finishes cleaning the table.

The simulation includes the following robots:
- a waiter that takes the order from the customers and brings the food to the table.
- a cleaner that cleans the table after the customers finish eating.
- a host (drone) that shows the customers to their table.
- main computer that coordinates the robots.

All robot are colidable and needs to be controled in a way that won't cause any collision.\\

the robots tasks:
- collect data from sensors.
- send information to the main computer (via emitter node).
- receive information from the main computer (via receiver node).
- execute the next action according to the message received from the main computer.

The main computer tasks:
- collect data from all robots in the simulation via the emitter-receiver nodes.
- run the online-planner to decide the next plan for the simulation.
- send the temporal plan to the robots in timed manner.
- replan if needed (due to changes in the environment or on fail).

## Usage

To run the simulation, you need to install Webots.\\
After installing Webots, you can run the simulation by opening the world file in the Webots software.\\

you can see the simulation without installing in the following link: [simulation](https://youtu.be/FZmX4vD9C5s) (with joly music for your pleasure)\\

## Credits

The project was developed by myself and my partner.\\

We used:
- [2023 Bitcraze AB in Webots](https://github.com/cyberbotics/webots/tree/master/projects/robots/bitcraze).\\
We modifeid their Crazyflie manual keyboard controler to go to a specific location.
Which, apparently, they had already implemented themselves, and better might we add.

- [LPG planner](https://github.com/aiplan4eu/up-lpg/blob/master/README.md) implemented on the [unified-planning library](https://unified-planning.readthedocs.io/en/latest/getting_started.html)

