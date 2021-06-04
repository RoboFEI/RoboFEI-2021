# RoboFEI-2021

# Webots's installation

1. Install the following:

        sudo apt-get update
        sudo apt-get upgrade
        sudo apt-get install python-dev
        sudo apt-get install python-is-python3
        sudo apt-get install default-jdk
        sudo apt-get install libboost-all-dev
        sudo pip install transforms3d
        sudo apt install git-all
        sudo apt install libtinfo5

2. Follow the first step in the instructions at the link bellow to install Webots:

       https://github.com/RoboCup-Humanoid-TC/webots/tree/release/projects/samples/contests/robocup


# How to run the controler

1. Open path/to/controllers/Control/Control.cpp.
2. On line 124, change "password" to your user Linux's password.
3. To compile the controler in Webots, first remove the intermediate build files (comb icon), then build the project (gear icon).
4. Run the simulation.