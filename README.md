This update includes most of 88485A's (Quantum Beasts') updated code for VEX WORLDS (25-26 V5RC Push Back). Our code uses ez-template and PROS. Downloading the ez-template base and replacing these files with identical names will result in our current functional code for viewing.

ez-template installation guide: (https://ez-robotics.github.io/EZ-Template/tutorials/installation)

- main.cpp is our main file, including autons, opcontrol (driving), and other competition-related features
- autons.cpp is our autonomous routines (as of this update, we have our updated left and right AWP routines, skills auto is still in the works)
- autons.hpp is the header file for our autonomous routines used in main.cpp
- subsystems.hpp is our header file for subsystem definitions (matchloader, descore wing, center goal gate, intake, outtake) used in main.cpp and autons.cpp
