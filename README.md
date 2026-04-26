<h1>88485A QUANTUM BEASTS</h1>

<p>This update includes most of <em>88485A's (Quantum Beasts')</em> updated code for VEX WORLDS (25-26 V5RC Push Back). Our code uses <em>ez-template</em> and <em>PROS</em>. Downloading the ez-template base and replacing these files with identical names will result in our current functional code for viewing.</p><h1></h1>

ez-template installation guide: (https://ez-robotics.github.io/EZ-Template/tutorials/installation)<h1></h1>

- main.cpp is our main file, including autons, opcontrol (driving), and other competition-related features
- autons.cpp is our autonomous routines (as of this update, we have our updated left and right, and solo AWP routines for matches; however, skills auto is still in the works due to a slight issue with out intake)
- autons.hpp is the header file for our autonomous routines used in main.cpp
- subsystems.hpp is our header file for subsystem definitions (matchloader, descore wing, center goal gate, intake, outtake) used in main.cpp and autons.cpp
