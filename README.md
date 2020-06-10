# Orbital Simulation

This is an orbital simulator written by Alex White.

##JS Simulator

This simulator was simply a passion project written to observe the emergence of solar system structrues due to gravitational laws. This was written in HTML/CSS/JavaScript with new exterior frameworks or libraries. This project does not follow SI units but instead uses arbitrary units such the orbiting planets would be observed on the user's screen. The project can easily be run by visiting https://ar4bid.github.io/orbital_simm/   

##Python Simulator

This simulator was primarily written for the Queen's Space Engineering Team to perform orbit predictions of a cubesat mission.

As of now this simulation is localized to a single python3 script which requires an external image for visualization of the earth. This image is on the git and was taken from https://visibleearth.nasa.gov/view.php?id=73909. Libraries used include matplotlib for 2d visualization, Mayavi for 3d visualization, PyQt5 for user interface and some others (see script for details). This simulation uses SI units such as kilograms for mass and seconds for time. 

Every second the simulation updates position, velocity and acceleration due to gravity, via leapfrog integration. This method is stable for oscillating systems. If you'd like to know more about the leapfrog method try reading https://en.wikipedia.org/wiki/Leapfrog_integration. Leapfrog integration takes place in the function "stepTime(sat, total_time, dt)". If this method were not used and you simply updated position, velocity and acceleration one by one, the orbit would spiral outward or inward depending on the order of the parameters being updated. This naive approach would be an example of the Euler method or backwards Euler method.

Gravity is currently calculated as if the earth were a point mass. This is sufficient for now but should be updated later to allow for things like orbit precession. 

## TODO for python sim

-Add user interface with the following:
  * Button to start and stop sim
  * View orbital description information
  * View position and altitude information
  * Allow the user to jump X number of seconds into the future or past
  * Specify an orbit by one of the following and then reset the sim:
    * 3 elements of position and 3 element of velocity
    * Inclination, longitude of the ascending node, argument of the periapsis, eccentricity, semimajor axis, mean anomaly at epoch.
  
-Add orientation planning algorithm

-Simulate and/or save information about multiple orbiting objects

-make time based off of user's current time so you could predict the ISS's orbit in real time for example

-Update the gravity function so that the earth isn't a point mass (this should lead to orbital precession)


