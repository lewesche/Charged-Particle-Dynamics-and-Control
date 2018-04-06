# Particle Soup
This project simulates and visualizes dynamics between charged particles using Coloumb's law, and tests implementations of various control laws. Enter in the particle masses, charges, initial position and velocity data, and get started! 
## Basics and Setup
The simulations are split into 1D, 2D, and 3D versions. All of the simulation files use one of the "Particle_Dynamics_(X)D.m" function files to calculate the changing states of the system using a standard forward Euler method. The dynamics are calculated by summing the forces acting on each dynamic particle, calculating the acceleration, then integrating this over a discrete time step to calculate the new system states (position, velocity). Each simulation files contains an animation section aswell. The size of each particle is proportional to its mass, and the color and brightness is proportional to its charge (blue=positive, red=negative, black=neutral). 
## Particle Soup
The The "Particle Soup" files are the best place to start. These files simulate a series of free moving particles contained or semi-contained by a number of fixed massless boundary charges. Try playing around with the masses, charges, initial state conditions, the number of particles, and boundary conditions of the system. Notice how the fixed boundary charges add energy to the system by exerting forces on the system without moving. See how long you can keep the particles contained! Note: All fixed and free particles should be positivley charged. 
## Controlled Particles
So far I have only written control laws for the 1D and 2D simulations. 
# 1D Control
Both controlled 1D simulations use the left and right fixed charges as actuators, which can exert positive and negative charges up to a certain maximum magnitude. The output of the controllers is determined by a PID algorithm (proportional integral derivative "error" sum) and scaled by the square of the distance from the actuator to the particle. The actuator outputs are scaled by the square of the distance to compensate for the inverse square relating the actuator charges and the force experienced by the particle (Coulomb's law). 

The first controls a simple one mass system. The second controls a two mass system, with each actuator targeting a single mass. The two mass system includes an option to "boost" the actuator output after a certain time by scaling it according the the integral of the "error" signal (actual position-desired position), which is useful when trying to position the two particles close together. 
# 2D Control
The first 2D controlled simulations simply groups the right, left, top, and bottom actuators and assigned the same charges to each set based on the outcome of a PID algorithm. For as simple as it is, it works suprisingly well even with a low number of boundary actuators. It requires the 2D Dynamics function file as well as the Actuator Limits function file to run.  


## Videos
1D Simulation: Fixed boundary charges, single mass PID control, multi-mass PID control: https://youtu.be/dAiw67qKbkM
2D Simulation: Coming soon!

