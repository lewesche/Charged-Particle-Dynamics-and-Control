# Particle Soup
This project simulates and visualizes dynamics between charged particles using Coloumb's law, and tests implementations of various control laws. Enter in the particle masses, charges, initial position and velocity data, and get started! 
## Videos
### 3D Simulation: https://www.youtube.com/watch?v=WoiuuW4Uc2k

2D Simulation: https://www.youtube.com/watch?v=xp0A7h7nltE

1D Simulation: https://www.youtube.com/watch?v=dAiw67qKbkM

## Basics and Setup
The simulations are split into 1D, 2D, and 3D versions. All of the simulation files use one of the "Particle_Dynamics_(X)D.m" function files to calculate the changing states of the system using a standard forward Euler method. The dynamics are calculated by summing the forces acting on each dynamic particle, calculating the acceleration, then integrating this over a discrete time step to calculate the new system states (position, velocity). Each simulation files contains an animation section aswell. The control simulations require the "Actuator_Limits.m" function. The size of each particle is proportional to its mass, and the color and brightness is proportional to its charge (blue=positive, red=negative, black=neutral). 
## Particle Soup
The The "Particle Soup" files are the best place to start. These files simulate a series of free moving particles contained or semi-contained by a number of fixed massless boundary charges. Try playing around with the masses, charges, initial state conditions, the number of particles, and boundary conditions of the system. Notice how the fixed boundary charges add energy to the system by exerting forces on the system without moving. See how long you can keep the particles contained! Note: All fixed and free particles should be positivley charged. 
## Controlled Particles 
1D, 2D, and 3D simulations use the left and right fixed charges as actuators, which can exert positive and negative charges up to a certain maximum magnitude. The control output is determined by a PID algorithm and scaled by the square of the distance from the actuator to the particle. The actuator outputs are scaled by the norm distance to compensate for the inverse square relating the actuator charges and the force experienced by the particle (Coulomb's law). The actuator output is limited by a finite amount of available power, so the available power is distributed among optimal actuators by classifying the effectivness of each actuator based on the particles actual position and desired position. 
### 1D Control
The first simulation controls a simple one mass system. The second controls a two mass system, with each actuator targeting a single mass. The two mass system includes an option to "boost" the actuator output after a certain time by scaling it according the the integral of the "error" signal (actual position-desired position), which is useful when trying to position the two particles close together. 
### 2D/3D Control
The 2D/3D simulations control a single positivley charged particle. It works by grouping the actuators by sides (ex top, bottom, right, left sides for 2D), and using each pair of sides to contorl the particles movement through a single dimension. The control system can be used to move the mass too a desired positions or even track moving targets through 2D/3D space. 
