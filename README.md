<snippet>
  <content><![CDATA[
# ${1:Particle Soup}
This project simulates and visualizes dynamics between charged particles using Coloumb's law. Enter in the particle masses, charges, initial position and velocity data, and get started! All of the files The "Particle Soup" files are used
## Basics and Setup
The simulations are split into 1D, 2D, and 3D versions. All of the simulation files use one of the "Particle_Dynamics_(X)D.m" function files to calculate the changing states of the system using a standard forward Euler method. The dynamics are calculated by summing the forces acting on each dynamic particle, calculating the acceleration, then integrating this over a discrete time step to calculate the new system states (position, velocity). Each simulation files contains an animation section aswell. The size of each particle is proportional to its mass, and the color and brightness is proportional to its charge (blue=positive, red=negative, black=neutral). 
## Particle Soup
The The "Particle Soup" files are the best place to start. These files simulate a series of free moving particles contained or semi-contained by a number of fixed massless boundary charges. Try playing around with the masses, charges, initial state conditions, the number of particles, and boundary conditions of the system. Notice how the fixed boundary charges add energy to the system by exerting forces on the system without moving. See how long you can keep the particles contained! Note: All fixed and free particles should be positivley charged. 
## Controlled Particles

## History
TODO: Write history
## Credits
TODO: Write credits
## License
TODO: Write license
]]></content>
  <tabTrigger>readme</tabTrigger>
</snippet>
