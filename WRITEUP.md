The difference between the backyard flyer solution and the motion planning solution is implementation of the planning of
 the drone's path to a destination given obstacles created from the digitalization of the map shown below.
 
 ![SF Map](./misc/map.png)

As the backyard flyer does not have a map it's movements are based on relative offsets from the local position of the 
drone’s start point and any waypoints need to be set by the user. The motion planning code is based on global positions 
and the movement is planned based on object avoidance to get to a defined destination. The motion planning script uses
A star to come up with an efficient and safe path to the destination given tolerances. See an example of a generated 
path below.

![Trajectory](./misc/flyer_on_trajectory.png)