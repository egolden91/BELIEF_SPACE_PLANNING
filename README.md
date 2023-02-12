# BELIEF_SPACE_PLANNING
Navigating an agent in a 2d environment in the belief space. based on the belief space navigation approach.
A stohastic propagation model of the agent is given. measurment beacons are scattered on the map. 
In this implimintation the agent navigate in the belief space by generating a set of trajectories around the beacons.
The belief is propagated (updated) using a classic kalman filtering approach. 
For each generated trajectory a cost function is calculated based on the determinant of the accumulated 2d covariance matrix.
The lowest cost trajectory is chosen and thus the best actions policy  

![image](https://user-images.githubusercontent.com/104156586/218325849-e4f80e0e-5a1a-4237-9a67-9a885d1badc0.png)


![image](https://user-images.githubusercontent.com/104156586/218325900-c34e6631-196d-456f-8131-b659497a496e.png)
