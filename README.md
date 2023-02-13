# BELIEF_SPACE_PLANNING
Navigating an agent in a 2d environment in the belief space. based on the belief space navigation approach.
A stohastic propagation model of the agent is given. measurment beacons are scattered on the map. 
In this implimintation the agent navigate in the belief space by generating a set of trajectories around the beacons.
The belief is propagated (updated) using a classic kalman filtering approach. 
For each generated trajectory a cost function is calculated based on the determinant of the accumulated 2d covariance matrix.
The lowest cost trajectory is chosen and thus the best actions policy.

By Eli Goldenshluger & Nitzan Makmal

![image](https://user-images.githubusercontent.com/104156586/218392547-4aa9d1a3-e3b8-4e7d-a728-6a970ab82791.png)![image](https://user-images.githubusercontent.com/104156586/218392586-2de29b95-4821-4aba-9eaa-fb807dd1e952.png)


![image](https://user-images.githubusercontent.com/104156586/218393128-5d42e0a7-8678-42d7-942e-3598653fca3d.png)



![image](https://user-images.githubusercontent.com/104156586/218325900-c34e6631-196d-456f-8131-b659497a496e.png)
