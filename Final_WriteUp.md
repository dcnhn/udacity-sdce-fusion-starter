# Track 3D-Objects Over Time

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?
In the final project, we had to develop the multi-target tracking of objects using recordings from the Waymo Open Dataset.
More specifically, we had to use the data of both the lidar and camera for the tracking task.
To succeed in the tracking, four tasks had to be solved:
1. Implement the equations of the Kalman Filter
2. Set up up the track management
3. Develop the association between the tracks and the sensor measurements
4. Integrate the camera sensor data into the tracking

#### 1. Kalman Filter Implementation
In this section, we had to implement both the prediction and update of the Kalman Filter algorithm.
For the prediction, the methods to compute the system matrix __F__ and the error covariance matrix __Q__ had to be implemented.
Both matrices have the dimension *6x6* as the state vector __x__ consisted of *6* states *(x, y, z, v<sub>x</sub>, v<sub>y</sub>, v<sub>z</sub>)*.

The update required the implementation of the residual $\gamma$, its covariance matrix __S__, respectively.

After completion, the execution of *loop_over_dataset.py* showed below RMSE plot:

![local image](doc/final01.png)

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 


### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?


### 4. Can you think of ways to improve your tracking results in the future?

