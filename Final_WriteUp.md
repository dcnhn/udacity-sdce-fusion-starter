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
Both matrices have the dimension *6 x 6* as the state vector __x__ consisted of *6* states ($x$, $y$, $z$, $v_x$, $v_y$, $v_z$).

The update required the implementation of the residual $\gamma$ and its covariance matrix __S__, respectively.
Within the update method, the error covariance matrix __P__ and the state vector __x__ were updated using $\gamma$, __S__, the Kalman gain __K__ and the Jacobian __H__, respectively.

After completion, the execution of *loop_over_dataset.py* showed below RMSE plot:

![local image](doc/final01.png)


#### 2. Track Management
For this section, it was required to solve the following:
1. Initialize a new track with the actual measurement.
2. Handle unassigned tracks by reducing their track score and deleting them from the track list in case the delete criteria are met.
3. Update assigned tracks by increasing their track score and setting their track state to *confirmed* in case the confidence threshold is reached.

To initialize the track with the measurement data, it is important to point out that the measurement data is taken in the sensor domain.
Therefore, it is required to transform the measurement into the vehicle coordinates.
The sensor class already provides the transformation matrix T from sensor to vehicle space using homogeneous coordinates.
The positional entries of the error covariance matrix __P__ can also be initialized using the same transformation matrix.
The velocity entries of __P__ will be initialized with hard-coded values of *params.py* as none of the sensors measures the speed.

To reduce the track score of unassigned tracks, the following equation was used:
$A \over W$
*W* is the window size to be monitored for the tracking. *A* is the number of assignments in the previous *W* cycles.

The deletion of a track was performed if at least one of the following conditions was met:
1. The track score of a __confirmed__ track dropped below the deletion threshold
2. The error covariance of the x-posiion was above the maximum allowed error covariance
3. The error covariance of the y-posiion was above the maximum allowed error covariance
4. No measurement was assigned to the track for M consecutive cycles

In case a measurement was assigned to the track, the following steps were performed:
1. Add $1 \over W$ to the track score
2. Set the track state to *confirmed* if the track score is above the *confirmed* threshold

After completion, the execution of *loop_over_dataset.py* showed below RMSE plot:

![local image](doc/final02.png)


### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 


### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?


### 4. Can you think of ways to improve your tracking results in the future?

