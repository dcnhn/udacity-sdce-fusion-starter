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
Both matrices have the dimensions *6 x 6* as the state vector __x__ consisted of *6* states ($x$, $y$, $z$, $v_x$, $v_y$, $v_z$).

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
The sensor class already provides the transformation matrix __T__ from sensor to vehicle space using homogeneous coordinates.
The positional entries of the error covariance matrix __P__ can also be initialized using __T__.
The velocity entries of __P__ will be initialized with hard-coded values of *params.py* as none of the sensors measure the speed.

To reduce the track score *s* of unassigned tracks, the equation $s = {a \over w}$ was used.
*w* is the window size to be monitored for the tracking. *a* is the number of assignments in the previous *w* cycles.
It is important to note that the track score should only be reduced if the unassigned track is located within the field of view (FOV) of the
currently processed sensor.

The deletion of a track was performed if at least one of the following conditions was met:
1. The track score of a __confirmed__ track dropped below the deletion threshold
2. The error covariance of the x-position was above the maximum allowed error covariance
3. The error covariance of the y-position was above the maximum allowed error covariance
4. No measurement was assigned to the track for M consecutive cycles

In case a measurement was assigned to the track, the following steps were performed:
1. Add $1 \over w$ to the track score *s*
2. Set the track state to *confirmed* if *s* is above the *confirmed* threshold. Otherwise, assign *tentative*.

After the completion of the above-mentioned steps, the execution of *loop_over_dataset.py* showed below RMSE plot:

![local image](doc/final02.png)

#### 3. Data Association
The data association required us to implement the logic to map one measurement to one specific track.
The association is represented by the association matrix __A__ of the dimensions *t x m*.
*t* is the number of tracks, whereas *m* describes the number of measurements.
The association matrix contains the *Mahalanobis* distance between each track and measurements.

To complete this section, it was required to implement the following methods:
1. The computation of the Mahalanobis distance
2. The gating check for the computed Mahalanobis distance
3. The logic to find a pair of measurement and track

The gating and the mahalanobis distance were implemented according to the material from the lessons.

To find a pair of measurement and track, the following logic was used:
1. Find the element *a* with the __minimum__ Mahalanobis distance in __A__
    * The row of the element represents the track
    * The column of the element represents the corresponding measurement
2. Remove both the row and column containing *a* from __A__.
3. Repeat until no entries in __A__ are left. 

Instead of literally removing the row and column from __A__ as described above, I set the corresponding entries to infinity.
The reason is that the initial approach to remove entries led to issues with the data integrity.

After the completion of the above-mentioned steps, the execution of *loop_over_dataset.py* showed below RMSE plot:

![local image](doc/final03.png)

#### 4. Camera Fusion
In the last section of this project, we had to include the processing of the camera data by uncommenting a statement in *loop_over_dataset.py*.
The constructor of the *Measurement* class had to be extended by the handling of the camera sensor.
Here, it is important to point out that the camera measures the data in two dimensions resulting in a measurement covariance matrix __R__ with the
dimensions *2 x 2*. The covariance entries were initialized using *params.py*.
Another aspect to keep in mind is the different FOVs of each sensor which can lead to the incorrect computation of the tracking metrics
if not taken into account. For example, a road user is only located inside the FOV of the lidar.
The track score of the road user will increase over time as it is perfectly detected by the lidar.
However, the track score is incorrectly decreased as soon as the camera sensor is processed even though the track is not within the camera's FOV.
To avoid this issue, the method *in_fov* of the Sensor class had to be implemented. The method returns a flag stating if
a track is inside the FOV. The position data is basically transformed to the sensor coordinates using the inverse of __T__.
Then, the polar angle is computed and a range check is done.
At last, we had to implement the nonlinear measurement function of the camera as the function is required to compute the residual $\gamma$ for the
Extended Kalman Filter.

After the completion of the above-mentioned steps, the execution of *loop_over_dataset.py* showed below RMSE plot:

![local image](doc/final04.png)

This is the resulting video of the tracking project:

![](results/my_tracking_results.mp4)

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)?
In my opinion, it is mandatory to have a camera-lidar fusion for tracking as this will introduce redundancies.
The camera will also help to stabilize the tracking as seen in this project.
Furthermore, the camera's strength is the object classification.
This information can be used to have different kinematic models for the different road users (pedestrian, cars, cyclists etc.).

### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?
* tuning the noise variables for the different systems (Automated Driving vs. Active Safety)
* data association
* scalability if only rule-based approaches are used
* data synchronization
* delays in the system

### 4. Can you think of ways to improve your tracking results in the future?
* using a more complex model for the prediction, e.g. the bicycle model
* parameter tuning for error and measurement covariance matrices
* using the Unscented Kalman Filter
* improving the object detection algorithms of both lidar and camera (training the models)
* estimate other measures such as width height length
