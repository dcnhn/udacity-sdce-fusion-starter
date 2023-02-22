# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Classes for track and track management
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
import collections

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Track:
    '''Track class with state, covariance, id, score'''
    def __init__(self, meas, id):
        print('creating track no.', id)
        M_rot = meas.sensor.sens_to_veh[0:3, 0:3] # rotation matrix from sensor to vehicle coordinates
        
        ############
        # TODO Step 2: initialization:
        # - replace fixed track initialization values by initialization of x and P based on 
        # unassigned measurement transformed from sensor to vehicle coordinates
        # - initialize track state and track score with appropriate values
        ############

        # Transform measurement from sensor coordinates to vehicle coordinates
        # The lidar measurement has dimensions = 3.
        # The transformation matrix will have the dimensions 4x4 as it is described in homogeneous  coordinates.
        # Therefore, we have to add another row to z, z_new = [z, 1]^T
        # which represents the translation of the coordinate transformation.
        zHomogeneous = np.ones((4, 1))
        zHomogeneous[0:3] = meas.z
        xVehicle = meas.sensor.sens_to_veh * zHomogeneous

        # Assign the transormed positions to the state vector.
        # Speed is initialized to zero as the lidar does not measure velocities
        self.x = np.ones((params.dim_state, 1))
        self.x[0:3] = xVehicle[0:3]

        # We can use the measurement covariance R to initialize the error covariance P of x, y and z.
        # However, we need to rotate R into the vehicle coordinates.
        # That means we only use the first three columns and rows of sens_to_veh  for the rotation
        rotMatrix = meas.sensor.sens_to_veh[0:3, 0:3]
        posP = rotMatrix * meas.R * rotMatrix.transpose()

        # Use high values for the intial error covariances of the speed.
        # Use a standard deviation of 50 --> variance = standardDev**2 = 50**2
        velP = np.matrix([[params.sigma_p44**2, 0,                   0],
                          [0,                   params.sigma_p55**2, 0],
                          [0,                   0,                   params.sigma_p66**2]])

        self.P = np.matrix(np.zeros([params.dim_state, params.dim_state]))
        self.P[0:3, 0:3] = posP
        self.P[3:6, 3:6] = velP

        self.state = 'initialized'
        self.score = 1 / params.window

        # Mask which describes if a measurement was assigned to the track over the cycles
        # Bit0: Current cycle of execution
        # Bit1 ... BitN: Past cycles
        self.assignMask = np.uint8(0b00000001)

        ############
        # END student code
        ############ 
               
        # other track attributes
        self.id = id
        self.width = meas.width
        self.length = meas.length
        self.height = meas.height
        self.yaw =  np.arccos(M_rot[0,0]*np.cos(meas.yaw) + M_rot[0,1]*np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates
        self.t = meas.t

    def set_x(self, x):
        self.x = x
        
    def set_P(self, P):
        self.P = P  
        
    def set_t(self, t):
        self.t = t
        
    def update_attributes(self, meas):
        # use exponential sliding average to estimate dimensions and orientation
        if meas.sensor.name == 'lidar':
            c = params.weight_dim
            self.width = c*meas.width + (1 - c)*self.width
            self.length = c*meas.length + (1 - c)*self.length
            self.height = c*meas.height + (1 - c)*self.height
            M_rot = meas.sensor.sens_to_veh
            self.yaw = np.arccos(M_rot[0,0]*np.cos(meas.yaw) + M_rot[0,1]*np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates
        
        
###################        

class Trackmanagement:
    '''Track manager with logic for initializing and deleting objects'''
    def __init__(self):
        self.N = 0 # current number of tracks
        self.track_list = []
        self.last_id = -1
        self.result_list = []
        
    def manage_tracks(self, unassigned_tracks, unassigned_meas, meas_list):  
        ############
        # TODO Step 2: implement track management:
        # - decrease the track score for unassigned tracks
        # - delete tracks if the score is too low or P is too big (check params.py for parameters that might be helpful, but
        # feel free to define your own parameters)
        ############
        if meas_list:
            print("manage_tracks: processing ", meas_list[0].sensor.name, "sensor")
        else:
            print("manage_tracks: no measurements available")
        
        # decrease score for unassigned tracks
        tracksToDelete = []
        for i in unassigned_tracks:
            track = self.track_list[i]
            # check visibility
            if meas_list: # if not empty
                if meas_list[0].sensor.in_fov(track.x):

                    # Decrease score if no measurement is provided
                    print("Reducing track score of track no. ", track.id)
                    # Shift assignment mask to the left by 1. Bit 0 will be a zero because no measurement was assigned
                    # to the track
                    track.assignMask <<= 1

                    # Then apply the window mask to the assignment mask with the bitwise AND operation.
                    # That way it is ensured that only the window bits are considered
                    windowedAssignMask = track.assignMask & params.windowMask

                    # To get the new score count all set bits and divide by window
                    track.score = bin(windowedAssignMask).count("1") / params.window

            # Check the delete conditions
            deleteTrack = ((track.P[0, 0] >= params.max_P) or
                            (track.P[1, 1] >= params.max_P) or
                            ((track.score <= params.delete_threshold) and (track.state == "confirmed")) or
                            (track.assignMask & params.deleteMask == 0))

            # Check conditions to delete a track
            if deleteTrack:
                tracksToDelete.append(track)

        # Iterate over delete list and remove each track
        for track in tracksToDelete:
            self.delete_track(track)

        ############
        # END student code
        ############ 
            
        # initialize new track with unassigned measurement
        for j in unassigned_meas: 
            if meas_list[j].sensor.name == 'lidar': # only initialize with lidar measurements
                self.init_track(meas_list[j])
            
    def addTrackToList(self, track):
        self.track_list.append(track)
        self.N += 1
        self.last_id = track.id

    def init_track(self, meas):
        track = Track(meas, self.last_id + 1)
        self.addTrackToList(track)

    def delete_track(self, track):
        print('deleting track no.', track.id)
        self.track_list.remove(track)
        
    def handle_updated_track(self, track):      
        ############
        # TODO Step 2: implement track management for updated tracks:
        # - increase track score
        # - set track state to 'tentative' or 'confirmed'
        ############
        # A measurement was assigned to the track
        # Therefore, update the assignMask
        track.assignMask <<= 1
        track.assignMask += 1

        # To compute the new score, apply the window mask to the assignment mask
        # with the bitwise AND operation. That way it is ensured that only the
        # window bits are considered
        windowedAssignMask = track.assignMask & params.windowMask

        # To get the new score count all set bits and divide by window
        track.score = bin(windowedAssignMask).count("1") / params.window

        # Assigne confirmed if the score is above the threshold
        track.state = "confirmed" if (track.score >= params.confirmed_threshold) else "tentative"

        # In python, I don't know if it is a call by reference
        # Therefore, loop over tracklist and update the track
        for trackObj in self.track_list:
            if (trackObj.id == track.id):
                trackObj = track

        ############
        # END student code
        ############ 