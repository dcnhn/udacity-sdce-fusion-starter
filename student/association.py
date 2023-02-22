# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params 

class Association:
    '''Data association class with single nearest neighbor association and gating based on Mahalanobis distance'''
    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []
        
    def associate(self, track_list, meas_list, KF):
             
        ############
        # TODO Step 3: association:
        # - replace association_matrix with the actual association matrix based on Mahalanobis distance (see below) for all tracks and all measurements
        # - update list of unassigned measurements and unassigned tracks
        ############
        nTracks = len(track_list)
        nMeas = len(meas_list)
        self.unassigned_tracks = list(range(nTracks))
        self.unassigned_meas = list(range(nMeas))
        self.association_matrix = np.matrix(np.zeros([nTracks, nMeas]))

        # Iterate over the association matrix
        for row in range(nTracks):
            for col in range(nMeas):
                # Compute MHD
                mhd = self.MHD(track_list[row], meas_list[col], KF)
                self.association_matrix[row, col] = mhd if self.gating(mhd, meas_list[col].sensor) else np.inf
        
        ############
        # END student code
        ############ 
                
    def get_closest_track_and_meas(self):
        ############
        # TODO Step 3: find closest track and measurement:
        # - find minimum entry in association matrix
        # - delete row and column
        # - remove corresponding track and measurement from unassigned_tracks and unassigned_meas
        # - return this track and measurement
        ############

        # Return variable
        update_track = np.nan
        update_meas = np.nan

        # Make local copy of the association matrix
        A = self.association_matrix.copy()

        # Check if minimum of A is less than infinity.
        # This means the matrix still contains valid distance values.
        # Also check if A has still elements
        if (np.min(A) < np.inf):
            # Get indices of the min element using np.where
            # np.where will return a tuple of arrays even if there is only one occurence
            minIndices = np.where(A == np.min(A))

            # In case, we have multiple occurences of min take the first element
            idxTrack = minIndices[0][0]
            idxMeas = minIndices[1][0]

            # Set whole row and col to inf to deactivate it for the association
            # Removing did not work in my case as the data integrity was destroyed
            A[idxTrack, :] = np.inf
            A[:, idxMeas] = np.inf

            # Remove the row of the track, the remove the column
            # Note: Remove them from BOTH the association matrix and the unassigned lists!
            # A = np.delete(A, idxTrack, axis=0)
            # A = np.delete(A, idxMeas, axis=1)
            self.unassigned_tracks.remove(idxTrack)
            self.unassigned_meas.remove(idxMeas)

            # Update the association matrix of self
            self.association_matrix = A.copy()

            # Assign found indices to return variable
            update_track = idxTrack
            update_meas = idxMeas

        ############
        # END student code
        ############ 
        return update_track, update_meas

    def gating(self, MHD, sensor):
        ############
        # TODO Step 3: return True if measurement lies inside gate, otherwise False
        ############
        gateThreshold = chi2.ppf(params.gating_threshold, df=sensor.dim_meas)
        insideGate = MHD < gateThreshold
        return insideGate
        
        ############
        # END student code
        ############ 
        
    def MHD(self, track, meas, KF):
        ############
        # TODO Step 3: calculate and return Mahalanobis distance
        ############

        # Compute residual
        gamma = KF.gamma(track, meas)

        # Get Jacobian H for a given state vector x
        H = meas.sensor.get_H(track.x)

        # Compute covariance of residual
        S = KF.S(track, meas, H)

        # Return gamma^T * inv(S) * gamma
        return gamma.transpose() * np.linalg.inv(S) * gamma
        
        ############
        # END student code
        ############ 
    
    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)
    
        # update associated tracks with measurements
        while self.association_matrix.shape[0]>0 and self.association_matrix.shape[1]>0:
            
            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break
            track = manager.track_list[ind_track]
            
            # check visibility, only update tracks in fov    
            if not meas_list[0].sensor.in_fov(track.x):
                continue
            
            # Kalman update
            print('update track', track.id, 'with', meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])
            
            # update score and track state 
            manager.handle_updated_track(track)
            
            # save updated track
            manager.track_list[ind_track] = track
            
        # run track management 
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)
        
        for track in manager.track_list:            
            print('track', track.id, 'score =', track.score)