# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        # According to Track class, the state vector x has 6 states:
        # x = [x, y, z, vx, vy, vz]^T
        #
        # Therefore, F will have the dimension 6x6
        #
        # A constant velocity model is assumed leading to the following equations:
        # x_k+1  = x_k + vx_k * dt
        # y_k+1  = y_k + vy_k * dt
        # z_k+1  = z_k + vz_k * dt
        # vx_k+1 = vx_k
        # vy_k+1 = vy_k
        # vz_k+1 = vz_k
        dt = params.dt
        return np.matrix([[1, 0, 0, dt, 0, 0],
                          [0, 1, 0, 0, dt, 0],
                          [0, 0, 1, 0, 0, dt],
                          [0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 1]])
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############

        # We use the assumptions of the lesson:
        # A) Noise through acceleration equal in all dimensions
        # B) Discretization in time domain
        dt = params.dt
        dtSquared = dt * dt
        dtCubic = dtSquared * dt
        qLin = dt * params.q
        qSquared = 0.5 * dtSquared * params.q
        qCubic = 0.333333 * dtCubic * params.q

        # Map values in the matrix according to lesson material
        return np.matrix([[qCubic,   0,        0,        qSquared, 0,        0],
                          [0,        qCubic,   0,        0,        qSquared, 0],
                          [0,        0,        qCubic,   0,        0,        qSquared],
                          [qSquared, 0,        0,        qLin,     0,        0],
                          [0,        qSquared, 0,        0,        qLin,     0],
                          [0,        0,        qSquared, 0,        0,        qLin]])

        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        # For readability, store prediction results in local variables
        xPred = self.F() * track.x
        PPred = self.F() * track.P * self.F().transpose() + self.Q()

        # Update the track object by calling its setters
        track.set_x(xPred)
        track.set_P(PPred)

        pass
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        # Compute residual
        gamma = self.gamma(track, meas)

        # Get Jacobian H for a given state vector x
        H = meas.sensor.get_H(track.x)

        # Compute covariance of residual
        S = self.S(track, meas, H)

        # Compute Kalman Gain
        K = track.P * H.transpose() * np.linalg.inv(S)

        # Compute new estimates
        xEst = track.x + K * gamma
        I = np.identity(params.dim_state)
        PEst = (I - K * H) * track.P

        # Call setters of track object to update the states
        track.set_x(xEst)
        track.set_P(PEst)

        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        # Transform state vector x to the measurement space by calling the measurement function h(x)
        # Note that the measurement class contains a sensor object as an attribute
        hAtX = meas.sensor.get_hx(track.x)

        # Return gamma which is the residual of the new measurement and the result of h(x)
        # gamma = z - h(x)
        return meas.z - hAtX
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############
        return H * track.P * H.transpose() + meas.R
        
        ############
        # END student code
        ############
