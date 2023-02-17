# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Process the point-cloud and prepare it for object detection
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# general package imports
import cv2
import numpy as np
import torch
import zlib

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

# waymo open dataset reader
from tools.waymo_reader.simple_waymo_open_dataset_reader import utils as waymo_utils
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2, label_pb2

# object detection tools and helper functions
import misc.objdet_tools as tools

# Constants to access range image channel dictionary
RANGE = 0
INTENSITY = 1
ELONGATION = 2
INSIDE_NO_LABEL_ZONE = 3

# Extract lidar from frame data
def extractLidar(frame, lidar_name):
    # Return variables
    found = False
    lidar = None

    # Sanity check for lidar_name
    if (dataset_pb2.LaserName.UNKNOWN < lidar_name <= dataset_pb2.LaserName.REAR):

        # Store lidars in a local variable
        lidars = frame.lasers

        # Iterate over serialized lidar data
        for l in lidars:

            # Compare name of current element with lidar_name
            if l.name == lidar_name:

                # Set return variables
                found = True
                lidar = l

    return found, lidar

def loadRangeImage(lidar):
    # Return variable
    rangeImage = []

    # Save the range image bytestream to a local variable
    # As mentioned in the course, we process the first response
    rangeImageStream = lidar.ri_return1.range_image_compressed

    # Check if the stream contains data by checking its length
    if (len(rangeImageStream) > 0):
        
        # Bytestream contains data. Convert return variable
        # to a float matrix of the dataset_pb2 package
        rangeImage = dataset_pb2.MatrixFloat()

        # Now we have to parse the stream to be able to process the data
        rangeImage.ParseFromString(zlib.decompress(rangeImageStream))
        
        # Convert data to a numpy array and zero out all negative entries
        rangeImage = np.array(rangeImage.data).reshape(rangeImage.shape.dims)
        rangeImage[rangeImage < 0] = 0.0

    return rangeImage

def getRangeImageChannels(rangeImage):
    # According to dataset.proto the channel mapping is:
    # 0: range
    # 1: intensity
    # 2: elongation
    # 3: is in no label zone
    return {
        RANGE: rangeImage[:, :, 0],
        INTENSITY: rangeImage[:, :, 1],
        ELONGATION: rangeImage[:, :, 2],
        INSIDE_NO_LABEL_ZONE: rangeImage[:, :, 3]
    }

def handleOutliers(channel, lowerOutlierPercentile = 0, upperOutlierPercentile = 100, scalingTerm = 0.3):
    # This method will scale outliers to the percentile value while maintaining the relative positions to each other.
    #
    # scalingTerm = 0.3 means:
    # The maximum will be 1.3 * upperPercentile, other upper outliers will be scaled between [upperPercentile, 1.3 * upperPercentile]
    # The minimum will be 0.7 * percentileLower, other lower outliers will be scaled between [0.7 * lowerOutlierPercentile, lowerOutlierPercentile]

    # Save channel in local copy as arrays are mutable. Don't wannt to call by reference
    channelCpy = channel.copy()

    # Handle invalid input
    lower = np.max([0, lowerOutlierPercentile])
    upper = np.min([100, upperOutlierPercentile])
    scaleTerm = np.max([0.05, scalingTerm])

    # Check if outlier handling is desired.
    if (0 < (upper - lower) < 100):
        # Save percentiles in local variable
        percentileLower = np.percentile(channelCpy, lower)
        percentileUpper = np.percentile(channelCpy, upper)

        # Get indices of outliers. We get a tuple of arrays of same length. 
        # First array represents: the row index
        # Second array represents: the col index
        idxUpper = np.where(channelCpy > percentileUpper)
        idxLower = np.where(channelCpy < percentileLower)

        # Compute baseline for scaling
        baseUpper = np.max(channelCpy) - percentileUpper
        baseLower = percentileLower - np.min(channelCpy)

        # Iterate over upper outliers
        for rowIdx in range(len(idxUpper[0])):
            
            # For readablity save indices in local variable
            iRow = idxUpper[0][rowIdx]
            iCol = idxUpper[1][rowIdx]

            # Flag indicating negative percentile
            isNegative = (percentileUpper < 0.0)

            # Compute factor to scale
            factor = (channelCpy[iRow, iCol] - percentileUpper) / baseUpper
            factor *= scaleTerm

            # Finally, scale outlier.
            channelCpy[iRow, iCol] = (percentileUpper * (1.0 - factor)) if isNegative else ((percentileUpper * (1.0 + factor)))

        # Iterate over lower outliers
        for rowIdx in range(len(idxLower[0])):
            
            # For readablity save indices in local variable
            iRow = idxLower[0][rowIdx]
            iCol = idxLower[1][rowIdx]

            # Flag indicating negative percentile
            isNegative = (percentileLower < 0.0)

            # Compute factor to scale
            factor = (percentileLower - channelCpy[iRow, iCol]) / baseLower
            factor *= scaleTerm

            # Finally, scale outlier.
            channelCpy[iRow, iCol] = (percentileLower * (1.0 + factor)) if isNegative else ((percentileLower * (1.0 - factor)))

    return channelCpy
        

def scaleDataToUInt8(channel):
    # DISCLAIMER:
    # I don't normalize the data using the formula of the class.
    # Instead, I just divide by the maximume value
    # 
    # Reason: Overflows!
    # Let's scale foo = np.array([5, 10, 6, 7, 8]) -> delta = max - min = 10 - 5 = 5
    #
    # Now, scale data according to course material
    # A. foo / delta: [1,   2,   1.2, 1.4, 1.6]
    # B. * 255:       [255, 510, 306, 357, 408] (numbers > 255 will be decreased by 256)
    # C. cast(uint8): [255, 254, 50,  101, 152]
    #
    # This behavior totally corrupts the data.
    scaledChannel = channel * 255 / np.max(channel)
    return scaledChannel.astype(np.uint8)


# visualize lidar point-cloud
def show_pcl(pcl):

    ####### ID_S1_EX2 START #######     
    #######
    print("student task ID_S1_EX2")

    # step 1 : initialize open3d with key callback and create window
    
    # step 2 : create instance of open3d point-cloud class

    # step 3 : set points in pcd instance by converting the point-cloud into 3d vectors (using open3d function Vector3dVector)

    # step 4 : for the first frame, add the pcd instance to visualization using add_geometry; for all other frames, use update_geometry instead
    
    # step 5 : visualize point cloud and keep window open until right-arrow is pressed (key-code 262)

    #######
    ####### ID_S1_EX2 END #######     
       

# visualize range image
def show_range_image(frame, lidar_name):

    ####### ID_S1_EX1 START #######     
    #######
    print("student task ID_S1_EX1")
    
    # step 1 : extract lidar data and range image for the roof-mounted lidar
    found, lidar = extractLidar(frame, lidar_name)

    # Check if the given lidar was found in the frame
    if found:

        # step 2 : extract the range and the intensity channel from the range image
        # step 3 : set values <0 to zero (is done in loadRangeImage)
        rangeImage = loadRangeImage(lidar)
        channels = getRangeImageChannels(rangeImage)
    
        # step 4 : map the range channel onto an 8-bit scale and make sure that the full range of values is appropriately considered
        rangeUInt8 = scaleDataToUInt8(channels[RANGE])

        # step 5 : map the intensity channel onto an 8-bit scale and normalize with the difference between the 1- and 99-percentile to mitigate the influence of outliers
        intensityUInt8 = handleOutliers(channels[INTENSITY], 1, 99, 0.5)
        intensityUInt8 = scaleDataToUInt8(intensityUInt8)

        # step 6 : stack the range and intensity image vertically using np.vstack and convert the result to an unsigned 8-bit integer
        img_range_intensity = np.vstack((rangeUInt8, intensityUInt8))
    
    #######
    ####### ID_S1_EX1 END #######     
    
    return img_range_intensity


# create birds-eye view of lidar data
def bev_from_pcl(lidar_pcl, configs):

    # remove lidar points outside detection area and with too low reflectivity
    mask = np.where((lidar_pcl[:, 0] >= configs.lim_x[0]) & (lidar_pcl[:, 0] <= configs.lim_x[1]) &
                    (lidar_pcl[:, 1] >= configs.lim_y[0]) & (lidar_pcl[:, 1] <= configs.lim_y[1]) &
                    (lidar_pcl[:, 2] >= configs.lim_z[0]) & (lidar_pcl[:, 2] <= configs.lim_z[1]))
    lidar_pcl = lidar_pcl[mask]
    
    # shift level of ground plane to avoid flipping from 0 to 255 for neighboring pixels
    lidar_pcl[:, 2] = lidar_pcl[:, 2] - configs.lim_z[0]  

    # convert sensor coordinates to bev-map coordinates (center is bottom-middle)
    ####### ID_S2_EX1 START #######     
    #######
    print("student task ID_S2_EX1")

    ## step 1 :  compute bev-map discretization by dividing x-range by the bev-image height (see configs)

    ## step 2 : create a copy of the lidar pcl and transform all metrix x-coordinates into bev-image coordinates    

    # step 3 : perform the same operation as in step 2 for the y-coordinates but make sure that no negative bev-coordinates occur

    # step 4 : visualize point-cloud using the function show_pcl from a previous task
    
    #######
    ####### ID_S2_EX1 END #######     
    
    
    # Compute intensity layer of the BEV map
    ####### ID_S2_EX2 START #######     
    #######
    print("student task ID_S2_EX2")

    ## step 1 : create a numpy array filled with zeros which has the same dimensions as the BEV map

    # step 2 : re-arrange elements in lidar_pcl_cpy by sorting first by x, then y, then -z (use numpy.lexsort)

    ## step 3 : extract all points with identical x and y such that only the top-most z-coordinate is kept (use numpy.unique)
    ##          also, store the number of points per x,y-cell in a variable named "counts" for use in the next task

    ## step 4 : assign the intensity value of each unique entry in lidar_top_pcl to the intensity map 
    ##          make sure that the intensity is scaled in such a way that objects of interest (e.g. vehicles) are clearly visible    
    ##          also, make sure that the influence of outliers is mitigated by normalizing intensity on the difference between the max. and min. value within the point cloud

    ## step 5 : temporarily visualize the intensity map using OpenCV to make sure that vehicles separate well from the background

    #######
    ####### ID_S2_EX2 END ####### 


    # Compute height layer of the BEV map
    ####### ID_S2_EX3 START #######     
    #######
    print("student task ID_S2_EX3")

    ## step 1 : create a numpy array filled with zeros which has the same dimensions as the BEV map

    ## step 2 : assign the height value of each unique entry in lidar_top_pcl to the height map 
    ##          make sure that each entry is normalized on the difference between the upper and lower height defined in the config file
    ##          use the lidar_pcl_top data structure from the previous task to access the pixels of the height_map

    ## step 3 : temporarily visualize the intensity map using OpenCV to make sure that vehicles separate well from the background

    #######
    ####### ID_S2_EX3 END #######       

    # TODO remove after implementing all of the above steps
    lidar_pcl_cpy = []
    lidar_pcl_top = []
    height_map = []
    intensity_map = []

    # Compute density layer of the BEV map
    density_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    _, _, counts = np.unique(lidar_pcl_cpy[:, 0:2], axis=0, return_index=True, return_counts=True)
    normalizedCounts = np.minimum(1.0, np.log(counts + 1) / np.log(64)) 
    density_map[np.int_(lidar_pcl_top[:, 0]), np.int_(lidar_pcl_top[:, 1])] = normalizedCounts
        
    # assemble 3-channel bev-map from individual maps
    bev_map = np.zeros((3, configs.bev_height, configs.bev_width))
    bev_map[2, :, :] = density_map[:configs.bev_height, :configs.bev_width]  # r_map
    bev_map[1, :, :] = height_map[:configs.bev_height, :configs.bev_width]  # g_map
    bev_map[0, :, :] = intensity_map[:configs.bev_height, :configs.bev_width]  # b_map

    # expand dimension of bev_map before converting into a tensor
    s1, s2, s3 = bev_map.shape
    bev_maps = np.zeros((1, s1, s2, s3))
    bev_maps[0] = bev_map

    bev_maps = torch.from_numpy(bev_maps)  # create tensor from birds-eye view
    input_bev_maps = bev_maps.to(configs.device, non_blocking=True).float()
    return input_bev_maps


