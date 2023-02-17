import numpy as np
import matplotlib.pyplot as plt

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


SHOW_PLOT = False

# Test case 1 all values are negative
a = 10 * np.random.random((20,20))
a = a - 1.1 * np.max(a)

iMin = np.where(a == np.min(a))
iMax = np.where(a == np.max(a))
a[iMin[0][0], iMin[1][0]] = -500
a[iMax[0][0], iMax[1][0]] = 500
b = handleOutliers(a, 1, 99, 0.2)
iMinB = np.where(b == np.min(b))
iMaxB = np.where(b == np.max(b))
print("CASE: All values negative")
print("Min Index Equal: ", iMin[0][0] == iMinB[0][0], iMin[1][0] == iMinB[1][0])
print("Max Index Equal: ",iMax[0][0] == iMaxB[0][0], iMax[1][0] == iMaxB[1][0])

if SHOW_PLOT:
    fig, axs = plt.subplots(2)
    im = axs[0].imshow(a)
    cbar = axs[0].figure.colorbar(im, ax=axs[0])

    im2 = axs[1].imshow(b)
    cbar = axs[1].figure.colorbar(im2, ax=axs[1])
    plt.show()

# Test case 2 negative + positive values
a = 10 * np.random.random((20,20))
a = a - np.mean(a)
iMin = np.where(a == np.min(a))
iMax = np.where(a == np.max(a))
a[iMin[0][0], iMin[1][0]] = -500
a[iMax[0][0], iMax[1][0]] = 500
b = handleOutliers(a, 1, 99, 0.2)
iMinB = np.where(b == np.min(b))
iMaxB = np.where(b == np.max(b))
print("CASE: negative + positive values")
print("Min Index Equal: ", iMin[0][0] == iMinB[0][0], iMin[1][0] == iMinB[1][0])
print("Max Index Equal: ",iMax[0][0] == iMaxB[0][0], iMax[1][0] == iMaxB[1][0])

# Test case 3 all values are positive
a = 10 * np.random.random((20,20))
a = a + 1.1 * np.max(a)

iMin = np.where(a == np.min(a))
iMax = np.where(a == np.max(a))
a[iMin[0][0], iMin[1][0]] = -500
a[iMax[0][0], iMax[1][0]] = 500
b = handleOutliers(a, 1, 99, 0.2)
iMinB = np.where(b == np.min(b))
iMaxB = np.where(b == np.max(b))
print("CASE: All values negative")
print("Min Index Equal: ", iMin[0][0] == iMinB[0][0], iMin[1][0] == iMinB[1][0])
print("Max Index Equal: ",iMax[0][0] == iMaxB[0][0], iMax[1][0] == iMaxB[1][0])
