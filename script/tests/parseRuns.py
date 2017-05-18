#/usr/bin/env python

# Parser script for run tests results
# Average and Variance of parsed values :
# path durations, number of 'new' nodes, computation time

"""
from parseRuns import main
main()
"""

from __future__ import division
import numpy as np
import math

runFile = "/local/mcampana/devel/hpp/src/hpp-rbprm-corba/script/tests/"

# --------------------------------------------------------------------#
# Read data and write / display final results

def main (fileName):
    prefixNbWaypoints = "nbWaypoints: "
    prefixNbParabolas = "nbParabolas: "
    prefixNbCentroidalFails = "nbCentroidalFails: "
    prefixNbFailsLimbRRT = "nbFailsLimbRRT: "
    #
    ## Get data:
    nbWaypoints_vector = parseRun (fileName, prefixNbWaypoints)
    nbParabolas_vector = parseRun (fileName, prefixNbParabolas)
    nbCentroidalFails_vector = parseRun (fileName, prefixNbCentroidalFails)
    nbFailsLimbRRT_vector = parseRun (fileName, prefixNbFailsLimbRRT)
    nbWaypoints_total = sum (nbWaypoints_vector)
    nbParabolas_total = sum (nbParabolas_vector)
    nbCentroidalFails_total = sum (nbCentroidalFails_vector)
    nbFailsLimbRRT_total = sum (nbFailsLimbRRT_vector)
    #
    nbSequencesWithCentroidalFail = compteNbSequencesNonZero (nbCentroidalFails_vector)
    nbSequencesWithLimbRRTFail = compteNbSequencesNonZero (nbFailsLimbRRT_vector)
    nbCentroidalFails_ratio = nbCentroidalFails_total / nbWaypoints_total
    nbFailsLimbRRT_ratio = nbFailsLimbRRT_total / nbParabolas_total
    #
    # Display
    print ("number of data: " + str(len(nbWaypoints_vector)))
    print ("nbWaypoints_total: " + str(nbWaypoints_total))
    print ("nbParabolas_total: " + str(nbParabolas_total))
    print ("nbCentroidalFails_total: " + str(nbCentroidalFails_total))
    print ("nbFailsLimbRRT_total: " + str(nbFailsLimbRRT_total))
    print ("nbSequencesWithCentroidalFail: " + str(nbSequencesWithCentroidalFail))
    print ("nbCentroidalFails_ratio: " + str(nbCentroidalFails_ratio))
    print ("nbSequencesWithLimbRRTFail: " + str(nbSequencesWithLimbRRTFail))
    print ("nbFailsLimbRRT_ratio: " + str(nbFailsLimbRRT_ratio))
    #
    # Save
    """f = open(runFile + fileName,'a')
    f.write('nbWaypoints_total: '+str(nbWaypoints_total)+'\n')
    f.write('nbParabolas_total: '+str(nbParabolas_total)+'\n')
    f.write('nbCentroidalFails_total: '+str(nbCentroidalFails_total)+'\n')
    f.write('nbFailsLimbRRT_total: '+str(nbFailsLimbRRT_total)+'\n')
    f.write('nbSequencesWithCentroidalFail: '+str(nbSequencesWithCentroidalFail)+'\n')
    f.write('nbSequencesWithLimbRRTFail: '+str(nbSequencesWithLimbRRTFail)+'\n')
    f.write('nbCentroidalFails_ratio: '+str(nbCentroidalFails_ratio)+'\n')
    f.write('nbFailsLimbRRT_ratio: '+str(nbFailsLimbRRT_ratio)+'\n')
    f.close()"""

# --------------------------------------------------------------------#

def parseRun (fileName, prefix):
    l = len (prefix)
    with open (runFile + fileName) as f:
        vector = []
        for line in f:
            if line [:l] == prefix :
                elem = line [l:]
                st = elem.strip ('\n') # remove end characters
                try:
                    #config = map (float, elem) # convert into float
                    vector.append (float(st))
                except:
                    print "catched"
    return np.array (vector)

# --------------------------------------------------------------------#

def compteNbSequencesNonZero (vector):
    l = len (vector)
    result = 0
    for i in range (0,l):
        if (vector [i] > 0):
            result = result + 1
	return result

# --------------------------------------------------------------------#

# Compute average and SD for the given prefix indicating a vector of floats in 'results.txt'
def computeTotal (prefix):
    vector = parseRun (prefix)
    print "Number of data: " + str(len (vector))
    print vector
    print "length: " + str(len(vector))
    #print "SD: "+str(math.sqrt(variance (vector)))+"\n"
    return sum (vector)

# --------------------------------------------------------------------#

# Compute average of a vector
def average (vector):
    av=0
    for elem in vector:
        av += elem
    av = av/len(vector)
    print "average: "+str(av)
    return av

# --------------------------------------------------------------------#

# Compute variance of a vector (Konig-Huygens formula)
def variance (vector):
    av = average (vector)
    var=0
    for elem in vector:
        var += elem**2
    var = var/len(vector)
    var = var - av**2
    if var<0:
        print "Variance is <0 because of rounding errors: "+str(var)
        var=0
    return var




