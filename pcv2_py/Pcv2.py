#============================================================================
# Name        : Pcv2.py
# Author      : Annamalai Lakshmanan
# Version     : 1.0
# Copyright   : -
# Description : Python implementation mirroring C++ Pcv2.cpp/Pcv2.h
#============================================================================

import numpy as np
import cv2 as cv

GEOM_TYPE_POINT = 0
GEOM_TYPE_LINE  = 1


def getCondition2D(points):
    """
    Get the conditioning matrix of given points
    @param points: the points as matrix
    @return: the condition matrix (already allocated)
    """
    # TO DO !!!
    return np.eye(3, dtype=np.float32)


def getDesignMatrix_homography2D(conditioned_base, conditioned_attach):
    """
    Define the design matrix as needed to compute 2D-homography
    @param conditioned_base: first set of conditioned points x' --> x' = H * x
    @param conditioned_attach: second set of conditioned points x --> x' = H * x
    @return: the design matrix to be computed
    """
    # TO DO !!!
    return np.zeros((8, 9), dtype=np.float32)


def solve_dlt_homography2D(A):
    """
    Solve homogeneous equation system by usage of SVD
    @param A: the design matrix
    @return: solution of the homogeneous equation system
    """
    # TO DO !!!
    return np.eye(3, dtype=np.float32)


def decondition_homography2D(T_base, T_attach, H):
    """
    Decondition a homography that was estimated from conditioned point clouds
    @param T_base: conditioning matrix T' of first set of points x'
    @param T_attach: conditioning matrix T of second set of points x
    @param H: conditioned homography that has to be un-conditioned (in-place)
    @return: deconditioned homography
    """
    # TO DO !!!
    return H


def homography2D(base, attach):
    """
    Compute the homography
    @param base: first set of points x'
    @param attach: second set of points x
    @return: homography H, so that x' = Hx
    """
    # TO DO !!!
    return np.eye(3, dtype=np.float32)




# Functions from exercise 1
# Reuse your solutions from the last exercise here

def applyH_2D(geomObjects, H, gtype):
    """
    Applies a 2D transformation to an array of points or lines
    @param geomObjects: Array of input objects, each in homogeneous coordinates
    @param H: Matrix representing the transformation
    @param gtype: The type of the geometric objects, point or line. All are the same type.
    @return: Array of transformed objects.
    """
    result = []
    
    #******* Small list/array cheat sheet ************************************/
    #
    #   Number of elements in array:                  len(a)
    #   Access i-th element (reading or writing):     a[i]
    #   Append an element to a list:                  a.append(element)
    #
    #**************************************************************************/

    # TO DO !!!

    if gtype == GEOM_TYPE_POINT:
        # TO DO !!!
        pass
    elif gtype == GEOM_TYPE_LINE:
        # TO DO !!!
        pass
    else:
        raise RuntimeError("Unhandled geometry type!")
    
    return result


def eucl2hom_point_2D(p):
    """
    Convert a 2D point from Euclidean to homogeneous coordinates
    @param p: The point to convert (in Euclidean coordinates)
    @return: The same point in homogeneous coordinates
    """
    # TO DO !!!
    return np.array([0.0, 0.0, 0.0], dtype=np.float32)
