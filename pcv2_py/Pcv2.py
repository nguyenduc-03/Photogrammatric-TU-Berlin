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
    points = np.asarray(points, dtype=np.float32)
    mean_x = np.mean(points[:, 0])
    mean_y = np.mean(points[:, 1])
    sigma_x = np.sum(np.abs(points[:, 0] - mean_x)) / len(points)
    sigma_y = np.sum(np.abs(points[:, 1] - mean_y)) / len(points)
    sigma_x = 1.0 if sigma_x == 0 else sigma_x
    sigma_y = 1.0 if sigma_y == 0 else sigma_y
    return np.array([
        [1.0 / sigma_x, 0.0, -mean_x / sigma_x],
        [0.0, 1.0 / sigma_y, -mean_y / sigma_y],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)


def getDesignMatrix_homography2D(conditioned_base, conditioned_attach):
    """
    Define the design matrix as needed to compute 2D-homography
    @param conditioned_base: first set of conditioned points x' --> x' = H * x
    @param conditioned_attach: second set of conditioned points x --> x' = H * x
    @return: the design matrix to be computed
    """
    n = len(conditioned_base)
    A = np.zeros((2 * n, 9), dtype=np.float32)
    for i in range(n):
        x = conditioned_attach[i]
        x_ = conditioned_base[i]
        A[2 * i] = [-x[0], -x[1], -1.0, 0.0, 0.0, 0.0, x[0] * x_[0], x[1] * x_[0], x_[0]]
        A[2 * i + 1] = [0.0, 0.0, 0.0, -x[0], -x[1], -1.0, x[0] * x_[1], x[1] * x_[1], x_[1]]
    return A


def solve_dlt_homography2D(A):
    """
    Solve homogeneous equation system by usage of SVD
    @param A: the design matrix
    @return: solution of the homogeneous equation system
    """
    _, _, Vh = np.linalg.svd(A)
    h = Vh[-1]
    return h.reshape((3, 3), order='C').astype(np.float32)


def decondition_homography2D(T_base, T_attach, H):
    """
    Decondition a homography that was estimated from conditioned point clouds
    @param T_base: conditioning matrix T' of first set of points x'
    @param T_attach: conditioning matrix T of second set of points x
    @param H: conditioned homography that has to be un-conditioned (in-place)
    @return: deconditioned homography
    """
    return np.linalg.inv(T_base).astype(np.float32) @ H @ T_attach.astype(np.float32)


def homography2D(base, attach):
    """
    Compute the homography
    @param base: first set of points x'
    @param attach: second set of points x
    @return: homography H, so that x' = Hx
    """
    base = np.asarray(base, dtype=np.float32)
    attach = np.asarray(attach, dtype=np.float32)
    T_base = getCondition2D(base)
    T_attach = getCondition2D(attach)
    conditioned_base = (base @ T_base.T).astype(np.float32)
    conditioned_attach = (attach @ T_attach.T).astype(np.float32)
    H_conditioned = solve_dlt_homography2D(getDesignMatrix_homography2D(conditioned_base, conditioned_attach))
    return decondition_homography2D(T_base, T_attach, H_conditioned)




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

    if gtype == GEOM_TYPE_POINT:
        for point in geomObjects:
            result.append(H @ point)
    elif gtype == GEOM_TYPE_LINE:
        for line in geomObjects:
            result.append(np.linalg.inv(H).T @ line)
    else:
        raise RuntimeError("Unhandled geometry type!")

    return result


def eucl2hom_point_2D(p):
    """
    Convert a 2D point from Euclidean to homogeneous coordinates
    @param p: The point to convert (in Euclidean coordinates)
    @return: The same point in homogeneous coordinates
    """
    return np.array([p[0], p[1], 1], dtype=np.float32)
