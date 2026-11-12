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
    pts = np.asarray(points, dtype=np.float64)

    # homogeneous -> Euclidean
    x = pts[:, 0] / pts[:, 2]
    y = pts[:, 1] / pts[:, 2]

    # centroid
    mx, my = x.mean(), y.mean()

    # scale: mean absolute deviation from the centroid, per axis
    sx = np.mean(np.abs(x - mx))
    sy = np.mean(np.abs(y - my))

    # x_cond = (x - mx) / sx  ->  scale and translation in one matrix
    return np.array([[1.0 / sx, 0.0,       -mx / sx],
                     [0.0,       1.0 / sy, -my / sy],
                     [0.0,       0.0,       1.0]])


def getDesignMatrix_homography2D(conditioned_base, conditioned_attach):
    """
    Define the design matrix as needed to compute 2D-homography
    @param conditioned_base: first set of conditioned points x' --> x' = H * x
    @param conditioned_attach: second set of conditioned points x --> x' = H * x
    @return: the design matrix to be computed
    """
    base = np.asarray(conditioned_base, dtype=np.float64)
    attach = np.asarray(conditioned_attach, dtype=np.float64)
    n = len(base)

    # two rows per point pair, 9 columns (the 9 entries of H)
    A = np.zeros((2 * n, 9), dtype=np.float64)

    for i in range(n):
        # base point x' = (xp, yp) in Euclidean form
        xp = base[i, 0] / base[i, 2]
        yp = base[i, 1] / base[i, 2]
        # attach point x = (x, y, 1) in homogeneous form
        X = attach[i] / attach[i, 2]

        # row for the x'-equation:  [-X, 0, xp * X]
        A[2 * i, 0:3] = -X
        A[2 * i, 6:9] = xp * X

        # row for the y'-equation:  [0, -X, yp * X]
        A[2 * i + 1, 3:6] = -X
        A[2 * i + 1, 6:9] = yp * X

    return A


def solve_dlt_homography2D(A):
    """
    Solve homogeneous equation system by usage of SVD
    @param A: the design matrix
    @return: solution of the homogeneous equation system
    """
    A = np.asarray(A, dtype=np.float64)

    # SVD: A = U * diag(S) * Vt, singular values in S sorted descending
    _, _, Vt = np.linalg.svd(A)

    # last row of Vt = singular vector of the smallest singular value
    h = Vt[-1]

    # 9-vector -> 3x3 matrix (row by row)
    return h.reshape(3, 3)


def decondition_homography2D(T_base, T_attach, H):
    """
    Decondition a homography that was estimated from conditioned point clouds
    @param T_base: conditioning matrix T' of first set of points x'
    @param T_attach: conditioning matrix T of second set of points x
    @param H: conditioned homography that has to be un-conditioned (in-place)
    @return: deconditioned homography
    """
    # H~ maps conditioned points: T' x' = H~ (T x)  ->  x' = T'^-1 H~ T x
    return np.linalg.inv(T_base) @ H @ T_attach


def homography2D(base, attach):
    """
    Compute the homography
    @param base: first set of points x'
    @param attach: second set of points x
    @return: homography H, so that x' = Hx
    """
    base = np.asarray(base, dtype=np.float64)
    attach = np.asarray(attach, dtype=np.float64)

    # 1. conditioning matrices of both point sets
    T_base = getCondition2D(base)
    T_attach = getCondition2D(attach)

    # 2. condition the points (points are rows, so transpose twice)
    cond_base = (T_base @ base.T).T
    cond_attach = (T_attach @ attach.T).T

    # 3. design matrix, 4. SVD solution
    A = getDesignMatrix_homography2D(cond_base, cond_attach)
    H_cond = solve_dlt_homography2D(A)

    # 5. undo the conditioning
    return decondition_homography2D(T_base, T_attach, H_cond)




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
            for p in geomObjects:
                result.append(H @ p) #Matrix multiplication (point rule)
            pass
    elif gtype == GEOM_TYPE_LINE:
            H_line = np.linalg.inv(H).T #Inverse transpose of H
            for l in geomObjects:
                result.append(H_line @ l) #Matrix multiplication (line rule)
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
    return np.array([p[0], p[1], 1.0], dtype=np.float32)
