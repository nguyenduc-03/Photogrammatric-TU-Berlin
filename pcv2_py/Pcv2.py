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
    pts = points[:, :2].astype(np.float64)  # (n,2)
    tx, ty = pts.mean(axis=0)
    sx = np.mean(np.abs(pts[:, 0] - tx))
    sy = np.mean(np.abs(pts[:, 1] - ty))
    T = np.array([
        [1.0 / sx, 0, -tx / sx],
        [0, 1.0 / sy, -ty / sy],
        [0, 0, 1.0]
    ], dtype=np.float64)
    return T


def getDesignMatrix_homography2D(conditioned_base, conditioned_attach):
    """
    Define the design matrix as needed to compute 2D-homography
    @param conditioned_base: first set of conditioned points x' --> x' = H * x
    @param conditioned_attach: second set of conditioned points x --> x' = H * x
    @return: the design matrix to be computed
    """
    # TO DO !!!
    n = conditioned_base.shape[0]
    A = np.zeros((2 * n, 9), dtype=np.float64)
    for i in range(n):
        u, v, w = conditioned_attach[i]
        up, vp, wp = conditioned_base[i]
        x = np.array([u, v, w])
        A[2 * i, 0:3] = -wp * x
        A[2 * i, 6:9] = up * x
        A[2 * i + 1, 3:6] = -wp * x
        A[2 * i + 1, 6:9] = vp * x
    return A


def solve_dlt_homography2D(A):
    """
    Solve homogeneous equation system by usage of SVD
    @param A: the design matrix
    @return: solution of the homogeneous equation system
    """
    # TO DO !!!
    _, _, Vt = np.linalg.svd(A)
    h = Vt[-1]
    H = h.reshape(3, 3)
    return H


def decondition_homography2D(T_base, T_attach, H):
    """
    Decondition a homography that was estimated from conditioned point clouds
    @param T_base: conditioning matrix T' of first set of points x'
    @param T_attach: conditioning matrix T of second set of points x
    @param H: conditioned homography that has to be un-conditioned (in-place)
    @return: deconditioned homography
    """
    # TO DO !!!
    T_base_inv = np.linalg.inv(T_base) # T'^{-1}
    H_dec = T_base_inv @ H @ T_attach # T'^{-1} · H̃ · T
    return H_dec


def homography2D(base, attach):
    """
    Compute the homography
    @param base: first set of points x'
    @param attach: second set of points x
    @return: homography H, so that x' = Hx
    """
    # TO DO !!!
    # 1. Compute conditioning matrices
    T_base = getCondition2D(base)
    T_attach = getCondition2D(attach)

    # 2. Condition the points
    base_c = (T_base @ base.T).T
    attach_c = (T_attach @ attach.T).T

    # 3. Design matrix
    A = getDesignMatrix_homography2D(base_c, attach_c)

    # 4. Solve using SVD
    H_tilde = solve_dlt_homography2D(A)

    # 5. Decondition
    H = decondition_homography2D(
        T_base, T_attach, H_tilde
    )

    # 6. Normalize
    H = H / H[2, 2]

    return H.astype(np.float32)


# Functions from exercise 1
# Reuse your solutions from the last exercise here
def eucl2hom_point_2D(p):
    return np.array([p[0], p[1], 1.0], dtype=np.float32)


def applyH_2D(geomObjects, H, gtype):
    result = []
    if gtype == GEOM_TYPE_POINT:
        for obj in geomObjects:
            result.append((H @ obj).astype(np.float32))
    elif gtype == GEOM_TYPE_LINE:
        H_inv_T = np.linalg.inv(H).T
        for obj in geomObjects:
            result.append((H_inv_T @ obj).astype(np.float32))
    else:
        raise RuntimeError("Unhandled geometry type!")
    return result

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
        for obj in geomObjects:
            result.append((H @ obj).astype(np.float32))
    elif gtype == GEOM_TYPE_LINE:
        H_inv_T = np.linalg.inv(H).T
        for obj in geomObjects:
            result.append((H_inv_T @ obj).astype(np.float32))
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
    return np.array([p[0], p[1], 1.0], dtype=np.float32)
