#!/usr/bin/env python -W ignore::DeprecationWarning

"""
Utils for C106B grasp planning project.
Author: Chris Correa.
Adapted for Spring 2020 by Amay Saxena
"""
import numpy as np
import math
from scipy.spatial.transform import Rotation

def find_intersections(mesh, p1, p2):
    """
    Finds the points of intersection between an input mesh and the
    line segment connecting p1 and p2.

    Parameters
    ----------
    mesh (trimesh.base.Trimesh): mesh of the object
    p1 (3x np.ndarray): line segment point
    p2 (3x np.ndarray): line segment point

    Returns
    -------
    on_segment (2x3 np.ndarray): coordinates of the 2 intersection points
    faces (2x np.ndarray): mesh face numbers of the 2 intersection points
    """
    ray_origin = (p1 + p2) / 2
    ray_length = np.linalg.norm(p1 - p2)
    ray_dir = (p2 - p1) / ray_length
    locations, index_ray, index_tri = mesh.ray.intersects_location(
        ray_origins=[ray_origin, ray_origin],
        ray_directions=[ray_dir, -ray_dir],
        multiple_hits=True)
    if len(locations) <= 0:
        return [], None
    dist_to_center = np.linalg.norm(locations - ray_origin, axis=1)
    dist_mask = dist_to_center <= (ray_length / 2) # only keep intersections on the segment.
    on_segment = locations[dist_mask]
    faces = index_tri[dist_mask]
    return on_segment, faces

def find_grasp_vertices(mesh, p1, p2):
    """
    If the tips of an ideal two fingered gripper start off at
    p1 and p2 and then close, where will they make contact with the object?
    
    Parameters
    ----------
    mesh (trimesh.base.Trimesh): mesh of the object
    p1 (3x np.ndarray): starting gripper point
    p2 (3x np.ndarray): starting gripper point

    Returns
    -------
    locations (nx3 np.ndarray): coordinates of the closed gripper's n contact points
    face_ind (nx np.ndarray): mesh face numbers of the closed gripper's n contact points
    """
    ray_dir = p2 - p1
    locations, index_ray, face_ind = mesh.ray.intersects_location(
        ray_origins=[p1, p2],
        ray_directions=[p2 - p1, p1 - p2],
        multiple_hits=False)
    return locations, face_ind

def normalize(vec):
    """
    Returns a normalized version of a numpy vector

    Parameters
    ----------
    vec (nx np.ndarray): vector to normalize

    Returns
    -------
    (nx np.ndarray): normalized vector
    """
    return vec / np.linalg.norm(vec)

def length(vec):
    """
    Returns the length of a 1 dimensional numpy vector

    Parameters
    ----------
    vec : nx1 :obj:`numpy.ndarray`

    Returns
    -------
    float
        ||vec||_2^2
    """
    return np.sqrt(vec.dot(vec))

def vec(*args):
    """
    all purpose function to get a numpy array of random things.  you can pass
    in a list, tuple, ROS Point message.  you can also pass in:
    vec(1,2,3,4,5,6) which will return a numpy array of each of the elements 
    passed in: np.array([1,2,3,4,5,6])
    """
    if len(args) == 1:
        if type(args[0]) == tuple:
            return np.array(args[0])
        elif ros_enabled and type(args[0]) == Point:
            return np.array((args[0].x, args[0].y, args[0].z))
        else:
            return np.array(args)
    else:
        return np.array(args)

def hat(v):
    """
    See https://en.wikipedia.org/wiki/Hat_operator or the MLS book

    Parameters
    ----------
    v (3x, 3x1, 6x, or 6x1 np.ndarray): vector to create hat matrix for

    Returns
    -------
    (3x3 or 6x6 np.ndarray): the hat version of the v
    """
    if v.shape == (3, 1) or v.shape == (3,):
        return np.array([
                [0, -v[2], v[1]],
                [v[2], 0, -v[0]],
                [-v[1], v[0], 0]
            ])
    elif v.shape == (6, 1) or v.shape == (6,):
        return np.array([
                [0, -v[5], v[4], v[0]],
                [v[5], 0, -v[3], v[1]],
                [-v[4], v[3], 0, v[2]],
                [0, 0, 0, 0]
            ])
    else:
        raise ValueError

def adj(g):
    """
    Adjoint of a rotation matrix. See the MLS book.

    Parameters
    ----------
    g (4x4 np.ndarray): homogenous transform matrix

    Returns
    -------
    (6x6 np.ndarray): adjoint matrix
    """
    if g.shape != (4, 4):
        raise ValueError

    R = g[0:3,0:3]
    p = g[0:3,3]
    result = np.zeros((6, 6))
    result[0:3,0:3] = R
    result[0:3,3:6] = np.matmul(hat(p), R)
    result[3:6,3:6] = R
    return result

def look_at_general(origin, direction):
    """
    Creates a homogenous transformation matrix at the origin such that the 
    z axis is the same as the direction specified. There are infinitely 
    many of such matrices, but we choose the one where the y axis is as 
    vertical as possible.  

    Parameters
    ----------
    origin (3x np.ndarray): origin coordinates
    direction (3x np.ndarray): direction vector

    Returns
    -------
    (4x4 np.ndarray): homogenous transform matrix
    """
    up = np.array([0, 0, 1])
    z = normalize(direction) # create a z vector in the given direction
    x = normalize(np.cross(up, z)) # create a x vector perpendicular to z and up
    y = np.cross(z, x) # create a y vector perpendicular to z and x

    result = np.eye(4)

    # set rotation part of matrix
    result[0:3,0] = x
    result[0:3,1] = y
    result[0:3,2] = z

    # set translation part of matrix to origin
    result[0:3,3] = origin

    return result

def create_transform_matrix(rotation_matrix, translation_vector):
    """
    Creates a homogenous 4x4 matrix representation of this transform

    Parameters
    ----------
    rotation_matrix (3x3 np.ndarray): Rotation between two frames
    translation_vector (3x np.ndarray): Translation between two frames

    """
    return np.r_[np.c_[rotation_matrix, translation_vector],[[0, 0, 0, 1]]]

def rotation_from_quaternion(q_xyzw):   #original
    """Convert quaternion array to rotation matrix.
    Parameters
    ----------
    q_wxyz : :obj:`numpy.ndarray` of float
        A quaternion in wxyz order.
    Returns
    -------
    :obj:`numpy.ndarray` of float
        A 3x3 rotation matrix made from the quaternion.
    """
    # The starter code line below is broken (assumes input is wxyz, when it is already xyzw)
    # Changes, commented out line below, changed fn arg to q_xyzw
    # q_xyzw = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]])

    r = Rotation.from_quat(q_xyzw)
    try:
        mat = r.as_dcm()
    except:
        mat = r.as_matrix()
    return mat

def rotationFromQuaternion(Q):   #Apr 26th update
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
    This rotation matrix converts a point in the local reference 
    frame to a point in the global reference frame.
    """
    # Extract the values from Q
    # q0 = Q[0]
    # q1 = Q[1]
    # q2 = Q[2]
    # q3 = Q[3]
    q0 = Q[3]
    q1 = Q[0]
    q2 = Q[1]
    q3 = Q[2]
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],[r10, r11, r12],
[r20, r21, r22]])
                            
    return rot_matrix

def quaternion_from_matrix(matrix):   #original
    """Return quaternion from rotation matrix.
    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
    True
    """
    try:
        r = Rotation.from_dcm(matrix)
    except:
        r = Rotation.from_matrix(matrix)
    return r.as_quat()

def quaternionFromMatrix(matrix, isprecise=False): #Apr 26th update
    """Return quaternion from rotation matrix.

    If isprecise is True, the input matrix is assumed to be a precise rotation
    matrix and a faster algorithm is used.

    >>> q = quaternion_from_matrix(numpy.identity(4), True)
    >>> numpy.allclose(q, [1, 0, 0, 0])
    True
    >>> q = quaternion_from_matrix(numpy.diag([1, -1, -1, 1]))
    >>> numpy.allclose(q, [0, 1, 0, 0]) or numpy.allclose(q, [0, -1, 0, 0])
    True
    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R, True)
    >>> numpy.allclose(q, [0.9981095, 0.0164262, 0.0328524, 0.0492786])
    True
    >>> R = [[-0.545, 0.797, 0.260, 0], [0.733, 0.603, -0.313, 0],
    ...      [-0.407, 0.021, -0.913, 0], [0, 0, 0, 1]]
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.19069, 0.43736, 0.87485, -0.083611])
    True
    >>> R = [[0.395, 0.362, 0.843, 0], [-0.626, 0.796, -0.056, 0],
    ...      [-0.677, -0.498, 0.529, 0], [0, 0, 0, 1]]
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.82336615, -0.13610694, 0.46344705, -0.29792603])
    True
    >>> R = random_rotation_matrix()
    >>> q = quaternion_from_matrix(R)
    >>> is_same_transform(R, quaternion_matrix(q))
    True

    """
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    if isprecise:
        q = np.empty((4,))
        t = np.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 1, 2, 3
            if M[1, 1] > M[0, 0]:
                i, j, k = 2, 3, 1
            if M[2, 2] > M[i, i]:
                i, j, k = 3, 1, 2
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = np.array(
            [
                [m00 - m11 - m22, 0.0, 0.0, 0.0],
                [m01 + m10, m11 - m00 - m22, 0.0, 0.0],
                [m02 + m20, m12 + m21, m22 - m00 - m11, 0.0],
                [m21 - m12, m02 - m20, m10 - m01, m00 + m11 + m22],
            ]
        )
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = np.linalg.eigh(K)
        q = V[[3, 0, 1, 2], np.argmax(w)]
    if q[0] < 0.0:
        np.negative(q, q)
    return q

def skew_3d(omega):
    """
    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.
    
    Args:
    omega - (3,) ndarray: the rotation vector
    
    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
    """

    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')

    omega_hat = np.zeros((3,3))
    omega_hat[0,1] = -omega[2]
    omega_hat[0,2] = omega[1]
    omega_hat[1,0] = omega[2]
    omega_hat[1,2] = -omega[0]
    omega_hat[2,0] = -omega[1]
    omega_hat[2,1] = omega[0]

    return omega_hat

def hat_3d(xi):
    """
    Converts a 3D twist to its corresponding 4x4 matrix representation
    
    Args:
    xi - (6,) ndarray: the 3D twist
    
    Returns:
    xi_hat - (4,4) ndarray: the corresponding 4x4 matrix
    """

    matrix = np.zeros((4, 4))
    matrix[0:3,0:3] = skew_3d(xi[3:6])
    matrix[0:3,3] = xi[0:3]
    return matrix