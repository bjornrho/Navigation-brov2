import math
import numpy as np
from numpy.linalg import norm

def yaw_from_quaternion(quaternion):
    """Returns yaw (Euler angle - rotation around z counterclockwise) in radians.

    Args:
        quaternion       (4,1 ndarray) : Quaternion of form [w,x,y,z]


    Returns:
        yaw_z            (4,1 ndarray) : Normalized quaternion
        """

    q_w,q_x,q_y,q_z = quaternion.T[0]

    t0 = +2.0 * (q_w * q_z + q_x * q_y)
    t1 = +1.0 - 2.0 * (q_y * q_y + q_z * q_z)
    yaw_z = math.atan2(t0, t1)

    return yaw_z

def cross(x):
    """Moves a 3 vector into so(3)

    Args:
        x (3 ndarray) : Vector parametrization

    Returns:
        x (3,3 ndarray) : Element of so(3)
        """
        
    x0, x1, x2 = x[:]

    return np.array([[0.0,   -x2[0],  x1[0]],
                    [ x2[0],  0.0,   -x0[0]],
                    [-x1[0],  x0[0],   0.0]])

def rodrigues(omega, phi):
    """Returns rotation matrix defined by rotation vector phi through the exponential map equation (78)

    Args:
        omega (3 ndarray) : Vector element of rotation vector
        phi                : Scalar element of rotation vector


    Returns:
        R (3,3 ndarray) : Rotation matrix
        """
    if phi == 0:
        return np.eye(3)

    omega = omega/norm(omega)
    #R = np.eye(3)*np.cos(phi) + self.cross(omega)*np.sin(phi) + omega@omega.T*(1-np.cos(phi))
    R = np.eye(3) + np.sin(phi)*cross(omega) + (1-np.cos(phi))*(omega@omega.T - np.eye(3))
    return R

def quaternion_to_rotation_matrix(quaternion):
    """Returns rotation matrix defined by quaternion following equation (117)

    Args:
        quaternion (4,1 ndarray) : quaternion of form [w,x,y,z]


    Returns:
        R          (3,3 ndarray) : Rotation matrix
        """        
    q_v = quaternion[1:]
    q_w = quaternion[0,0]

    R = (q_w**2 - (q_v.T@q_v)[0,0])*np.eye(3) + 2*q_v@q_v.T + 2*q_w*cross(q_v)
    return R

def rotation_vector_to_quaternion(u, phi):
    """Returns normalized quaternion defined by rotation vector following equation (101)

    Args:
        u       (3,1 ndarray) : Vector element of rotation vector
        phi                   : Scalar element of rotation vector


    Returns:
        q       (4,1 ndarray) : Normalized quaternion
        """
    if norm(u) == 0:
        return np.array([[1.0], [0.0], [0.0], [0.0]])


    q_w = np.cos(phi/2)
    q_v = (u/norm(u)) * np.sin(phi/2)
    q = np.block([[q_w],[q_v]])

    q = q/norm(q)
    return q

def quaternion_product(p, q):
    """Returns normalized quaternion product of two quaternions following equation (12)

    Args:
        p       (4,1 ndarray) : Quaternion of form [w,x,y,z]
        q       (4,1 ndarray) : Quaternion of form [w,x,y,z]


    Returns:
        qproduct       (4,1 ndarray) : Normalized quaternion
        """
        
    p_w,p_x,p_y,p_z = p.T[0]
    q_w,q_x,q_y,q_z = q.T[0]

    qproduct = np.array([[p_w*q_w - p_x*q_x - p_y*q_y - p_z*q_z],
                         [p_w*q_x + p_x*q_w + p_y*q_z - p_z*q_y],
                         [p_w*q_y - p_x*q_z + p_y*q_w + p_z*q_x],
                         [p_w*q_z + p_x*q_y - p_y*q_x + p_z*q_w]])

    qproduct = qproduct/norm(qproduct)
    return qproduct