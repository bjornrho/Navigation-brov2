import math


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