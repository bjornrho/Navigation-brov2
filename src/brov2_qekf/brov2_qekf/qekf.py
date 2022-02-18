import numpy as np
from numpy.linalg import inv
from scipy.spatial.transform import Rotation


# Implementation of the Quaternion based Extended Kalman Filter from Joan Solà's article: https://arxiv.org/abs/1711.02508
# and the qekf implementation from FRoSt Lab available at https://bitbucket.org/frostlab/underwateriekf/src/main/
class QEKF:
    def __init__(self, x_0, dx_0, P_0, std_a, std_gyro, std_dvl, std_depth, std_a_bias, std_gyro_bias, dvl_offset):
        """Initialization of the QEKF

        Args:
            x_0           (19 list) : Initial starting point of nominal state
            dx_0        (18 ndarray) : Initial starting point of error-state  
            P_0        (18x18 array) : Initial covariance of error-state
            std_a                    : Accelerometer Gaussian noise
            std_gyro                 : Gyrometer Gaussian noise
            std_dvl                  : Dvl Gaussian noise
            std_depth                : Pressure sensor Gaussian noise
            std_a_bias               : Accelerometer bias
            std_gyro_bias            : Gyrometer bias
            dvl_offset   (3 ndarray) : Offset value for DVL placement on ROV
            """
        
        # Initializing nominal state, error-state (mean zero) and covariance
        self.x = np.array(x_0).reshape(-1,1)
        self.dx = dx_0
        self.P = P_0

        # Initializing uncertainty terms (must be retrieved from parameters.yaml)
        self.std_gyro = std_gyro
        self.std_a = std_a
        self.std_dvl = std_dvl
        self.std_depth = std_depth
        self.std_gyro_bias = std_gyro_bias
        self.std_a_bias = std_a_bias

        # Defining offset values and previous IMU input (to be used accounting for offset)
        self.dvl_offset = np.array(dvl_offset).reshape(-1,1)
        self.last_u = np.zeros((2,3))



    def integrate(self, u, dt):
        """Integrates measurement into nominal state

        Args:
            u     (2,3 ndarray) : Vector parametrization
            dt                  : Time step between consecutive IMU measurements

        Returns:
            x     (k+1 ndarray) : Nominal state"""

        p = self.x[0:3]
        v = self.x[3:6]
        q = self.x[6:10]
        a_b = self.x[10:13]
        omega_b = self.x[13:16]
        g = self.x[16:]
        
        a_m = u[0].reshape(-1,1)
        omega_m = u[1].reshape(-1,1)
        
        R_q = self.quaternion_to_rotation_matrix(self, q)
        q_rot = self.rotation_vector_to_quaternion((omega_m - omega_b), dt)


        # Integrating IMU measurements into nominal state following equation (260), where biases and g doesn't change
        self.x[0:3] += v*dt + (1/2)*(R_q@(a_m - a_b) + g)*(dt**2)
        self.x[3:6] += (R_q@(a_m - a_b) + g)*dt
        self.x[6:10] = self.quaternion_product(q,q_rot)

        return self.x



    def predict(self, u, dt):
        """Runs prediction step of QEKF

        Args:
            u     (2,3 ndarray) : Control taken at this step
            u[0]                : Acceleration
            u[1]                : Angular velocity
            dt                  : Time step between consecutive IMU measurements

        Returns:
            dx_hat  (k ndarray) : Propagated error-state
            P_hat   (mxm array) : Propagated covariances
            """
        
        #Propagate the state using equation (268)
        g = np.array([0, 0, 9.81])
        p = self.dx[:3]
        v = self.dx[3:6]
        R = self.dx[6:15].reshape((3,3))
        zero    = np.zeros((3,3))
        I       = np.eye(3)

        omega = u[1] - self.std_gyro_bias
        a = u[0] - self.std_a_bias

        # Jacobian wrt. the error following equation (270)
        F_x = np.block([[I,     I*dt,   zero,                           zero,   zero,   zero],
                        [zero,  I,      -R@self.cross(a)*dt,            -R*dt,  zero,   zero],
                        [zero,  zero,   self.rodrigues(self,omega,dt).T,     zero,   -I*dt,  zero],
                        [zero,  zero,   zero,                           I,      zero,   zero],
                        [zero,  zero,   zero,                           zero,   I,      zero],
                        [zero,  zero,   zero,                           zero,   zero,   I]])
        
        dx_hat = F_x@self.dx #Should apparently be skipped due to mean initialized to zero. Let it be for now.

        #Propagate the uncertainty using equation (269)
        std_a = self.std_a
        std_g = self.std_gyro
        std_ab = self.std_a_bias
        std_gb = self.std_gyro_bias

        Q_i = dt*np.diag([std_a,std_a,std_a,std_g,std_g,std_g,std_ab,std_ab,std_ab,std_gb,std_gb,std_gb])**2
        Q_i[:6,:6] = Q_i[:6,:6]*dt

        # Jacobian wrt. the perturbation vectors following equation (271)
        F_i = np.zeros((18,12))
        F_i[3:15] = np.eye(12)

        P_hat = F_x@self.P@F_x.T + F_i@Q_i@F_i.T

        self.dx = dx_hat
        self.P = P_hat
        self.last_u = u

        return dx_hat, P_hat



    def update_depth(self, depth_measurement):
        """Runs correction step of QEKF

        Args:
            depth_measurement   : Incoming depth measurement from the pressure sensor (corresponding to dx[2])                       
        Returns:
            dx_hat  (19x1 ndarray) : Corrected state
            P_hat    (18x18 array) : Corrected covariances
            """

        # Defining the Jacobian H and the depth covariance V
        H_x = np.zeros((1,19))
        H_x[0,2] = 1
        V = self.std_depth**2
        
        q_w,q_x,q_y,q_z = self.x[6:10].T[0]
        Q_dtheta = (1/2)*np.array([[-q_x,   -q_y,   -q_z],
                                   [q_w,    -q_z,   q_y],
                                   [q_z,    q_w,    -q_x],
                                   [-q_y,   q_x,    q_w]]) # Equation (281)
        
        # Forming X_dx following equation (280)
        X_dx = np.zeros((19,18))
        X_dx[:6,:6] = np.eye(6)
        X_dx[6:10,6:9] = Q_dtheta
        X_dx[10:19,9:18] = np.eye(9)
        
        H = H_x@X_dx

        innovation = depth_measurement - self.dx[2]

        # Defining the Kalman gain
        K = self.P@H.T*inv(H@self.P@H.T + V)
        
        # Correcting state and covariance according to equations (275) and (276)
        dx_hat = K*innovation
        P_hat = (np.eye(18)-K@H)@self.P

        self.dx = dx_hat
        self.P = P_hat
        
        return dx_hat, P_hat



    def update_dvl(self, dvl_measurement, dvl_covariance=None):
        """Runs correction step of QEKF

        Args:
            dvl_measurement     (3 ndarray) : Incoming velocity measurement from the dvl corresponding to dx[3:6]
            dvl_covariance    (3x3 ndarray) : Covariance matrix accompanying DVL measurement, V in equation (274)

        Returns:
            dx_hat  (19x1 ndarray)          : Corrected state
            P_hat    (18x18 array)          : Corrected covariances
            """

        if dvl_covariance is None:
            dvl_covariance = (np.eye(3)*self.std_dvl)**2

        # Account for the DVL offset
        omega_m = self.last_u[1].reshape(-1,1)
        omega_b = self.x[13:16]
        z = dvl_measurement + self.dvl_offset*(omega_m - omega_b)

        # Defining the Jacobian H and the DVL covariance V
        H_x = np.zeros((3,19))
        H_x[:,3:6] = np.eye(3)

        q_w,q_x,q_y,q_z = self.x[6:10].T[0]
        Q_dtheta = (1/2)*np.array([[-q_x,   -q_y,   -q_z],
                                   [q_w,    -q_z,   q_y],
                                   [q_z,    q_w,    -q_x],
                                   [-q_y,   q_x,    q_w]]) # Equation (281)
        
        # Forming X_dx following equation (280)
        X_dx = np.zeros((19,18))
        X_dx[:6,:6] = np.eye(6)
        X_dx[6:10,6:9] = Q_dtheta
        X_dx[10:19,9:18] = np.eye(9)
        
        H = H_x@X_dx

        # Rotation matrix
        R = self.quaternion_to_rotation_matrix(self, self.x[6:10])

        innovation = z - R.T@self.x[3:6] # NOT SURE ABOUT FRAMES HERE?

        # Defining the Kalman gain
        K = self.P@H.T@inv(H@self.P@H.T + dvl_covariance)

        # Correcting state and covariance according to equations (275) and (276)
        dx_hat = K@innovation
        P_hat = (np.eye(18)-K@H)@self.P

        self.dx = dx_hat
        self.P = P_hat
        
        return dx_hat, P_hat



    def inject(self):
        """Runs injection step of QEKF following equation (282)

        Args:
            dt                      : Time step between what???


        Returns:
            x       (k+1 ndarray)   : Updated nominal state
            """

        q_dtheta = Rotation.from_euler('xyz', self.dx[6:9].T[0]).as_quat()
        q_dtheta[0], q_dtheta[-1] = q_dtheta[-1], q_dtheta[0]
        
        self.x[0:3] += self.dx[0:3]
        self.x[3:6] += self.dx[3:6]
        self.x[6:10] = self.quaternion_product(self.x[6:10], np.array([q_dtheta]).T)
        self.x[10:13] += self.dx[9:12]
        self.x[13:16] += self.dx[12:15]
        self.x[16:] += self.dx[15:]

        return self.x



    def reset(self):
        """Runs reset step of QEKF

        Args:
            dt                  : Time step between what???


        Returns:
            dx_hat  (k ndarray) : Reset error-state
            P_hat   (mxm array) : Reset covariances
            """

        d_theta = self.dx[6:9]
        # Reset the error-state estimate following equation (285)
        self.dx = np.zeros((18,1))

        # Form G and reset the covariances following equation (286)
        G = np.zeros((18,18))
        G[:6,:6] = np.eye(6)
        G[6:9,6:9] = np.eye(3) - self.cross((1/2)*d_theta)
        G[9:18,9:18] = np.eye(9)

        self.P = G@self.P@G.T

        return self.dx, self.P





    @staticmethod
    def cross(x):
        """Moves a 3 vector into so(3)

        Args:
            x (3 ndarray) : Vector parametrization

        Returns:
            x (3,3 ndarray) : Element of so(3)
            """

        return np.array([[   0, -x[2],  x[1]],
                        [ x[2],     0, -x[0]],
                        [-x[1],  x[0],     0]])

    @staticmethod
    def rodrigues(self, omega, dt):
        """Returns rotation matrix defined by rotation vector phi through the exponential map equation (78)

        Args:
            omega (3 ndarray) : Vector element of rotation vector
            dt                : Scalar element of rotation vector


        Returns:
            rot_mat (3,3 ndarray) : Rotation matrix
            """

        R = np.eye(3)*np.cos(dt) + self.cross(omega)*np.sin(dt) + omega@omega.T*(1-np.cos(dt))
        return R

    @staticmethod
    def quaternion_to_rotation_matrix(self, quaternion):
        """Returns rotation matrix defined by quaternion following equation (117)

        Args:
            quaternion (4,1 ndarray) : quaternion of form [w,x,y,z]


        Returns:
            R          (3,3 ndarray) : Rotation matrix
            """
      
        q_v = quaternion[1:]
        q_w = quaternion[0][0]
        R = (q_w**2 - (q_v.T@q_v)[0][0])*np.eye(3) + 2*q_v@q_v.T + 2*q_w*self.cross(q_v.T[0])
        return R

    @staticmethod
    def rotation_vector_to_quaternion(u, phi):
        """Returns quaternion defined by rotation vector following equation (101)

        Args:
            u         (3 ndarray) : Vector element of rotation vector
            phi                   : Scalar element of rotation vector


        Returns:
            q       (4,1 ndarray) : Quaternion
            """

        q_v = u * np.sin(phi/2)
        q_w = np.cos(phi/2)
        q = np.block([[q_w],[q_v]])
        return q


    @staticmethod
    def quaternion_product(p, q):
        """Returns quaternion product of two quaternions following equation (12)

        Args:
            p       (4,1 ndarray) : Quaternion of form [w,x,y,z]
            q       (4,1 ndarray) : Quaternion of form [w,x,y,z]


        Returns:
            qproduct       (4,1 ndarray) : Quaternion
            """
        
        p_w,p_x,p_y,p_z = p.T[0]
        q_w,q_x,q_y,q_z = q.T[0]

        qproduct = np.array([[p_w*q_w - p_x*q_x - p_y*q_y - p_z*q_z],
                             [p_w*q_x + p_x*q_w + p_y*q_z - p_z*q_y],
                             [p_w*q_y - p_x*q_z + p_y*q_w + p_z*q_x],
                             [p_w*q_z + p_x*q_y - p_y*q_x + p_z*q_w]])

        return qproduct