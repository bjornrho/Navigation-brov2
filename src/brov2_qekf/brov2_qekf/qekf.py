import numpy as np
from numpy.linalg import inv
from numpy.linalg import norm
import sys
sys.path.append('utility_functions')
import utility_functions

class QEKF:
    def __init__(self, x_0, dx_0, P_0, std_a, std_gyro, std_dvl, std_depth, std_orientation, std_a_bias, std_gyro_bias, 
                 dvl_offset, barometer_offset, imu_offset):
        """Initialization of the QEKF

        Args:
            x_0                 (19 list) : Initial values of nominal state
            dx_0             (18 ndarray) : Initial values of error-state  
            P_0             (18x18 array) : Initial covariance of error-state
            std_a                         : Accelerometer Gaussian noise
            std_gyro                      : Gyrometer Gaussian noise
            std_dvl                       : Dvl Gaussian noise
            std_depth                     : Pressure sensor Gaussian noise
            std_orientation               : Orientation estimate BNO055 Gaussian noise
            std_a_bias                    : Accelerometer bias
            std_gyro_bias                 : Gyrometer bias
            dvl_offset        (3 ndarray) : Offset value for DVL placement on ROV
            barometer_offset  (3 ndarray) : Offset value for barometer placement on ROV
            imu_offset        (3 ndarray) : Offset value for IMU placement on ROV
            """
        
        # Resetting filter to initialization values
        self.filter_reset(x_0, dx_0, P_0, std_gyro, std_a, std_dvl, std_depth, std_orientation, std_gyro_bias, std_a_bias)

        # Defining offset values and previous IMU input (to be used accounting for offset)
        self.dvl_offset = np.array(dvl_offset).reshape(-1,1)
        self.barometer_offset = np.array(barometer_offset).reshape(-1,1)
        self.imu_offset = np.array(imu_offset).reshape(-1,1)
        self.last_u = np.zeros((2,3))

        # NIS-related initialization
        self.IMU_NIS_counter = 0
        self.store_NIS = False
        self.file_dir_name = ""
        self.NIS_writer = None

    def filter_reset(self, x_0, dx_0, P_0, std_gyro, std_a, std_dvl, std_depth, 
                                             std_orientation, std_a_bias, std_gyro_bias):
        """Resetting filter to initialization values

        Args:
            x_0                 (19 list) : Initial starting point of nominal state
            dx_0             (18 ndarray) : Initial starting point of error-state  
            P_0             (18x18 array) : Initial covariance of error-state
            std_a                         : Accelerometer Gaussian noise
            std_gyro                      : Gyrometer Gaussian noise
            std_dvl                       : Dvl Gaussian noise
            std_depth                     : Pressure sensor Gaussian noise
            std_orientation               : Orientation estimate BNO055 Gaussian noise
            std_a_bias                    : Accelerometer bias
            std_gyro_bias                 : Gyrometer bias"""
        
        # Initializing nominal state, error-state (mean zero) and covariance
        self.x = np.array(x_0).reshape(-1,1)
        self.dx = dx_0
        self.P = P_0

        # Initializing uncertainty terms (must be retrieved from parameters.yaml)
        self.std_gyro = std_gyro
        self.std_a = std_a
        self.std_dvl = std_dvl
        self.std_depth = std_depth
        self.std_orientation = std_orientation
        self.std_gyro_bias = std_gyro_bias
        self.std_a_bias = std_a_bias

    def integrate(self, u, dt):
        """Integrates measurement into nominal state

        Args:
            u     (2,3 ndarray) : Vector parametrization
            dt                  : Time step between consecutive IMU measurements

        Returns:
            x     (k+1 ndarray) : Nominal state"""

        v = self.x[3:6]
        q = self.x[6:10]
        a_b = self.x[10:13]
        omega_b = self.x[13:16]
        g = self.x[16:]

        a_m = u[0].reshape(-1,1)
        omega_m = u[1].reshape(-1,1)

        R_q = utility_functions.quaternion_to_rotation_matrix(q)
        phi = norm((omega_m - omega_b))*dt
        q_rot = utility_functions.rotation_vector_to_quaternion((omega_m - omega_b), phi)
        a_offset = utility_functions.cross(omega_m)@utility_functions.cross(omega_m)@self.imu_offset

        # Integrating IMU measurements into nominal state following equation (260), where biases and g doesn't change
        self.x[0:3] += v*dt + (1/2)*(R_q@(a_m - a_b - a_offset) - g)*(dt**2)
        self.x[3:6] += (R_q@(a_m - a_b - a_offset) - g)*dt
        self.x[6:10] = utility_functions.quaternion_product(q,q_rot)

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
        q = self.x[6:10]
        R = utility_functions.quaternion_to_rotation_matrix(q)
        zero    = np.zeros((3,3))
        I       = np.eye(3)
        a_b = self.x[10:13]
        omega_b = self.x[13:16]

        a_m = u[0].reshape(-1,1)
        omega_m = u[1].reshape(-1,1)

        omega = omega_m - omega_b
        a_offset = utility_functions.cross(omega)@utility_functions.cross(omega)@self.imu_offset
        a = a_m - a_b - a_offset

        # Jacobian wrt. the error following equation (270)
        F_x = np.block([[I,     I*dt,   zero,                                                   zero,   zero,   zero],
                        [zero,  I,      -R@utility_functions.cross(a)*dt,                       -R*dt,  zero,   -I*dt],
                        [zero,  zero,   utility_functions.rodrigues(omega,norm(omega)*dt).T,    zero,   -I*dt,  zero],
                        [zero,  zero,   zero,                                                   I,      zero,   zero],
                        [zero,  zero,   zero,                                                   zero,   I,      zero],
                        [zero,  zero,   zero,                                                   zero,   zero,   I]])
        
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

        self.P = P_hat
        self.last_u = u

        return self.dx, P_hat

    def update_orientation(self, q):
        # Defining the Jacobian H and the depth covariance V
        q_w,q_x,q_y,q_z = self.x[6:10].T[0]

        H_x = np.zeros((4,19))
        H_x[:,6:10] = np.eye(4)

        V = np.eye(4) * (self.std_orientation**2)
        
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

        innovation = np.array([[(q[0]-q_w)[0]],[(q[1]-q_x)[0]],[(q[2]-q_y)[0]],[(q[3]-q_z)[0]]])
        K = self.P@H.T@inv(H@self.P@H.T + V)
        
        # Correcting state and covariance according to equations (275) and (276)
        self.dx = K@innovation
        self.P = (np.eye(18) - K@H)@self.P@(np.eye(18) - K@H).T + K@V@K.T

        if self.store_NIS and self.IMU_NIS_counter%10==0:
            NIS = (innovation.T@inv(H@self.P@H.T + V)@innovation)[0,0]
            self.NIS_writer.writerow([NIS, np.nan, np.nan])
        self.IMU_NIS_counter += 1 # IMU publishing at a whopping 100 Hz. Limiting the amount of NIS data stored.

        return self.dx, self.P

    def update_depth(self, depth_measurement_raw):
        """Runs correction step of QEKF

        Args:
            depth_measurement_raw  : Incoming depth measurement from the barometer (corresponding to x[2])                       
        Returns:
            dx_hat  (19x1 ndarray) : Corrected state
            P_hat    (18x18 array) : Corrected covariances
            """

        # Account for pressure sensor offset (is the sign correct??)
        R = utility_functions.quaternion_to_rotation_matrix(self.x[6:10])
        depth_measurement_corrected = depth_measurement_raw + (R@self.barometer_offset)[2]


        # Defining the Jacobian H and the depth covariance V
        q_w,q_x,q_y,q_z = self.x[6:10].T[0]

        H_x = np.zeros((1,19))
        H_x[0,2] = 1
        H_x[0,6] = (2*(q_y*self.barometer_offset[0] - q_x*self.barometer_offset[1] + q_w*self.barometer_offset[2]))
        H_x[0,7] = (2*(q_z*self.barometer_offset[0] - q_w*self.barometer_offset[1] - q_x*self.barometer_offset[2]))
        H_x[0,8] = (2*(q_w*self.barometer_offset[0] + q_z*self.barometer_offset[1] - q_y*self.barometer_offset[2]))
        H_x[0,9] = (2*(q_x*self.barometer_offset[0] + q_y*self.barometer_offset[1] + q_z*self.barometer_offset[2]))

        V = self.std_depth**2
        
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

        innovation = depth_measurement_corrected - self.x[2]
        K = self.P@H.T*inv(H@self.P@H.T + V)
        
        # Correcting state and covariance according to equations (275) and (276)
        self.dx = K*innovation
        self.P = (np.eye(18) - K@H)@self.P@(np.eye(18) - K@H).T + V*K@K.T

        if self.store_NIS:
            NIS = (innovation*inv(H@self.P@H.T + V)*innovation)[0,0]
            self.NIS_writer.writerow([np.nan, NIS, np.nan])

        return self.dx, self.P

    def update_dvl(self, dvl_measurement_raw_body, dvl_covariance_body):
        """Runs correction step of QEKF

        Args:
            dvl_measurement_raw_body  (3 ndarray) : Incoming velocity measurement from the DVL (in corresponding frame)
            dvl_covariance          (3x3 ndarray) : Covariance matrix accompanying DVL measurement, V in equation (274)

        Returns:
            dx_hat  (19x1 ndarray)                : Corrected state
            P_hat    (18x18 array)                : Corrected covariances
            """

        # Account for the DVL offset
        omega_m = self.last_u[1].reshape(-1,1)
        omega_b = self.x[13:16]
        dvl_measurement_corrected = dvl_measurement_raw_body - utility_functions.cross(omega_m - omega_b)@np.array(self.dvl_offset)

        # Orient from body to global NED frame
        R = utility_functions.quaternion_to_rotation_matrix(np.array(self.x[6:10]).reshape(-1,1))
        dvl_measurement_corrected_NED = (R@dvl_measurement_corrected)[:2]
        dvl_covariance_NED = (R@dvl_covariance_body@R.T)[:2,:2]


        # Defining the Jacobian H
        H_x = np.zeros((2,19))
        H_x[:,3:5] = np.eye(2)

        q_w,q_x,q_y,q_z = self.x[6:10].T[0]
        Q_dtheta = (1/2)*np.array([[-q_x,   -q_y,   -q_z],
                                   [q_w,    -q_z,   q_y],
                                   [q_z,    q_w,    -q_x],
                                   [-q_y,   q_x,    q_w]]) # Equation (281)
        X_dx = np.zeros((19,18))
        X_dx[:6,:6] = np.eye(6)
        X_dx[6:10,6:9] = Q_dtheta
        X_dx[10:19,9:18] = np.eye(9) # Equation (280)
        
        H = H_x@X_dx

        innovation = dvl_measurement_corrected_NED - self.x[3:5]
        K = self.P@H.T@inv(H@self.P@H.T + dvl_covariance_NED)

        # Correcting state and covariance according to equations (275) and (276)
        self.dx = K@innovation
        
        #self.P = (np.eye(18) - K@H)@self.P
        self.P = (np.eye(18) - K@H)@self.P@(np.eye(18) - K@H).T + K@dvl_covariance_NED@K.T

        if self.store_NIS:
            NIS = (innovation.T@inv(H@self.P@H.T + dvl_covariance_NED)@innovation)[0,0]
            self.NIS_writer.writerow([np.nan, np.nan, NIS])

        return self.dx, self.P

    def inject(self):
        """Runs injection step of QEKF following equation (282)

        Args:
            dt                      : Time step between what???


        Returns:
            x       (k+1 ndarray)   : Updated nominal state
            """

        q_dtheta = utility_functions.rotation_vector_to_quaternion(self.dx[6:9], norm(self.dx[6:9]))
        
        self.x[0:3] += self.dx[0:3]
        self.x[3:6] += self.dx[3:6]
        self.x[6:10] = utility_functions.quaternion_product(self.x[6:10], q_dtheta)
        self.x[10:13] += self.dx[9:12]
        self.x[13:16] += self.dx[12:15]
        self.x[16:] += self.dx[15:]

        return self.x

    def reset(self):
        """Runs reset step of QEKF

        Returns:
            dx_hat  (k ndarray) : Reset error-state
            P_hat   (mxm array) : Reset covariances
            """

        dtheta = self.dx[6:9]
        # Reset the error-state estimate following equation (285)
        self.dx = np.zeros((18,1))

        # Form G and reset the covariances following equation (286)
        G = np.zeros((18,18))
        G[:6,:6] = np.eye(6)
        G[6:9,6:9] = np.eye(3) - utility_functions.cross((1/2)*dtheta)
        G[9:18,9:18] = np.eye(9)

        self.P = G@self.P@G.T

        return self.dx, self.P