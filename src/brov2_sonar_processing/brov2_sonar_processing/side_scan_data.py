from math import pi


class side_scan_data:
    def __init__(self, nS=500, rng=30, sensor_angle_placement=pi/4, sensor_opening=pi/3):
        self.nSamples = nS                              # Number of samples per active side and ping
        self.range = rng                                # Sonar range in meters
        self.res = (self.range*1.0)/self.nSamples;      # Ping resolution [m] across track
        self.theta = sensor_angle_placement
        self.alpha = sensor_opening

        self.right = []                                 # Right side sonar data 2D(ping ix, sample ix)
        self.left = []                                  # Left side sonar data 2D(ping ix, sample ix)
        self.altitude = []