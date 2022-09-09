from math import pi


class side_scan_data:
    def __init__(self, nS=500, rng=30, sensor_angle_placement=pi/4, sensor_opening=pi/3):
        self.nSamples = nS                              # Number of samples per active side and ping
        self.range = rng                                # Sonar range in meters
        self.res = (self.range*1.0)/(self.nSamples*2);  # Ping resolution [m] across track. Divided by 2 to make wave traveling both back and forth.
        self.theta = sensor_angle_placement
        self.alpha = sensor_opening

        self.swath_structure = swath_structure()


class swath_structure:
    def __init__(self):
        self.swath_right = []                           # Right side sonar data
        self.swath_left = []                            # Left side sonar data

        self.state = None                       # State of platform upom swath arrival
        self.altitude = None                    # Altitude of platform upom swath arrival