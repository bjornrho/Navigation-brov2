class side_scan_data:
    def __init__(self, nS=500, rng=30):
        self.nSamples = nS                              # Number of samples per active side and ping
        self.range = rng                                # Sonar range in meters
        self.res = (self.range*1.0)/self.nSamples;      # Ping resolution [m] across track
        self.right = []                                 # Right side sonar data 2D(ping ix, sample ix)
        self.left = []                                  # Left side sonar data 2D(ping ix, sample ix)