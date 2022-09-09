import numpy as np
from csaps import csaps

class cubic_spline_regression:
    def __init__(self, nS=500, sp=1e-6):
            self.nSamples = nS
            self.smoothing_parameter = sp

    def swath_normalization(self, swath):
        x = np.linspace(0., self.nSamples, self.nSamples)
        spl = csaps(x, swath, x, smooth=self.smoothing_parameter)
        
        return np.divide(swath, spl), spl