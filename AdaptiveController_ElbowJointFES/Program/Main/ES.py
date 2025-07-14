"""
Base class of algorithm Stoatic Extrenum Seeking (SES). Online tuner of the PID.
"""
import random
import numpy as np


class ExtremumSeeking:

    def __init__(self, param_estimate,
                 perputation_amplitude, adaptation_gain):

        self.integralError = 0
        self.periodIndex = 0

        #multipliears
        self.a = perputation_amplitude
        self.beta = adaptation_gain #OR stepSize


        # Actual input on the map [Kp,Ki,Kd]
        self.theta = [0,0,0]
        # Used for finding best value. Due to muscle lag the
        self.old_theta = [0,0,0]
        # Best Optimizer of map (O*)
        self.theta_circumflex  = param_estimate # (Ô) Real time estimate of the optimizer (O*)

        self.lowest_errorCost = float('inf')
        self.bestThetaCircumflex = [0, 0, 0]

    def updateError(self, error):
        #since the TimeStep of the error is constant, and period of elapsed
        # time is a multiplication of the time steps. The time is negated
        # at the calculations and can be done by just indexing

        squaredError = np.square(error)
        self.integralError += squaredError # * TimeStep
        self.periodIndex += 1 # * Timestep = elaspsed time

    def calculateErrorCost(self):
        errorCost = self.integralError/self.periodIndex
        print("this is the periodIndec :",self.periodIndex,
              self.integralError, errorCost)

        self.__startNewPeriod()
        return errorCost

    def __startNewPeriod(self):
        self.integralError = 0
        self.periodIndex = 0

    def checkPositivity(self, old_gains, new_gains):
        """
            CHanges gains to last best values or old ones randomly if the
            gain is negative
        Args:
            old_value: float

        """
        for i in range(len(new_gains)):
            if new_gains[i] < 0:
                new_gains[i] = old_gains[i]
        return new_gains
    # Example usage
