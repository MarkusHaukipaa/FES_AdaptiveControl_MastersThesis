
"""
SES class. works using the besics of ES class

Corrected: periodic index was used  ot create avg gradient direction. noiw
the perturbations are counted. the avg periodic index was 15700 and now the
perutrbation amount is 8
"""
from AdaptiveController_ElbowJointFES.Program.Main.ES import ExtremumSeeking
import numpy as np

class StoaticExtrenumSeeking(ExtremumSeeking):
    """
    SES algorithm
    Perputate the gains of PID controller. Calculates the resulting fitness
    score of each tried gain. Forms a gradient from average direction of the
    gains and it's magnitude with the fitness score. Corrects the average
    estimate gains direction according to the gradient.
    Extra parameters:
        perputation amplitude = amplitude of gain perputations
        adaption gain = amplitude of estimates change
    """
    # standartdeviation of value search
    std_ = [np.sqrt(1), np.sqrt(2), np.sqrt(3)]


    def __init__(self, param_estimate, perputation_amplitude = 0.01,
                 perturbationWeights = np.array([0.25,1,1]),
                 adaptation_gain = 0.025):
        super().__init__(param_estimate, perputation_amplitude, adaptation_gain)

        self.sinNoiseAdd = np.array([0, 0, 0]) # Sum of perputations (noise)
        self.errorCost = 0 # fitness value
        self.lowestErrorCost = float('inf')
        self.perputationWeights = perturbationWeights
        # Tap for finding best estimate values
        self.costs = [] # keep track of cost funtion change

        self.perturbateCalc = 0

    def __generateGaussianWhiteNoise(self, mean = 0):
        """
        Generates Gaussian White Noise for the stochastic function. For each PID
        gain formulates own random number.

        Args:
            mean: int, mean value of random numbers generated

        Returns: Array[float], noise for each gain

        """

        # Gaussian random number generator (Noise)
        noise = []
        for deviation in self.std_:
            noise.append(np.random.normal(loc=mean, scale = deviation))

        return noise

    def __updateThetaCircumflex(self, avgSinNoise):
        """

        Args:
            avgSinNoise: average of Random sinusoidal perputations

        """

        old_theta_circumflex =  self.theta_circumflex

        self.theta_circumflex= (self.theta_circumflex
                                - 2 * self.beta * self.perputationWeights/
                                self.a
                                * avgSinNoise
                                * self.errorCost)

        K = self.theta_circumflex[0]
        Ti = self.theta_circumflex[1]


        if K < self.a * self.perputationWeights[0]:
            self.theta_circumflex[0]  = (1.5 * self.a *
                                         self.perputationWeights[0])

        if Ti < self.a* self.perputationWeights[1]:
            self.theta_circumflex[1] = 3*self.a* self.perputationWeights[1]

        self.perturbateCalc = 0

    def getThetaCircumflex(self):
        return self.theta_circumflex

    def getEstimateGains(self):
        K = self.theta_circumflex[0]
        Ti = self.theta_circumflex[1]
        Td = self.theta_circumflex[2]

        gains = [K, K/Ti, K*Td]

        return gains

    def getAlfa(self):
        return self.a

    def getBeta(self):
        return self.beta

    def __averageNoise(self):
        # AVG noise is smaller than real AVG sinNoise
        # This is how the parameters of SES were tuned
        # Since all the time the parameter tuning is needed to be made
        #
        return self.sinNoiseAdd/self.perturbateCalc

    def getaverageNoise(self):
        return self.__averageNoise()

    def __perputateTheta(self, noise):
        """
        Adds sinusoidal perputations to gain estimate to make new ones.
        Args:
            noise: Array[float], Random perputations

        """
        self.old_theta = self.theta
        self.theta = self.theta_circumflex + self.a * self.perputationWeights * np.sin(noise)

        self.theta = self.checkPositivity(
            self.old_theta, self.theta)

        # return the affected noise( if negative values, the noise corrected
        # correlatiung to the one that makes that change

        return  (self.theta-self.theta_circumflex)/(self.a*self.perputationWeights)


        # checks
        # positivity and corrects
        # values to be non-negative


    def perputateGains(self):
        # Controls the perputation and updates average noise
        noise = self.__generateGaussianWhiteNoise()
        noise = self.__perputateTheta(noise)

        self.sinNoiseAdd = np.add(self.sinNoiseAdd, noise)

        self.perturbateCalc += 1

    def updateOptimizer(self):
        """
        UpdateOptimizer adapts the estimate AND adapts the adaption and
        perputation gains for the next generation according to the change of
        error. The change and limits are capped.

        """
        sinNoiseAverage = self.__averageNoise()
        self.errorCost = self.calculateErrorCost()
        self.costs.append(self.errorCost)

        #-------------
        # For Tracking best gain values
        if self.errorCost < self.lowestErrorCost:
            self.lowestErrorCost = self.errorCost
            self.bestThetaCircumflex = self.theta_circumflex
        # Update the estimate
        self.__updateThetaCircumflex(sinNoiseAverage)
        self.sinNoiseAdd = 0

    def getLowestErrorGains(self):
        #subsitatial way to do it
        K = self.bestThetaCircumflex[0]
        Ti = self.bestThetaCircumflex[1]
        Td = self.bestThetaCircumflex[2]

        # gains = [K, Ti, Td]
        gains = [K,K/Ti,K*Td]
        return gains

    def getGains(self):
        ##subsitatial way to do it
        K = self.theta[0]
        Ti = self.theta[1]
        Td = self.theta[2]

        gains = [K,K/Ti,K*Td]

        return gains

    def getCosts(self):
        return self.costs
