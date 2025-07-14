
class PID:

    def __init__(self, Kp, Ki, Kd):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral_error = 0
        self.derivate_error = 0
        self.output = 0
        self.clamped = False
        self.adjustToChange = False
        self.i = 0


    def getLastError(self):
        return self.last_error

    def getIntegralError(self):
        return self.integral_error

    def getDerivateError(self):
        return self.derivate_error

    def nullIntegral(self):
        self.integral_error = 0

    def getGains(self):
        return self.Kp, self.Ki, self.Kd

    def compute(self, error, samplingTime):
        """

        Args: self, error, TIME_STEP
            error: float, current error


        Returns: excitation, error,

        """
        # self.__checkWindUp(error)

        if not self.clamped:
            self.integral_error +=  (error * samplingTime)

        self.derivate_error = (error - self.last_error) / samplingTime
        self.last_error = error
        self.output = (self.Kp * error + self.Ki * self.integral_error
                       + self.Kd * self.derivate_error)

        return self.output

    def printExcInParts(self,error,samplingTime):
        #Information
        if not self.clamped:
            self.integral_error += (error * samplingTime)

        self.derivate_error = (error - self.last_error) / samplingTime
        self.output = (self.Kp * error + self.Ki * self.integral_error
                       + self.Kd * self.derivate_error)

        print(f"Prop{self.Kp * error}, Int{self.Ki * self.integral_error}, "
              f"deri{self.Kd * self.derivate_error} and Total:{self.output}")

    def updateGains (self, newKp, newKi, newKd):
        if self.adjustToChange:
            self.integral_error = self.integral_error*(self.Ki/newKi)

        self.Kp = newKp
        self.Ki = newKi
        self.Kd = newKd

    def dontAdjustToChange(self):
        self.adjustToChange = False

    def doAdjustToChange(self):
        self.adjustToChange = True

    def clampIntegral(self):
        self.clamped = True

    def releaseClampIntegral(self):
        self.clamped = False

    def __checkWindUp(self,error):
        """
        Limits the windup resultant of dead time and perturbations. The windup
        can be too aggressively realised in the case of increase in integral
        gain resulting to oscillations
        Args:
            error: float, error value now
        """
        #If the last and resent error are different signs (direction)
        if error*self.last_error < 0:
            self.clamped = False
            self.i = 0
        elif not self.clamped:
            self.i += 1
            if self.i > 16000: # 8 second intervall(adaption rate)
                self.clamped = True