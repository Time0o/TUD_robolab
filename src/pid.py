class PIDController:
    """
    PID controller implementation.
    """

    COOLDOWN = 5

    def __init__(self, kp=16.5, ki=1.07, kd=0.0):
        """
        :param kp: Weight of proportional factor
        :param ki: Weight of integral factor
        :param kd: Weight of differential factor
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self._accumulated_error = 0.0
        self._previous_error = 0.0

    def control(self, error):
        """
        Obtain correction factor.

        :param error: Accumulated error since last invocation
        :return: Correction factor
        """
        self._accumulated_error = self._accumulated_error + error

        ret =  self.kp * error + \
               self.ki * self._accumulated_error + \
               self.kd * (error - self._previous_error)

        self._previous_error = error

        return ret
