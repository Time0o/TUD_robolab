from math import pi, sqrt
from time import sleep, time

from ev3dev.ev3 import ColorSensor, GyroSensor, UltrasonicSensor

class DistanceSensor:
    def __init__(self):
        """
        Set up ultrasonic distance sensor.
        """
        self._sensor = UltrasonicSensor()
        self._sensor.mode = UltrasonicSensor.MODE_US_DIST_CM

    def read(self):
        """
        Obtain measured distance in mm.

        :return: Measured distance in mm
        """
        return self._sensor.value()

class Gyro:
    def __init__(self):
        """
        Set up gyroscope.
        """
        self._sensor = GyroSensor()

    def calibrate(self):
        """
        Calibrate and set up the gyroscope.
        """
        self._sensor.mode = GyroSensor.MODE_GYRO_RATE
        self._sensor.mode = GyroSensor.MODE_GYRO_ANG
        sleep(0.1)

    def read(self):
        """
        Read gyroscope.

        :return: Gyroscope reading in degrees
        """
        return self._sensor.value()

class LightSensor:
    COLOR_NONE = 0
    COLOR_RED = 1
    COLOR_BLUE = 2

    THRESHOLD_SCAN = 0.3
    THRESHOLD_DOCK = 0.3
    THRESHOLD_WHITE = 0.8
    EPSILON = 0.3

    def __init__(self):
        """
        Set up light sensor.
        """
        self._sensor = ColorSensor()
        self._sensor.mode = ColorSensor.MODE_RGB_RAW

    def calibrate_black(self):
        """
        Calibrate the sensor's pure pure black recognition.
        """
        r, g, b = self._sensor.raw
        self._ik = r + g + b

    def calibrate_white(self):
        """
        Calibrate the sensor's pure white recognition.
        """
        r, g, b = self._sensor.raw
        self._iw = r + g + b

    def read(self):
        """
        Read the light sensor.

        :return: Color and intensity, scaled from zero to one, taking the
                 calibrated black and white values into account
        """
        r, g, b = self._sensor.raw
        intensity = sum((r, g, b))

        color = LightSensor.COLOR_NONE
        if r > 1 / 2 * intensity:
            color = LightSensor.COLOR_RED
        elif r < 1 / 5 * intensity:
            color = LightSensor.COLOR_BLUE

        intensity_scaled = (2 * intensity - (self._iw + self._ik)) / \
                           (self._iw - self._ik)

        return color, intensity_scaled
