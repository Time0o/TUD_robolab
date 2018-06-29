from math import sin, cos, pi

import robotsetup as setup

class RotationEstimator:
    """
    Estimates the rotation of the robot, based on motor and gyroscope data.
    """

    def __init__(self, motor_left, motor_right, gyro, alpha=0):
        """
        Constructor.

        :param motor_left: Left motor object
        :param motor_right: Right motor object
        :param gyro: Gyro object
        :param alpha: Weight determining influence of gyro data on estimate
        """
        self._motor_left = motor_left
        self._motor_right = motor_right
        self._gyro = gyro

        self._alpha = alpha

    def calibrate(self):
        """
        Calibrate the gyroscope and motor rotary encoders.
        """
        self._gyro.calibrate()
        self._rel_pos_left = self._motor_left.position
        self._rel_pos_right = self._motor_right.position

    def update(self, arbitrate=False):
        """
        Update position estimate.

        :param arbitrate: If True, arbitrate rotation to a multiple of 90 degrees
        :return: Current rotation in degrees
        """
        counts_left = self._motor_left.position - self._rel_pos_left
        counts_right = self._motor_right.position - self._rel_pos_right

        avg_counts = (counts_left - counts_right) / 2
        avg_distance = setup.WHEEL_CIRCUMFERENCE_MM * \
                       (avg_counts / setup.WHEEL_COUNTS_PER_ROT)

        rotation_gyro = self._gyro.read()
        rotation_wheels = 360 * (avg_distance / setup.AXIS_CIRCUMFERENCE_MM)
        rotation = rotation_wheels + self._alpha * \
                   (rotation_gyro - rotation_wheels)

        if arbitrate:
            return (90 * round(rotation / 90)) % 360

        return rotation

class IterativePoseEstimator:
    """
    Iteratively estimates the robots real world pose, based on motor and
    gyroscope data.
    """

    def __init__(self, motor_left, motor_right, gyro, alpha=0):
        """
        Constructor.

        :param motor_left: Left motor object
        :param motor_right: Right motor object
        :param gyro: Gyro object
        :param alpha: Weight determining influence of gyro data on estimate
        """
        self._motor_left = motor_left
        self._motor_right = motor_right
        self._gyro = gyro

        self._alpha = alpha

    def calibrate(self, initial_position, initial_rotation):
        """
        Calibrate to initial rotation and rotation.

        :param initial_position: Initial position (x,y)
        :param initial_rotation: Initial rotation in degrees
        """
        self._x_est, self._y_est = initial_position
        self._rot_initial = initial_rotation
        self._rot_est = self._rot_initial

        self._gyro.calibrate()

        self._rel_pos_left = self._motor_left.position
        self._rel_pos_right = self._motor_right.position

    def update(self, arbitrate=False):
        """
        Update the current pose estimate.

        :param arbitrate: If True, pose is arbitrated to nearest grid point.
        :return: Pose estimate ((x,y),rotation)
        """
        counts_left = self._motor_left.position - self._rel_pos_left
        counts_right = self._motor_right.position - self._rel_pos_right
        self._rel_pos_left = self._motor_left.position
        self._rel_pos_right = self._motor_right.position

        distance_left = setup.WHEEL_CIRCUMFERENCE_MM * \
                        (counts_left / setup.WHEEL_COUNTS_PER_ROT)

        distance_right = setup.WHEEL_CIRCUMFERENCE_MM * \
                         (counts_right / setup.WHEEL_COUNTS_PER_ROT)

        distance = (distance_left + distance_right) / 2

        rotation_gyro = self._rot_initial + self._gyro.read()
        rotation_wheels = self._rot_est + \
            180 * (distance_left - distance_right) / setup.AXIS_CIRCUMFERENCE_MM

        rot_pred = rotation_wheels + self._alpha * (rotation_gyro - rotation_wheels)
        x_pred = self._x_est + distance * sin((2 * pi) * (rot_pred / 360))
        y_pred = self._y_est + distance * cos((2 * pi) * (rot_pred / 360))

        self._x_est = x_pred
        self._y_est = y_pred
        self._rot_est = rot_pred % 360

        if arbitrate:
            x_arb = 500 * round(self._x_est / 500)
            y_arb = 500 * round(self._y_est / 500)
            rot_arb = (90 * round(self._rot_est / 90)) % 360
            return (x_arb, y_arb), rot_arb

        return (self._x_est, self._y_est), self._rot_est
        return self.x_est, self.y_est, self.phi_est
