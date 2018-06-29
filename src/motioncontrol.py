from math import pi
from time import sleep

from ev3dev.ev3 import LargeMotor
from ev3dev.ev3 import Sound

from odometry import IterativePoseEstimator, RotationEstimator
from pid import PIDController
from planet import Direction, Planet
from robotsetup import LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT
from sensors import DistanceSensor, Gyro, LightSensor

class MotionControl:
    TRANS_DUTY_CYCLE = 50
    ROT_DUTY_CYCLE = 30
    MAX_DUTY_CYCLE = 50

    DOCK_OFFLINE_THRESHOLD = 20
    DOCK_RETRY_TIMEOUT = 1

    ROTATE_SKIPLINE_TIMEOUT = 1
    ROTATE_STOP_RIGHT_TIMEOUT = 0.25
    ROTATE_PAUSE_AFTER_TIMEOUT = 0.2
    ROTATE_DOCK_LINE_TICKS = 2

    CALIBRATE_TURN_RIGHT_TIMEOUT = 0.5
    CALIBRATE_CALIBRATION_TIMEOUT = 0.5

    FOLLOW_ROLLFORWARD_TIMEOUT = 0.8
    FOLLOW_OFFLINE_THRESHOLD = 10
    FOLLOW_OBSTACLE_DISTANCE_THRESHOLD = 150
    FOLLOW_OBSTACLE_ESTIMATION_FREQUENCY = 5
    FOLLOW_POSE_ESTIMATION_FREQUENCY = 10

    SCAN_PATHS_INITIAL_TURN = 315

    def __init__(self):
        """
        Constructor, set up devices.
        """
        self._left_motor = LargeMotor(LEFT_MOTOR_PORT)
        self._right_motor = LargeMotor(RIGHT_MOTOR_PORT)

        self._distance_sensor = DistanceSensor()
        self._gyro = Gyro()
        self._light_sensor = LightSensor()

        self._pose_estimator = IterativePoseEstimator(
            self._left_motor, self._right_motor, self._gyro)
        self._rotation_estimator = RotationEstimator(
            self._left_motor, self._right_motor, self._gyro)

        self.position = (0, 0)
        self.rotation = 0

        self.sound = Sound()

    @staticmethod
    def _limit_duty_cycle(duty_cycle):
        if duty_cycle > MotionControl.MAX_DUTY_CYCLE:
            return MotionControl.MAX_DUTY_CYCLE

        if duty_cycle < -MotionControl.MAX_DUTY_CYCLE:
            return -MotionControl.MAX_DUTY_CYCLE

        return duty_cycle

    def _dock(self):
        pid = PIDController()

        self._left_motor.run_direct()
        self._right_motor.run_direct()

        offline_counter = 0
        cooldown = 0
        while True:
            _, intensity = self._light_sensor.read()

            if intensity > LightSensor.THRESHOLD_WHITE:
                offline_counter += 1
                if offline_counter == MotionControl.DOCK_OFFLINE_THRESHOLD:
                    if __debug__:
                        print("WARNING: docking failed, retrying...")

                    self._left_motor.stop(
                        stop_action=LargeMotor.STOP_ACTION_BRAKE)
                    self._right_motor.stop(
                        stop_action=LargeMotor.STOP_ACTION_BRAKE)

                    self._left_motor.run_direct(
                        duty_cycle_sp=MotionControl.ROT_DUTY_CYCLE)
                    self._right_motor.run_direct(
                        duty_cycle_sp=-MotionControl.ROT_DUTY_CYCLE)

                    sleep(MotionControl.DOCK_RETRY_TIMEOUT)
                    self._dock()
            else:
                offline_counter = 0

            if abs(intensity) < LightSensor.EPSILON:
                cooldown += 1
                if cooldown > PIDController.COOLDOWN:
                    break
            else:
                cooldown = 0

            correction = pid.control(intensity)
            self._left_motor.duty_cycle_sp = self._limit_duty_cycle(-correction)
            self._right_motor.duty_cycle_sp = self._limit_duty_cycle(correction)

        self._left_motor.stop(stop_action=LargeMotor.STOP_ACTION_HOLD)
        self._right_motor.stop(stop_action=LargeMotor.STOP_ACTION_HOLD)

    def _reset(self):
        self._left_motor.stop(stop_action=LargeMotor.STOP_ACTION_COAST)
        self._right_motor.stop(stop_action=LargeMotor.STOP_ACTION_COAST)

    def _rotate(self, direction, skip_line=True):
        if direction == 'left':
            self._left_motor.run_direct(
                duty_cycle_sp=-MotionControl.ROT_DUTY_CYCLE)
            self._right_motor.run_direct(
                duty_cycle_sp=MotionControl.ROT_DUTY_CYCLE)
        elif direction == 'right':
            self._left_motor.run_direct(
                duty_cycle_sp=MotionControl.ROT_DUTY_CYCLE)
            self._right_motor.run_direct(
                duty_cycle_sp=-MotionControl.ROT_DUTY_CYCLE)
        else:
            raise ValueError('invalid direction specifier')

        if skip_line:
            _, intensity = self._light_sensor.read()
            if intensity < LightSensor.THRESHOLD_WHITE:
                sleep(MotionControl.ROTATE_SKIPLINE_TIMEOUT)

        line_ticks = 0
        while True:
            _, intensity = self._light_sensor.read()
            if intensity < LightSensor.THRESHOLD_DOCK:
                line_ticks += 1
                if line_ticks == MotionControl.ROTATE_DOCK_LINE_TICKS:
                    break
            else:
                line_ticks = 0

        if direction == 'right':
            sleep(MotionControl.ROTATE_STOP_RIGHT_TIMEOUT)

        self._left_motor.stop(stop_action=LargeMotor.STOP_ACTION_HOLD)
        self._right_motor.stop(stop_action=LargeMotor.STOP_ACTION_HOLD)
        sleep(MotionControl.ROTATE_PAUSE_AFTER_TIMEOUT)

    def calibrate(self):
        """
        Calibrate the light sensor.
        """
        self._light_sensor.calibrate_black()
        sleep(MotionControl.CALIBRATE_CALIBRATION_TIMEOUT)
        self._left_motor.run_direct(duty_cycle_sp=MotionControl.ROT_DUTY_CYCLE)
        self._right_motor.run_direct(duty_cycle_sp=-MotionControl.ROT_DUTY_CYCLE)
        sleep(MotionControl.CALIBRATE_TURN_RIGHT_TIMEOUT)

        self._left_motor.stop(stop_action=LargeMotor.STOP_ACTION_BRAKE)
        self._right_motor.stop(stop_action=LargeMotor.STOP_ACTION_BRAKE)
        self._light_sensor.calibrate_white()
        sleep(MotionControl.CALIBRATE_CALIBRATION_TIMEOUT)

        self._rotate('left')
        self._dock()

    def follow(self, backtracking=False):
        """
        Follow the line the robot is currently on to the next node.

        :param backtracking: True when backtracking after encountering an
                             obstacle
        :return: Color and position of reached node and resulting rotation of
                 the robot
        """
        pid = PIDController()
        self._pose_estimator.calibrate(self.position, self.rotation)

        pose_estimation_ticks = 0
        offline_ticks = 0
        obstacle_ticks = 0

        self._left_motor.run_direct()
        self._right_motor.run_direct()

        while True:
            color, intensity = self._light_sensor.read()

            if color != LightSensor.COLOR_NONE:
                self._left_motor.duty_cycle_sp=MotionControl.TRANS_DUTY_CYCLE
                self._right_motor.duty_cycle_sp=MotionControl.TRANS_DUTY_CYCLE
                sleep(MotionControl.FOLLOW_ROLLFORWARD_TIMEOUT)

                self._left_motor.stop(stop_action=LargeMotor.STOP_ACTION_BRAKE)
                self._right_motor.stop(stop_action=LargeMotor.STOP_ACTION_BRAKE)

                self.position, self.rotation = self._pose_estimator.update()

                if backtracking:
                    return None

                return color, self.position, self.rotation

            obstacle_ticks += 1
            if obstacle_ticks == MotionControl.FOLLOW_OBSTACLE_ESTIMATION_FREQUENCY:
                obstacle_distance = self._distance_sensor.read()
                if obstacle_distance < MotionControl.FOLLOW_OBSTACLE_DISTANCE_THRESHOLD:
                    if __debug__:
                        print("WARNING: obstacle")

                    self._left_motor.stop(stop_action=LargeMotor.STOP_ACTION_HOLD)
                    self._right_motor.stop(stop_action=LargeMotor.STOP_ACTION_HOLD)
                    self._rotate('left')
                    self._dock()

                    return self.follow(backtracking=True)

                obstacle_ticks = 0

            if intensity > LightSensor.THRESHOLD_WHITE:
                offline_ticks += 1
            else:
                offline_ticks = 0

            if offline_ticks == MotionControl.FOLLOW_OFFLINE_THRESHOLD:
                if __debug__:
                    print("WARNING: offline")

                self._left_motor.stop(stop_action=LargeMotor.STOP_ACTION_HOLD)
                self._right_motor.stop(stop_action=LargeMotor.STOP_ACTION_HOLD)
                self._rotation_estimator.calibrate()
                self._rotate('left', skip_line=False)
                self._dock()
                if self._rotation_estimator.update(arbitrate=True) == 180:
                    return self.follow(backtracking=True)
                else:
                    offline_ticks = 0
                    self._left_motor.run_direct()
                    self._right_motor.run_direct()
                    continue

            correction = pid.control(intensity)

            self._left_motor.duty_cycle_sp = self._limit_duty_cycle(
                MotionControl.TRANS_DUTY_CYCLE - correction)
            self._right_motor.duty_cycle_sp = self._limit_duty_cycle(
                MotionControl.TRANS_DUTY_CYCLE + correction)

            pose_estimation_ticks += 1
            if pose_estimation_ticks == MotionControl.FOLLOW_POSE_ESTIMATION_FREQUENCY:
                self._pose_estimator.update()
                pose_estimation_ticks = 0

    def scan_paths(self):
        """
        Scan exits of the node the robot is currently standing on.

        :return: exits, direction the robot is facing after completed scan
        """
        starting_direction = self.rotation

        self._rotation_estimator.calibrate()

        self._left_motor.run_direct(duty_cycle_sp=-MotionControl.ROT_DUTY_CYCLE)
        self._right_motor.run_direct(duty_cycle_sp=MotionControl.ROT_DUTY_CYCLE)

        exits = set()
        while True:
            current_turn = self._rotation_estimator.update()
            if abs(current_turn) > MotionControl.SCAN_PATHS_INITIAL_TURN:
                break

            _, intensity = self._light_sensor.read()
            if abs(intensity) < LightSensor.THRESHOLD_SCAN:
                current_direction = Planet.to_direction(starting_direction + current_turn)
                if __debug__:
                    print(current_direction)
                exits.add(current_direction)

        self._rotate('left', skip_line=False)

        if __debug__:
            print("Rotated left")

        self._dock()

        final_turn = self._rotation_estimator.update()
        final_direction = Planet.to_direction(starting_direction + final_turn)
        exits.add(final_direction)

        self.rotation = final_direction

        return exits, final_direction

    def turn_to(self, direction, sweep=False):
        """
        Turn the robot to a given absolute direction

        :param direction: Direction to turn to
        :param sweep: Perform sweep maneuver while turning
        :return: Actual (measured) rotation
        """
        starting_direction = self.rotation
        turn = (direction - starting_direction) % 360

        self._rotation_estimator.calibrate()

        if turn == 0:
            if sweep:
                self._left_motor.run_direct(
                    duty_cycle_sp=MotionControl.ROT_DUTY_CYCLE / 2)
                self._right_motor.run_direct(
                    duty_cycle_sp=-MotionControl.ROT_DUTY_CYCLE / 2)
                sleep(0.5)
            self._dock()
        elif turn == 90:
            self._rotate('right')
            self._dock()
        elif turn == 180:
            self._rotate('left')
            if abs(self._rotation_estimator.update()) < 135:
                self._rotate('left')
            self._dock()
        elif turn == 270:
            self._rotate('left')
            self._dock()
        else:
            raise ValueError('invalid turn: {}'.format(turn))

        actual_turn = self._rotation_estimator.update(arbitrate=True)
        actual_direction = (starting_direction + actual_turn) % 360
        if __debug__:
            if actual_direction != direction:
                warn = "WARNING: direction after turn not as desired (expected {} but got {})"
                print(warn.format(direction, actual_direction))

        self.rotation = actual_direction
        return self.rotation

    def update_position(self, position):
        """
        Update the current position.

        :param position: New position
        """
        self.position = position

    def update_rotation(self, direction):
        """
        Update the current rotation.

        :param direction: New direction
        """
        self.rotation = direction
