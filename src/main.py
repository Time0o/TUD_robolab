#!/usr/bin/env python3

import ev3dev.ev3 as ev3

from robotbrain import RobotBrain

def run():
    brain = RobotBrain()
    brain.explore()

if __name__ == '__main__':
    run()
