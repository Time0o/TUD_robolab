#!/usr/bin/env python3

from math import inf, sqrt
from uuid import uuid4

from ev3dev.ev3 import Sound
import paho.mqtt.client as mqtt

from communication import Communication
from motioncontrol import MotionControl
from planet import Direction, Planet

ADJACENCIES = {
    Direction.NORTH: [(-1, 1), (0, 1), (1, 1)],
    Direction.EAST: [(1, 1), (1, 0), (1, -1)],
    Direction.SOUTH: [(1, -1), (0, -1), (-1, -1)],
    Direction.WEST: [(-1, -1), (-1, 0), (-1, 1)]
}

def play_daisy():
   with open('daisy.txt') as f:
       notes = [line.rstrip('\n') for line in f.readlines()]
       notes = ' '.join(notes)
       s = Sound()
       s.beep(notes)

class RobotBrain:
    def __init__(self, testplanet=None):

        mqtt_client = mqtt.Client(
            client_id=str(uuid4()),
            clean_session=False,
            protocol=mqtt.MQTTv31)

        self.communication = Communication(mqtt_client, planet=testplanet)

        self.motioncontrol = MotionControl()
        self.planet = Planet()

        self.visited = set()
        self.current_path = []
        self.target = None
        self.use_target_heuristic = False

        # calibrate and drive to starting node
        self.motioncontrol.calibrate()
        start_color, _, _ = self.motioncontrol.follow()

        start = self.communication.send_ready()

        if __debug__:
            print("received starting node from server: {}".format(start))

        self.planet.set_starting_node(start, start_color)

        self.current_node = start
        self.visited.add(start)
        self.motioncontrol.update_position(Planet.from_node(start))

        # discover exits
        self.motioncontrol.update_rotation(Planet.from_direction(Direction.NORTH))
        exits, self.current_direction = self.motioncontrol.scan_paths()

        if __debug__:
            print('discovered exits: {}'.format(exits))

        for exit in exits:
            if exit == Direction.SOUTH:
                continue
            self.planet.add_undiscovered_exit(self.current_node + (exit,))

    def explore(self):
        while True:
            if __debug__:
                print("exploring, current pose is: {}".format(
                    self.current_node + (self.current_direction,)))

            # follow path to next node
            start = self.current_node + (self.current_direction,)
            reached = self.motioncontrol.follow()

            destination = None
            blocked = False
            if reached is None:
                destination = start
                blocked = True

            else:
                color, position, rotation = reached
                node = self.planet.to_node(position, color)
                direction = self.planet.to_direction(rotation)
                destination = node + (Planet.invert_direction(direction),)

            if __debug__:
                print("reached node at: {}".format(destination))

            destination, weight, blocked, paths, target = \
                self.communication.send_path(start, destination, blocked)

            if __debug__:
                print("received path response from server:")
                print("  destination: {}:".format(destination))
                print("  weight: {}".format(weight))
                print("  paths: {}".format(paths))
                print("  target: {}".format(target))

            self.planet.add_path(start, destination, weight)
            self.planet.mark_exit_discovered(start)
            self.planet.mark_exit_discovered(destination)

            if target:
                self.target=target

                if __debug__:
                    print("new target!")

            received_paths = False
            if paths:
                received_paths = True
                for start, dest, weight in paths:
                    self.planet.add_path(start, dest, weight)
                    self.planet.mark_exit_discovered(start)
                    self.planet.mark_exit_discovered(dest)

            x, y, incoming_direction = destination
            direction = Planet.invert_direction(incoming_direction)

            self.current_node = (x, y)
            self.current_direction = direction

            if self.target:
                target_x, target_y = self.target
                self.use_target_heuristic = sqrt((target_x - x)**2 + \
                                            (target_y - y)**2) < 15

                if not self.use_target_heuristic and __debug__:
                    print("WARNING: ignoring target heuristics!")

            self.motioncontrol.update_position(
                Planet.from_node(self.current_node))
            self.motioncontrol.update_rotation(
                Planet.from_direction(self.current_direction))

            already_visited = True
            if self.current_node not in self.visited:
                already_visited = False

                exits, direction = self.motioncontrol.scan_paths()
                self.current_direction = direction

                self.motioncontrol.update_rotation(Planet.from_direction(
                    self.current_direction))

                if __debug__:
                    print('discovered exits: {}'.format(exits))

                for exit in exits:
                    if exit == incoming_direction:
                        continue
                    self.planet.add_undiscovered_exit(self.current_node + (exit,))

                self.visited.add(self.current_node)

            current_x, current_y = self.current_node
            known_paths = self.planet.get_paths()
            undiscovered = self.planet.get_undiscovered_exits()

            # exploration completed
            target_path_known = self.target and \
                self.planet.shortest_path(self.current_node, self.target)

            if not undiscovered and not target_path_known:
                self.communication.send_exploration_completed("...")

                if __debug__:
                    print("completey discovered planet!")

                break

            # progress towards target
            if self.target is not None:
                if __debug__:
                    print("There is a target...")

                if self.current_node == self.target:
                    self.communication.send_target_reached("...")

                    if __debug__:
                        print("finished!")

                    break

                shortest_path = \
                  self.planet.shortest_path(self.current_node, self.target)
                if shortest_path:
                    if __debug__:
                        print("found shortest path to target")

                    self.current_path = shortest_path
                elif self.use_target_heuristic:
                    if __debug__:
                        print("no known path to target, finding path to nearest projected node")

                    target_x, target_y = self.target

                    best_node = None
                    best_node_distance = inf
                    best_exit = None
                    best_path = None

                    reachable_nodes = \
                      self.planet.get_connected_known_nodes(self.current_node)

                    for node in reachable_nodes:
                        shortest_path = self.planet.shortest_path(
                            self.current_node, node)

                        if shortest_path is None:
                            continue

                        exits = self.planet.get_undiscovered_exits().get(node)
                        if not exits:
                            continue

                        node_x, node_y = node

                        for exit in exits:
                            projected_x, projected_y = ADJACENCIES[exit][1]

                            projected_node_distance = \
                                sqrt((target_x - (node_x + projected_x))**2 + \
                                     (target_y - (node_y + projected_y))**2)

                            if projected_node_distance > best_node_distance:
                                continue

                            if projected_node_distance < best_node_distance or \
                               len(shortest_path) < len(best_path):

                                best_node = node
                                best_node_distance = projected_node_distance
                                best_exit = exit
                                best_path = shortest_path + [(node_x, node_y, exit)]

                    self.current_path = best_path

            # current node not yet discovered
            if self.current_node in undiscovered and \
               (not self.use_target_heuristic or target is None):

                if __debug__:
                    print("current node not discovered")

                undiscovered_exits = undiscovered[self.current_node]

                # prefer paths that are likely to lead to already discovered nodes
                preferred_exit_heuristics = {}

                for exit in undiscovered_exits:
                    preferred_exit_heuristics[exit] = 0
                    for delta_x, delta_y in ADJACENCIES[exit]:
                        if (current_x + delta_x, current_y + delta_y) in known_paths:
                            preferred_exit_heuristics[exit] += 1

                if __debug__:
                    print("exit heuristics: {}".format(preferred_exit_heuristics))

                while True:
                    preferred_exit = None
                    max_heuristic = -1
                    for direction in [Direction.NORTH, Direction.EAST,
                                      Direction.SOUTH, Direction.WEST]:

                        heuristic = preferred_exit_heuristics.get(direction)
                        if heuristic is None:
                            continue

                        if heuristic > max_heuristic:
                            preferred_exit = direction
                            max_heuristic = heuristic

                    if __debug__:
                        print("choosing exit: " + str(preferred_exit))

                    turn = self.motioncontrol.turn_to(preferred_exit,
                                                      sweep=already_visited)
                    if turn == preferred_exit:
                        break

                    if __debug__:
                        warn = "WARNING: exit {} not found, marking as discovered"
                        print(warn.format(preferred_exit))

                    self.planet.mark_exit_discovered(
                        self.current_node + (preferred_exit,))
                    preferred_exit_heuristics.pop(preferred_exit)

                self.current_direction = preferred_exit
                self.motioncontrol.update_rotation(
                    Planet.from_direction(self.current_direction))

                continue

            elif (not self.current_path or received_paths) and \
                 (not self.use_target_heuristic or self.target is None):

                if __debug__:
                    print("current node discovered, looking for nearest undiscovered node")

                # find nearest node with unexplored exit
                nearest_undiscovered_node = None
                nearest_undiscovered_node_path = None
                nearest_undiscovered_node_distance = inf

                for node in undiscovered:
                    path = self.planet.shortest_path(self.current_node, node)
                    if not path:
                        continue

                    pathlen = len(path)

                    if pathlen < nearest_undiscovered_node_distance:
                        nearest_undiscovered_node = node
                        nearest_undiscovered_node_path = path
                        nearest_undiscovered_node_distance = pathlen

                self.current_path = nearest_undiscovered_node_path

                if __debug__:
                    dbg = "backtracking to nearest undiscovered node {} on path {}"
                    print(dbg.format(nearest_undiscovered_node,
                                     nearest_undiscovered_node_path))

            if __debug__:
                print("following path {}".format(self.current_path))

            _, _, exit = self.current_path[0]
            self.current_path = self.current_path[1:]

            while self.motioncontrol.turn_to(exit, sweep=already_visited) != exit:
                pass

            self.current_direction = exit
            self.motioncontrol.update_rotation(
                Planet.from_direction(self.current_direction))

        play_daisy()
