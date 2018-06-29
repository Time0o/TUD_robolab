from copy import copy
from enum import IntEnum, unique
from functools import total_ordering
from heapq import heappush, heappop
from math import inf

@unique
class Direction(IntEnum):
    NORTH = 0
    EAST  = 90
    SOUTH = 180
    WEST  = 270

class Planet:
    NODE_DISTANCE_MM = 500

    def __init__(self):
        """
        Constructor.
        """
        self._paths = {}
        self._undiscovered_exits = {}

    @staticmethod
    def from_direction(direction):
        """
        Convert direction to angle.

        :param direction: Direction mnemonic
        :return: Angle in degrees
        """
        return direction.value

    @staticmethod
    def from_node(node):
        """
        Convert a node position into real world coordinates.

        :param node: Node coordinates
        :return: Real world coordinates
        """
        x, y = node
        return x * Planet.NODE_DISTANCE_MM, y * Planet.NODE_DISTANCE_MM

    @staticmethod
    def invert_direction(direction):
        """
        Return the inverse of some direction.

        :param direction: Direction to invert
        :return: Inverted direction
        """
        return {
            Direction.NORTH : Direction.SOUTH,
            Direction.EAST : Direction.WEST,
            Direction.SOUTH : Direction.NORTH,
            Direction.WEST : Direction.EAST
        }[direction]

    @staticmethod
    def to_direction(direction):
        """
        Convert angle to nearest direction.

        :param direction: Angle in degrees
        :return: Nearest direction
        """
        return {
           0   : Direction.NORTH,
           90  : Direction.EAST,
           180 : Direction.SOUTH,
           270 : Direction.WEST
        }[(90 * round(direction / 90)) % 360]

    def _add_node(self, x : int, y : int):
        self._paths[(x, y)] = {}

    def add_path(self, start, target, weight):
        """
        Add a path to the planet.

        :param start: Start node (x, y, direction)
        :param target: Destination node (x, y, direction)
        :param weight: Path weight
        """
        start_x, start_y, start_direction = start
        target_x, target_y, target_direction = target

        start_coordinates = (start_x, start_y)
        target_coordinates = (target_x, target_y)

        if not self._paths.get(start_coordinates):
            self._paths[start_coordinates] = {}
        if not self._paths.get(target_coordinates):
            self._paths[target_coordinates] = {}

        self._paths[start_coordinates][start_direction] = \
            (target_coordinates, target_direction, weight)
        self._paths[target_coordinates][target_direction] = \
            (start_coordinates, start_direction, weight)

    def add_undiscovered_exit(self, exit):
        """
        Add an undiscovered exit to a node on the planet.

        :param exit: Exit node (x,y,direction)
        """
        x, y, direction = exit
        node = (x, y)

        if not self._undiscovered_exits.get(node):
            self._undiscovered_exits[node] = set()
        self._undiscovered_exits[node].add(direction)

    def mark_exit_discovered(self, exit):
        """
        Mark an exit as discovered.

        :param exit: Exit node (x, y, direction)
        """
        x, y, direction = exit
        node = (x, y)

        if node in self._undiscovered_exits and direction in self._undiscovered_exits[node]:
            self._undiscovered_exits[node].remove(direction)

            if not self._undiscovered_exits[node]:
                self._undiscovered_exits.pop(node)

    def get_connected_known_nodes(self, start):
        """
        Transitively obtain a set of nodes connected to some given node.

        :param start: Initial node
        :return: Set of connected nodes
        """
        known_paths = self.get_paths()

        def dfs(node, visited):
            if node not in known_paths:
                return

            for neighbour in [n for n, _, _ in known_paths[node].values()]:
                if neighbour not in visited:
                    visited.add(neighbour)
                    dfs(neighbour, visited)

        connected = set()
        dfs(start, connected)
        return connected

    def get_paths(self):
        """
        Obtain all known paths.

        :return: All known paths
        """
        return self._paths

    def get_undiscovered_exits(self):
        """
        Obtain all undiscovered exits.

        :return: All undiscovered exits
        """
        return self._undiscovered_exits

    def set_starting_node(self, node, color):
        """
        Sets the planets starting node.

        :param node: Starting node (x,y)
        :param color: The starting node's color
        """
        self._start = node
        self._start_color = color

    def shortest_path(self, start, target):
        """
        Obtain shortest path between two nodes (taking path weights into account)

        This method uses Dijkstra's algorithm and a (rather involved) heap-queue
        implementation.

        :param start: Start Node (x,y)
        :param target: Destination node (x,y) of the target node
        :return: Shortest path from start to target node
        """

        if start == target:
            return []

        @total_ordering
        class Vertex:
            def __init__(self, coordinates):
                self.coordinates = coordinates
                self.distance = inf
                self.prev_vertex = None
                self.prev_direction = None
                self.visited = False
                self.valid = True

            def __eq__(self, other):
                if not self.valid and other.valid or self.valid and not other.valid:
                    return False

                return self.distance == other.distance

            def __lt__(self, other):
                if not self.valid and other.valid:
                    return False
                if self.valid and not other.valid:
                    return True
                return self.distance < other.distance

            def predecessors(self):
                class PredecessorIterator:
                    def __init__(self, start):
                        self.vertex = start

                    def __iter__(self):
                        return self

                    def __next__(self):
                        if self.vertex.prev_vertex is None:
                            raise StopIteration

                        ret = self.vertex.prev_vertex.coordinates + \
                              (self.vertex.prev_direction,)

                        self.vertex = self.vertex.prev_vertex

                        return ret

                return PredecessorIterator(self)

        heap = []
        heappointers = {}
        for coordinates in self._paths:
            vertex = Vertex(coordinates)
            if coordinates == start:
                vertex.distance = 0

            heappush(heap, vertex)
            heappointers[coordinates] = vertex

        while heap:
            u = heappop(heap)

            if not u.valid:
                continue
            u.visited = True

            if u.coordinates == target:
                shortest_path = []
                for predecessor in u.predecessors():
                    shortest_path.insert(0, predecessor)
                return shortest_path or None

            paths = self._paths[u.coordinates]
            for outgoing in paths:
                destination, incoming, weight = paths[outgoing]

                if weight == -1:
                    continue

                v = heappointers[destination]
                if v.visited:
                    continue

                if v.distance > u.distance + weight:
                    v.distance = u.distance + weight
                    v.prev_vertex = u
                    v.prev_direction = outgoing

                    updated = copy(v)
                    v.valid = False

                    heappush(heap, updated)
                    heappointers[v.coordinates] = updated

    def to_node(self, position, color, ignore_colors=False):
        """
        Arbitrate a real world position to nearest grid node, taking color of
        current node into account (horizontally/vertically adjacent grid nodes
        never share a color).

        :param position: Real world position (obtained via odometry)
        :param color: Color of current note
        :param ignore_colors: If True, do not use the color of the current node
                              to improve the position estimate.
        """
        x, y = position
        x_next = Planet.NODE_DISTANCE_MM * round(x / Planet.NODE_DISTANCE_MM)
        y_next = Planet.NODE_DISTANCE_MM * round(y / Planet.NODE_DISTANCE_MM)

        x_planet = x_next // Planet.NODE_DISTANCE_MM
        y_planet = y_next // Planet.NODE_DISTANCE_MM

        if not ignore_colors:
            start_x, start_y = self._start
            start_mod2 = (start_x + start_y) % 2
            next_mod2 = (x_planet + y_planet) % 2

            if (color == self._start_color) != (start_mod2 == next_mod2):
                if abs(x_next - x) < abs(y_next - y):
                    y_planet += 1 if y_next - y < 0 else -1
                else:
                    x_planet += 1 if x_next - x < 0 else -1

        return x_planet, y_planet
