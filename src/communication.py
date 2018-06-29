import re

from time import sleep, time

from ev3dev.ev3 import Sound

from planet import Direction

class Communication:
    GROUP_ID = '011'
    TIMEOUT = 2

    URL = 'robolab.inf.tu-dresden.de'
    PORT = 8883
    PW = '4BCtilGlZg'

    def __init__(self, mqtt_client, planet=None):
        """
        Connect to MQTT server.
        """

        self._client = mqtt_client

        self._client.username_pw_set(Communication.GROUP_ID,
                                     password=Communication.PW)

        self._client.connect(Communication.URL, Communication.port)

        self.sound=Sound()

        if planet:
            self.set_testplanet(planet)

        sleep(2)

    @staticmethod
    def _abbreviate_direction(direction):
        return {
            Direction.NORTH : 'N',
            Direction.EAST  : 'E',
            Direction.SOUTH : 'S',
            Direction.WEST  : 'W'
        }[direction]

    @staticmethod
    def _expand_direction(direction):
        return {
            'N' : Direction.NORTH,
            'E' : Direction.EAST,
            'S' : Direction.SOUTH,
            'W' : Direction.WEST
        }[direction]

    def _on_ready(self, client, userdata, message):
        """
        Initial server callback function.

        :param client: Unused
        :param userdata: Unused
        :param message: Server message
        """
        payload = message.payload.decode('utf-8')

        if __debug__:
            print("server sent '{}'".format(payload))

        # ignore echoes of messages sent to server
        if not payload.startswith('ACK'):
            return
        else:
            payload = payload[4:]
            self._received_first = True

        match = re.match(r'^(.*) (-?\d+),(-?\d+)$', payload)
        if match:
            groups = match.groups()
            self._planet = groups[0]
            self._client.subscribe('planet/' + self._planet, qos=1)
            self._start = tuple(map(int, groups[1:]))
        else:
            err = 'expected start coordinates but received: "{}"'
            raise ValueError(err.format(payload))

    def _on_path(self, client, userdata, message):
        """
        Server callback function for receiving path data.

        :param client: Unused
        :param userdata: Unused
        :param message: Server message
        """
        self._t0 = time()

        payload = message.payload.decode('utf-8')

        if __debug__:
            print("server sent '{}'".format(payload))

        # ignore message originally sent to server
        if not payload.startswith('ACK'):
            return
        else:
            payload = payload[4:]
            self._received_first = True

        # receive path correction and optionally additional paths
        match = re.match(
            r'^path (-?\d+),(-?\d+),([NESW]) (-?\d+),(-?\d+),([NESW]) (free|blocked) (\d+|-1)$',
            payload)

        if match:
            groups = match.groups()

            x1, y1 = map(int, groups[:2])
            dir1 = Communication._expand_direction(groups[2])

            x2, y2 = map(int, groups[3:5])
            dir2 = Communication._expand_direction(groups[5])

            blockage = groups[6] == 'blocked'

            weight = int(groups[7])

            if (x1, y1, dir1) == self._start:
                if __debug__ and (x2, y2, dir2) != self._destination:
                    print("WARNING: received path correction from server")

                self._correct_destination = (x2, y2, dir2)
                self._correct_blockage = blockage
                self._correct_weight = weight

            else: # adding paths to a planet is idempotent so duplicate paths
                  # do not need to be handled separately here
                self._additional_paths.append(
                    ((x1, y1, dir1), (x2, y2, dir2), weight))
            return

        # optionally receive a target
        match = re.match(r'^target (-?\d+),(-?\d+)$', payload)
        if match:
            target = tuple(map(int, match.groups()))
            if self._additional_target and target != self._additional_target:
                raise ValueError('received multiple targets in same session')

            self._additional_target = target

            return

        err = "received malformed server message: '{}'"
        raise ValueError(err.format(payload))

    def _pass(self, client, userdata, message):
        """
        No-op Server callback function.

        :param client: Unused
        :param userdata: Unused
        :param message: Unused
        """
        pass

    def send_message(self, topic, payload, on_message=None, wait_for_first=False):
        """
        Send a message to MQTT server.

        :param topic: MQTT topic
        :param payload: Payload to be sent to the server
        :param on_message: Callback function
        """
        self._client.subscribe(topic, qos=1)
        self._client.on_message = on_message or self._pass

        if __debug__:
            print("sending '{}' to server (topic is {})".format(payload, topic))

        self._client.loop_start()
        self._received_first = not wait_for_first
        self._t0 = time()
        self._client.publish(topic, payload=payload, qos=1)

        while not self._received_first or time() - self._t0 < Communication.TIMEOUT:
            pass

        self._client.loop_stop()

    def send_ready(self):
        """
        Send ready signal to the MQTT server.

        :return: start node coordinates returned by server.
        """
        self._start = None

        self.send_message('explorer/' + Communication.GROUP_ID, 'SYN ready',
                          on_message=self._on_ready, wait_for_first=True)

        return self._start

    def send_path(self, start, destination, blocked=False):
        """
        Send a discovered path to the MQTT server.

        :param start: Coordinates (x, y, direction) of the starting node
        :param destination: Coordinates (x,y, direction) of the destination node
        :param blocked: True if path was blocked
        """
        self._start = start
        self._destination = destination
        self._correct_destination = None
        self._correct_blockage = None
        self._correct_weight = None
        self._additional_paths = []
        self._additional_target = None

        start_x, start_y, start_dir = start
        destination_x, destination_y, destination_dir = destination
        payload = 'SYN path {},{},{} {},{},{} {}'.format(
            start_x, start_y, self._abbreviate_direction(start_dir),
            destination_x, destination_y,
            self._abbreviate_direction(destination_dir),
            'free' if not blocked else 'blocked')

        self.send_message('planet/' + self._planet, payload, self._on_path,
                          wait_for_first=True)

        return self._correct_destination, self._correct_weight, \
               self._correct_blockage, \
               self._additional_paths, self._additional_target

    def send_target_reached(self, message=None):
        """
        Indicate to the MQTT server that the target has been reached.

        :param message: Arbitrary additional message to send to the server
        """
        payload = "target reached!"
        if message:
            payload += " " + message

        self.send_message('planet/' + self._planet, payload)

    def send_exploration_completed(self, message=None):
        """
        Indicate to the MQTT server that exploration is completed.

        :param message: Arbitrary additional message to send to the server
        """
        payload = "exploration completed!"
        if message:
            payload += " " + message

        self.send_message('planet/' + self._planet, payload)

    def set_testplanet(self, testplanet):
        """
        Indicate the name of the current testplanet to the MQTT server.
        (Should only be used during testruns).

        :param testplanet: Test planet name
        """
        self.send_message('explorer/' + Communication.GROUP_ID,
                          'testplanet ' + testplanet)
