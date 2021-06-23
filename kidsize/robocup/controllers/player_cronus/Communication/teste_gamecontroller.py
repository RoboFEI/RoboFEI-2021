
from gamestate import GameState

import json
import os
import socket
import time
import traceback
import sys


class GCListener:
    """A simple UDP socket listening to the GameController broadcast messages"""

    def __init__(self):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(('0.0.0.0', 3838))
        self._socket.setblocking(False)
        self._state = None

    def receive(self):
        try:
            data, peer = self._socket.recvfrom(GameState.sizeof())
        except BlockingIOError:
            return
        except Exception as e:
            print(f'UDP input failure: {e}')
            pass
        if not data:
            print('No UDP data received')
            return
        self._state = GameState.parse(data)

    def getState(self):
        return self._state


system_start = time.time()


gc_listener = GCListener()

finished = False
critical_failure = False
simulated_time = 0

while True:
	try:
		gc_listener.receive()
		print(gc_listener.getState().game_state)
		print(gc_listener.getState())
	except:
		print('nada')
	time.sleep(0.1)



