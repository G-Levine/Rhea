import numpy as np
import hid
import yaml
# Requires the hidapi library to be installed.

from data_types import user_input_dtype

import zmq
import time

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

class UserInputManager():
    """
    Manages user input from the gamepad.
    """

    def __init__(self):
        self.user_input = np.zeros(1, dtype=user_input_dtype)
        self.config = yaml.load(open('config.yaml'), Loader=yaml.FullLoader)
        self.user_input['start_toggle'] = True
        self.user_input['stand_up_toggle'] = True
        self.command_end_time = time.time()

    def update(self):
        if self.command_end_time < time.time():
            self.user_input['x'] = 0
            self.user_input['yaw'] = 0
        try:
            message = socket.recv(zmq.NOBLOCK)
            message = message.decode('utf8')
            if "stand" in message:
                self.user_input['stand_up_toggle'] = True
            elif "sit" in message:
                self.user_input['stand_up_toggle'] = False
            elif "move" in message:
                command_substrings = message.split("_")
                self.user_input['x'] = command_substrings[1]
                self.user_input['yaw'] = command_substrings[2]
                self.command_end_time = time.time() + float(command_substrings[3])
            response = b"Got command"
            socket.send(response)
        except zmq.Again:
            pass
        return self.user_input


if __name__ == '__main__':
    user_input_manager = UserInputManager()
    while True:
        user_input_manager.update()
        print(user_input_manager.user_input)
        time.sleep(1.0)
