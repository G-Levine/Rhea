import numpy as np
import hid
import yaml
# Requires the hidapi library to be installed.

from data_types import user_input_dtype

import zmq
import time

# Create the ZMQ sockets
# The API client socket is used to receive commands from the API client thread
api_client_context = zmq.Context()
api_client_socket = api_client_context.socket(zmq.REP)
api_client_socket.bind("tcp://*:5555")

# The collision monitor socket is used to receive collision notifications from the collision monitor thread
collision_monitor_context = zmq.Context()
collision_monitor_socket = collision_monitor_context.socket(zmq.REP)
collision_monitor_socket.bind("tcp://*:5556")

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
        # Check if the command has timed out
        if self.command_end_time < time.time():
            self.user_input['x'] = 0
            self.user_input['yaw'] = 0

        # Receive command from the API client thread if available
        try:
            message = api_client_socket.recv(zmq.NOBLOCK)
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
            api_client_socket.send(response)
        except zmq.Again:
            pass

        # Receive collision notification from the collision monitor thread if available
        try:
            message = collision_monitor_socket.recv(zmq.NOBLOCK)
            message = message.decode('utf8')
            if "collision" in message:
                # If collided, stop moving and turn around
                self.user_input['x'] = -0.5
                self.user_input['yaw'] = 1.57
                self.command_end_time = time.time() + 1.0
            response = b"Got collision notification"
            collision_monitor_socket.send(response)
        except zmq.Again:
            pass

        return self.user_input


if __name__ == '__main__':
    user_input_manager = UserInputManager()
    while True:
        user_input_manager.update()
        print(user_input_manager.user_input)
        time.sleep(1.0)
