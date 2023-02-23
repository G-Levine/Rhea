import numpy as np
import hid
# Requires the hidapi library to be installed.

from data_types import user_input_dtype


class UserInputManager():
    """
    Manages user input from the gamepad.
    """

    def __init__(self):
        self.user_input = np.zeros(1, dtype=user_input_dtype)
        self.device = hid.device()
        for device in hid.enumerate():
            if (device['product_string'] == 'FrSky Simulator') or (device['product_string'] == 'BetaFPV Taranis Joystick') or (device['product_string'] == 'FrSky Taranis Joystick') or (device['vendor_id'] == 1155):
                self.device.open(device['vendor_id'], device['product_id'])
                break
        # if not self.device:
            # raise RuntimeError('No gamepad found.')
        if self.device:
            self.device.set_nonblocking(True)

    def update(self):
        if not self.device:
            return self.user_input
        report = self.device.read(64)
        if report:
            report = [r - 256 if r > 127 else r for r in report]
            self.user_input['start_toggle'] = report[7] > 0
            self.user_input['stand_up_toggle'] = report[10] > 0
            self.user_input['x'] = report[4] / 127.0
            self.user_input['yaw'] = report[3] / 127.0
        return self.user_input


if __name__ == '__main__':
    user_input_manager = UserInputManager()
    while True:
        user_input_manager.update()
        print(user_input_manager.user_input)
