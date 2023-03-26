import numpy as np
import hid
import yaml
# Requires the hidapi library to be installed.

from data_types import user_input_dtype

ALLOWABLE_DEVICES = ['FrSky Simulator', 'BetaFPV Taranis Joystick', 'FrSky Taranis Joystick', 'BETAFPV Joystick']

class UserInputManager():
    """
    Manages user input from the gamepad.
    """

    def __init__(self):
        self.user_input = np.zeros(1, dtype=user_input_dtype)
        self.config = yaml.load(open('config.yaml'), Loader=yaml.FullLoader)
        try:
            self.device = hid.device()
            for device in hid.enumerate():
                if device['product_string'] in ALLOWABLE_DEVICES:
                    self.device.open(device['vendor_id'], device['product_id'])
                    break
            # if not self.device:
                # raise RuntimeError('No gamepad found.')
            if self.device:
                self.device.set_nonblocking(True)
        except:
            self.device = None
            print("No input device")
            self.user_input['start_toggle'] = True
            self.user_input['stand_up_toggle'] = True

    def update(self):
        if not self.device:
            return self.user_input
        report = self.device.read(64)
        if report:
            report = [r - 256 if r > 127 else r for r in report]
            self.user_input['start_toggle'] = report[7] >= 0
            self.user_input['stand_up_toggle'] = report[10] >= 0
            x = report[4] / 127.0
            yaw = report[3] / 127.0
            if abs(x) < self.config['deadzone']:
                x = 0
            if abs(yaw) < self.config['deadzone']:
                yaw = 0
            self.user_input['x'] = x
            self.user_input['yaw'] = yaw
        return self.user_input


if __name__ == '__main__':
    user_input_manager = UserInputManager()
    while True:
        user_input_manager.update()
        print(user_input_manager.user_input)
