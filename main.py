import asyncio
import time
import sys
import yaml

from state_estimator import StateEstimator
# from user_input_manager import UserInputManager
from api_input_manager import UserInputManager
from telemetry_manager import TelemetryManager
from commander import Commander
from controller import Controller

async def main(simulate, telemetry):
    loop_rate = yaml.safe_load(open("config.yaml"))["loop_rate_hz"]
    state_estimator = StateEstimator()
    user_input_manager = UserInputManager()
    commander = Commander()
    controller = Controller()

    if simulate:
        from hardware_interface_simulator import HardwareInterfaceSimulator
        hardware_interface = HardwareInterfaceSimulator()
    else:
        from hardware_interface import HardwareInterface
        hardware_interface = HardwareInterface()
        await hardware_interface.start_all()

    if telemetry:
        telemetry_manager = TelemetryManager()

    i = 0
    curr_time = time.time()
    while True:
        user_input_manager.update()
        state_estimator.update(observation=hardware_interface.observation)
        commander.update(state=state_estimator.state,
                         user_input=user_input_manager.user_input)
        controller.update(state=state_estimator.state,
                          command=commander.command)
        if type(hardware_interface).__name__ == "HardwareInterface":
            await hardware_interface.update(action=controller.action)
        else:
            hardware_interface.update(action=controller.action)

        if i % 10 == 0:
            if telemetry:
                telemetry_manager.update(state=state_estimator.state)
        i %= 10
        last_time = curr_time
        curr_time = time.time()
        await asyncio.sleep(max(0, 1/loop_rate - (curr_time - last_time)))


"""
Main control script for the Rhea robot.
"""
if __name__ == '__main__':
    simulate = "--sim" in sys.argv
    telemetry = "--telemetry" in sys.argv
    asyncio.run(main(simulate, telemetry))