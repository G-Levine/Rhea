import asyncio
import json
import time
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
from foxglove_websocket.types import ChannelId


async def main():
    class Listener(FoxgloveServerListener):
        def on_subscribe(self, server: FoxgloveServer, channel_id: ChannelId):
            print("First client subscribed to", channel_id)

        def on_unsubscribe(self, server: FoxgloveServer, channel_id: ChannelId):
            print("Last client unsubscribed from", channel_id)

    async with FoxgloveServer("0.0.0.0", 8765, "example server") as server:
        server.set_listener(Listener())
        chan_id = await server.add_channel(
            {
                "topic": "joint_states",
                "encoding": "json",
                "schemaName": "JointState",
                "schema": json.dumps(
                    {
                        "type": "object",
                        "properties": {
                            "header": {
                                "type": "object",
                                "properties": {
                                    "stamp": {
                                        "type": "number",
                                    },
                                },
                            },
                            "name": {
                                "type": "string",
                            },
                            "position": {
                                "type": "number",
                            },
                            "velocity": {
                                "type": "number",
                            },
                            "effort": {
                                "type": "number",
                            },
                        },
                    }
                ),
            }
        )

        i = 0
        while True:
            i += 1
            await asyncio.sleep(0.2)
            await server.send_message(
                chan_id,
                time.time_ns(),
                json.dumps({"header": {"stamp": time.time()}, "name": "leg_1_r", "position": 1.0, "velocity": 0.0, "effort": 0.0}).encode("utf8"),
            )


if __name__ == "__main__":
    run_cancellable(main())